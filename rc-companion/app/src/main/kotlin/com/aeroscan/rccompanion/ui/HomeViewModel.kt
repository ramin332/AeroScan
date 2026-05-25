package com.aeroscan.rccompanion.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.cloud.PlyParser
import com.aeroscan.rccompanion.cloud.PlyVoxelDownsample
import com.aeroscan.rccompanion.filepick.PickedFile
import com.aeroscan.rccompanion.mop.AugmentSession
import com.aeroscan.rccompanion.mop.StatusSession
import com.aeroscan.rccompanion.wpml.WpmlParser
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

/**
 * State machine for the augment flow:
 *
 *   Idle
 *    └── onFilePicked → Picked
 *         └── augment() → ParsingKmz → Uploading → AwaitingPreview
 *                              └── PreviewReceived → ReviewReady
 *                                       ├── approve() → Approving → ReadyToFly
 *                                       └── reject()  → Idle
 *                              └── error            → Error
 *
 * The pilot's tap-to-fly happens on Pilot 2's Custom Widget, NOT in this app —
 * once we hit ReadyToFly, the rc-companion's job is done. The card in that
 * state nudges the pilot to switch to Pilot 2.
 */

/**
 * Readiness-banner state for the Manifold augment service. Surfaced from
 * [HomeViewModel.checkStatus] via [HomeViewModel.banner] and rendered by
 * HomeScreen. Drives a single coloured banner the pilot reads BEFORE augmenting.
 */
sealed class BannerState {
    /** No check has run yet (initial). */
    data object Idle : BannerState()

    /** A PING is in flight. */
    data object Checking : BannerState()

    /** Env healthy + mesh present — augment will work. (green) */
    data class Ready(val label: String) : BannerState()

    /** Env healthy but the latest flight has no mesh — won't augment. (red) */
    data class NoMesh(val label: String) : BannerState()

    /** The Manifold's Python env probe failed — augment would crash. (amber) */
    data class EnvError(val label: String) : BannerState()

    /** No STAT reply (app down, link down, old Manifold). (grey) */
    data class Unreachable(val label: String) : BannerState()
}

/**
 * Pure mapping from a [StatusSession.Result] to a [BannerState]. Priority is
 * EnvError > NoMesh > Ready so the most blocking problem wins the banner.
 * Kept top-level + side-effect-free so it's unit-testable without the MSDK.
 */
fun bannerFor(result: StatusSession.Result): BannerState = when (result) {
    is StatusSession.Result.Unreachable ->
        BannerState.Unreachable("Manifold not reachable — is the app running? (${result.reason})")

    is StatusSession.Result.Ok -> {
        val s = result.status
        when {
            !s.envOk -> BannerState.EnvError("env error: ${s.envDetail}")
            !s.meshPresent -> BannerState.NoMesh("Not ready — run auto-explore first ${s.latestFlight}")
            else -> {
                val pts = if (s.nPoints >= 1_000_000) "%.1fM pts".format(s.nPoints / 1e6)
                    else "${s.nPoints} pts"
                BannerState.Ready(
                    "Ready · ${s.latestFlight} · mesh ✓ (${s.meshChunks} chunks, $pts) · env ✓ · %.1f GB free"
                        .format(s.blackboxFreeGb),
                )
            }
        }
    }
}

class HomeViewModel(app: Application) : AndroidViewModel(app) {

    /** Per-WP voxel size used to downsample the KMZ's cloud.ply before
     * shipping. 1 m is the empirical sweet spot from the bench (see
     * scripts/bench_icp_target_density.py + validate_2m_voxel_e2e.py): yields
     * ~440 KB gzipped fingerprint, ICP transform delta < 0.005°, 27% gimbal
     * disagreement vs full-cloud (all aimed at valid building surfaces). */
    private val voxelSizeM: Double = 1.0

    sealed interface UiState {
        data object Idle : UiState
        data class Picked(val file: PickedFile) : UiState
        data class ParsingKmz(val file: PickedFile) : UiState
        data class Uploading(val file: PickedFile, val sent: Long, val total: Long) : UiState
        data class AwaitingPreview(val file: PickedFile, val elapsedSec: Long = 0) : UiState
        data class ReviewReady(
            val file: PickedFile,
            val summary: PreviewSummary,
            val augmentedKmz: ByteArray,
            /** Where the augmented KMZ was persisted, or null if save failed. */
            val savedKmzPath: String?,
        ) : UiState
        data class Approving(val file: PickedFile, val summary: PreviewSummary) : UiState
        data class ReadyToFly(val file: PickedFile, val summary: PreviewSummary) : UiState
        data class Error(val file: PickedFile?, val message: String) : UiState
    }

    /** Compact view of the summary JSON for the UI — only the fields the
     *  pilot reviews. Decoded from the PRVW frame's summary slab. */
    data class PreviewSummary(
        val name: String,
        val waypointCount: Int,
        val waypointsAimed: Int,
        val facadeCount: Int,
        val pitchMin: Double,
        val pitchMax: Double,
        val pitchMedian: Double,
        val anomalyPitchUp: Int,
        val anomalyPitchDown: Int,
        val anomalyIndicesPitchUp: IntArray,
        val anomalyIndicesPitchDown: IntArray,
        val icpRmseM: Double,
        val elapsedSec: Double,
    ) {
        companion object {
            fun fromJson(jsonBytes: ByteArray): PreviewSummary {
                val obj = JSONObject(String(jsonBytes, Charsets.UTF_8))
                val gs = obj.getJSONObject("gimbal_stats")
                val pitch = gs.getJSONObject("pitch_deg")
                val ac = gs.getJSONObject("anomaly_counts")
                val ai = gs.getJSONObject("anomaly_indices")
                val icp = obj.getJSONObject("icp")
                return PreviewSummary(
                    name = obj.optString("name", "Mission"),
                    waypointCount = obj.getInt("waypoints_total"),
                    waypointsAimed = obj.getInt("waypoints_aimed"),
                    facadeCount = obj.getInt("facades"),
                    pitchMin = pitch.getDouble("min"),
                    pitchMax = pitch.getDouble("max"),
                    pitchMedian = pitch.getDouble("median"),
                    anomalyPitchUp = ac.getInt("pitch_up"),
                    anomalyPitchDown = ac.getInt("pitch_down"),
                    anomalyIndicesPitchUp = jsonIntArray(ai.getJSONArray("pitch_up")),
                    anomalyIndicesPitchDown = jsonIntArray(ai.getJSONArray("pitch_down")),
                    icpRmseM = icp.getDouble("icp_rmse_m"),
                    elapsedSec = obj.getDouble("elapsed_s"),
                )
            }

            private fun jsonIntArray(arr: org.json.JSONArray): IntArray {
                val out = IntArray(arr.length())
                for (i in 0 until arr.length()) out[i] = arr.getInt(i)
                return out
            }
        }
    }

    private val _ui = MutableStateFlow<UiState>(UiState.Idle)
    val ui: StateFlow<UiState> = _ui.asStateFlow()

    /** Manifold readiness banner — driven by [checkStatus]. */
    private val _banner = MutableStateFlow<BannerState>(BannerState.Idle)
    val banner: StateFlow<BannerState> = _banner.asStateFlow()

    val connection: StateFlow<Connection.State> = Connection.state

    private var augmentJob: Job? = null
    private var session: AugmentSession? = null
    private var statusJob: Job? = null

    fun onFilePicked(picked: PickedFile) {
        _ui.value = UiState.Picked(picked)
    }

    fun reset() {
        augmentJob?.cancel()
        augmentJob = null
        session = null
        _ui.value = UiState.Idle
    }

    fun cancel() {
        augmentJob?.cancel()
    }

    /** Pick → parse → uplink → wait. */
    fun augment() {
        val current = _ui.value
        val picked = when (current) {
            is UiState.Picked -> current.file
            is UiState.Error -> current.file
            else -> return
        } ?: return

        if (connection.value !is Connection.State.AircraftConnected) {
            _ui.value = UiState.Error(picked, "M4E not connected. Power on the aircraft and pair the RC.")
            return
        }

        // Fail fast on a known-bad readiness state (from the PING/STAT banner) —
        // don't upload a KMZ and run a doomed ~2-min augment when the Manifold has
        // already told us it can't succeed (no mesh / bad env / unreachable).
        val blockReason: String? = when (val b = banner.value) {
            is BannerState.NoMesh -> "No mesh on the latest flight — fly a Smart3D scan, then augment."
            is BannerState.EnvError -> "Manifold env not ready (${b.label}). Augment can't run."
            is BannerState.Unreachable -> "Manifold not reachable — start the AeroScan app on the drone, then retry."
            else -> null  // Idle / Checking / Ready — proceed
        }
        if (blockReason != null) {
            _ui.value = UiState.Error(picked, blockReason)
            return
        }

        augmentJob = viewModelScope.launch {
            _ui.value = UiState.ParsingKmz(picked)
            val ctx = getApplication<Application>().applicationContext

            // 1. Read picked KMZ → parse waylines + extract cloud.ply.
            val kmzBytes = withContext(Dispatchers.IO) {
                runCatching {
                    ctx.contentResolver.openInputStream(picked.uri)?.use { it.readBytes() }
                }.getOrNull()
            }
            if (kmzBytes == null) {
                _ui.value = UiState.Error(picked, "Could not read picked file. Try re-selecting.")
                return@launch
            }
            val parseResult = withContext(Dispatchers.Default) {
                runCatching {
                    WpmlParser.parseKmz(kmzBytes, missionName = picked.displayName.removeSuffix(".kmz"))
                }
            }
            val intent = parseResult.getOrNull()?.intent
            val cloudBytes = parseResult.getOrNull()?.cloudPlyBytes
            if (intent == null) {
                _ui.value = UiState.Error(picked,
                    "KMZ parse failed: ${parseResult.exceptionOrNull()?.message ?: "unknown"}")
                return@launch
            }
            if (cloudBytes == null) {
                _ui.value = UiState.Error(picked,
                    "KMZ has no cloud.ply — required as the ICP target.")
                return@launch
            }

            // 2. Voxel-downsample the cloud at 1 m.
            val fingerprintBytes = withContext(Dispatchers.Default) {
                runCatching {
                    val pc = PlyParser.read(cloudBytes)
                        ?: error("cloud.ply could not be parsed as binary PLY")
                    val ds = PlyVoxelDownsample.downsample(pc, voxelSizeM)
                    PlyParser.writeBinary(ds)
                }
            }.getOrNull()
            if (fingerprintBytes == null) {
                _ui.value = UiState.Error(picked, "cloud.ply downsample failed.")
                return@launch
            }

            // 3. Open MOP session, send AUGM, await PRVW.
            _ui.value = UiState.Uploading(picked, 0,
                total = (intent.toJsonString().toByteArray().size + fingerprintBytes.size).toLong())
            val sess = AugmentSession()
            session = sess

            // Subscribe to events to update the UI.
            launch {
                sess.events.collect { ev ->
                    when (ev) {
                        is AugmentSession.Event.Connecting -> { /* keep state */ }
                        is AugmentSession.Event.Sending -> {
                            (_ui.value as? UiState.Uploading)?.let {
                                _ui.value = it.copy(sent = ev.bytesSent, total = ev.totalBytes)
                            }
                        }
                        is AugmentSession.Event.SendComplete -> {
                            _ui.value = UiState.AwaitingPreview(picked)
                        }
                        is AugmentSession.Event.WaitingForPreview -> {
                            _ui.value = UiState.AwaitingPreview(picked)
                        }
                        is AugmentSession.Event.PreviewReceived -> {
                            val summary = runCatching { PreviewSummary.fromJson(ev.summaryJson) }
                                .getOrElse {
                                    _ui.value = UiState.Error(picked,
                                        "Preview JSON parse failed: ${it.message}")
                                    return@collect
                                }
                            // Persist the augmented KMZ + summary to app-private
                            // external storage so the pilot can transfer them off
                            // the RC via USB (visible under Android/data/<pkg>/files/
                            // missions/) without root or extra permissions. Survives
                            // the in-memory UiState — pilot can recover the artifact
                            // even after Reject.
                            val savedPath = saveAugmentedKmz(
                                ctx, picked.displayName, ev.augmentedKmz, ev.summaryJson,
                            )
                            _ui.value = UiState.ReviewReady(picked, summary, ev.augmentedKmz, savedPath)
                        }
                        is AugmentSession.Event.ExecuteSent -> {
                            // approve() flipped state to Approving before sending
                            // EXEC, so by the time this event lands the state is
                            // Approving (NOT ReviewReady). Transition from there.
                            val approving = _ui.value as? UiState.Approving
                            if (approving != null) {
                                _ui.value = UiState.ReadyToFly(approving.file, approving.summary)
                            }
                        }
                        is AugmentSession.Event.Cancelled -> {
                            if (_ui.value !is UiState.Idle) _ui.value = UiState.Idle
                        }
                        is AugmentSession.Event.Failed -> {
                            _ui.value = UiState.Error(picked,
                                ev.cause.message ?: "Augment failed.")
                        }
                        is AugmentSession.Event.Closed -> { /* terminal; UI already set */ }
                    }
                }
            }

            sess.sendAndAwaitPreview(intent, fingerprintBytes)
        }
    }

    /** Pilot tapped APPROVE on the preview — ship EXEC, kmz_runner uploads to aircraft. */
    fun approve() {
        val rr = _ui.value as? UiState.ReviewReady ?: return
        val sess = session ?: return
        viewModelScope.launch {
            _ui.value = UiState.Approving(rr.file, rr.summary)
            sess.approve(missionId = rr.summary.name)
            // ExecuteSent event → ReadyToFly transition wired in the events
            // collector above.
        }
    }

    /**
     * Run a one-shot Manifold readiness check (PING → STAT) and map the result
     * onto [banner]. Bounded by [StatusSession.query]'s own timeout.
     *
     * Skipped (no-op, banner left as-is) while an augment is in flight — the
     * augment session owns the MOP channel and a concurrent status PING would
     * collide with it. Also short-circuits to [BannerState.Unreachable] if the
     * aircraft link isn't up, since the PING can't reach the Manifold without it.
     */
    fun checkStatus() {
        if (session != null || augmentJob?.isActive == true) return
        if (statusJob?.isActive == true) return

        if (connection.value !is Connection.State.AircraftConnected) {
            _banner.value = bannerFor(
                StatusSession.Result.Unreachable("M4E not connected — power on the aircraft and pair the RC"),
            )
            return
        }

        statusJob = viewModelScope.launch {
            _banner.value = BannerState.Checking
            _banner.value = bannerFor(StatusSession().query())
        }
    }

    /** Pilot tapped REJECT — drop the session, return to Idle. */
    fun reject() {
        val sess = session
        viewModelScope.launch {
            sess?.closeAndRelease()
            session = null
            _ui.value = UiState.Idle
        }
    }

    private fun saveAugmentedKmz(
        ctx: android.content.Context,
        sourceName: String,
        kmzBytes: ByteArray,
        summaryJson: ByteArray,
    ): String? {
        return try {
            // App-scoped external storage: /sdcard/Android/data/<pkg>/files/missions/
            // Visible via Files app + USB. No special permissions needed (API 19+).
            // Survives app updates; cleared on uninstall.
            val baseDir = ctx.getExternalFilesDir(null) ?: ctx.filesDir
            val missionsDir = File(baseDir, "missions").apply { mkdirs() }

            val tsFmt = SimpleDateFormat("yyyyMMdd'T'HHmmss'Z'", Locale.US).apply {
                timeZone = TimeZone.getTimeZone("UTC")
            }
            val ts = tsFmt.format(Date())
            // Strip any path components from the picked filename and the .kmz suffix.
            val baseName = File(sourceName).nameWithoutExtension.take(40)

            val kmzFile = File(missionsDir, "${ts}_${baseName}.augmented.kmz")
            kmzFile.writeBytes(kmzBytes)
            File(missionsDir, "${ts}_${baseName}.summary.json").writeBytes(summaryJson)

            kmzFile.absolutePath
        } catch (t: Throwable) {
            android.util.Log.e("HomeViewModel", "saveAugmentedKmz failed", t)
            null
        }
    }
}
