package com.aeroscan.rccompanion.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.cloud.PlyParser
import com.aeroscan.rccompanion.cloud.PlyVoxelDownsample
import com.aeroscan.rccompanion.filepick.PickedFile
import com.aeroscan.rccompanion.mop.AugmentSession
import com.aeroscan.rccompanion.mop.AugmentFraming
import com.aeroscan.rccompanion.mop.StatusSession
import com.aeroscan.rccompanion.wpml.WpmlParser
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.async
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
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
/** Re-check period while the Manifold is not reachable / not ready. */
const val READINESS_POLL_MS = 5_000L

/**
 * How long to wait before the next automatic readiness check, or null to stop.
 *
 * Why polling: the AeroScan app on the Manifold is started by the pilot from
 * DJI Pilot 2 (camera view → payload panel → enable), and it takes 10–30 s
 * after that to bind its MOP channel (measured on every 2026-07-10 session).
 * A single check at aircraft-connect time lands inside that window, so the
 * banner stayed "Unreachable" until someone tapped Retry — the "have to send a
 * message first" folklore. NoMesh and EnvError also change on their own (a
 * scan lands, the env comes up), so they keep polling too. Ready stops.
 */
fun nextPollDelayMs(state: BannerState): Long? = when (state) {
    is BannerState.Unreachable, is BannerState.NoMesh, is BannerState.EnvError -> READINESS_POLL_MS
    is BannerState.Ready, BannerState.Checking, BannerState.Idle -> null
}

/** One line about the recorded mission, appended to any STAT-derived banner. */
fun missionSuffix(s: AugmentFraming.ManifoldStatus): String = when (s.missionState) {
    "interrupted" -> if (s.missionResumeFrom >= 0)
        " · ⏸ ${s.missionId} interrupted at WP ${s.missionLastIndex}/${s.missionTotal} — Continue on Pilot 2 after the swap"
    else " · ⏸ ${s.missionId} stopped at WP ${s.missionLastIndex} — Fly restarts it"
    "flying", "paused" -> " · ✈ ${s.missionId} ${s.missionState} at WP ${s.missionLastIndex}/${s.missionTotal}"
    "completed" -> " · ✅ ${s.missionId} completed ${s.missionLastIndex}/${s.missionTotal}"
    "ready" -> " · ▶ ${s.missionId} uploaded — tap Fly on Pilot 2"
    else -> ""
}

/** "8 w", "3 d", "5 h", "20 min" — how old the newest scan is. */
fun ageText(seconds: Long): String = when {
    seconds < 0 -> "?"
    seconds < 3600 -> "${seconds / 60} min"
    seconds < 2 * 86400 -> "${seconds / 3600} h"
    seconds < 14 * 86400 -> "${seconds / 86400} d"
    else -> "${seconds / (7 * 86400)} w"
}

fun bannerFor(result: StatusSession.Result): BannerState = when (result) {
    is StatusSession.Result.Unreachable ->
        BannerState.Unreachable(
            "AeroScan app not running on the drone. In DJI Pilot 2: camera view → payload panel → " +
                "enable AeroScan (psdk-demo). It needs ~30 s to come up; this banner re-checks every 5 s. " +
                "(${result.reason})",
        )

    is StatusSession.Result.Ok -> {
        val s = result.status
        val mission = missionSuffix(s)
        when {
            !s.envOk -> BannerState.EnvError("env error: ${s.envDetail}$mission")
            !s.meshPresent -> BannerState.NoMesh(
                if (s.meshStale)
                    "Newest scan ${s.meshFlight} is ${ageText(s.meshAgeS)} old — augment will use it; " +
                        "fly a new Smart3D scan for a current model$mission"
                else "No scan on the drone — fly a Smart3D scan, then augment$mission",
            )
            else -> {
                val pts = if (s.nPoints >= 1_000_000) "%.1fM pts".format(s.nPoints / 1e6)
                    else "${s.nPoints} pts"
                BannerState.Ready(
                    "Ready · ${s.latestFlight} · mesh ✓ (${s.meshChunks} chunks, $pts) · env ✓ · %.1f GB free"
                        .format(s.blackboxFreeGb) + mission,
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
        // --- honest plan metrics (augment summary since 2026-09-02; all optional) ---
        val gsdMedianMmPx: Double? = null,
        val gsdTargetMmPx: Double? = null,
        val stopAtWaypoint: Boolean = true,
        val flips: Int = 0,
        val farPicks: Int = 0,
        val unaimed: Int = 0,
        val warnings: Int = 0,
        val errors: Int = 0,
        val validationMessages: List<String> = emptyList(),
        val validationIndices: IntArray = IntArray(0),
        /** Facade rectangles in mission ENU, for the mission view. Empty on older Manifolds. */
        val facadeGeom: List<FacadeQuad> = emptyList(),
        /** Facade each waypoint is aimed at, −1 = none. Empty on older Manifolds. */
        val wpTargets: IntArray = IntArray(0),
    ) {
        /** Every waypoint the pilot should look at on the map. */
        val flaggedIndices: Set<Int>
            get() = (anomalyIndicesPitchUp.toList() + anomalyIndicesPitchDown.toList() + validationIndices.toList()).toSet()

        companion object {
            fun fromJson(jsonBytes: ByteArray): PreviewSummary {
                val obj = JSONObject(String(jsonBytes, Charsets.UTF_8))
                val gs = obj.getJSONObject("gimbal_stats")
                val pitch = gs.getJSONObject("pitch_deg")
                val ac = gs.getJSONObject("anomaly_counts")
                val ai = gs.getJSONObject("anomaly_indices")
                val icp = obj.getJSONObject("icp")
                val gsd = obj.optJSONObject("gsd")
                val aim = obj.optJSONObject("aim")
                val validation = obj.optJSONArray("validation")
                val geom = obj.optJSONArray("facade_geom")
                val quads = ArrayList<FacadeQuad>(geom?.length() ?: 0)
                if (geom != null) for (i in 0 until geom.length()) {
                    val g = geom.getJSONObject(i)
                    val vs = g.optJSONArray("v") ?: continue
                    val v = DoubleArray(vs.length() * 3)
                    for (c in 0 until vs.length()) {
                        val corner = vs.getJSONArray(c)
                        v[3 * c] = corner.getDouble(0); v[3 * c + 1] = corner.getDouble(1); v[3 * c + 2] = corner.getDouble(2)
                    }
                    val ns = g.optJSONArray("n")
                    val nv = DoubleArray(3)
                    if (ns != null) for (k in 0 until minOf(3, ns.length())) nv[k] = ns.getDouble(k)
                    quads.add(FacadeQuad(v, nv, 0))
                }
                val tgt = obj.optJSONArray("wp_target")
                val targets = IntArray(tgt?.length() ?: 0) { tgt!!.getInt(it) }
                var warnings = 0; var errors = 0
                val messages = ArrayList<String>()
                val vIdx = ArrayList<Int>()
                if (validation != null) for (i in 0 until validation.length()) {
                    val v = validation.getJSONObject(i)
                    when (v.optString("severity")) {
                        "warning" -> warnings++
                        "error" -> errors++
                        else -> continue
                    }
                    messages.add(v.optString("message", v.optString("code")))
                    v.optJSONArray("waypoint_indices")?.let { arr ->
                        for (j in 0 until arr.length()) vIdx.add(arr.getInt(j))
                    }
                }
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
                    gsdMedianMmPx = gsd?.optDouble("median_mm_per_px")?.takeIf { !it.isNaN() },
                    gsdTargetMmPx = gsd?.optDouble("target_mm_per_px")?.takeIf { !it.isNaN() },
                    stopAtWaypoint = obj.optBoolean("stop_at_waypoint", true),
                    flips = aim?.optInt("reversals_gt90", 0) ?: 0,
                    farPicks = aim?.optInt("far_picks", 0) ?: 0,
                    unaimed = aim?.optInt("unaimed", 0) ?: 0,
                    warnings = warnings,
                    errors = errors,
                    validationMessages = messages,
                    validationIndices = vIdx.toIntArray(),
                    facadeGeom = quads,
                    wpTargets = targets,
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

    /** Last parsed STAT, for the "replace interrupted mission?" confirmation. */
    private var lastStatus: AugmentFraming.ManifoldStatus? = null

    /** Last STAT for the status strip (null until the first successful check). */
    private val _status = MutableStateFlow<AugmentFraming.ManifoldStatus?>(null)
    val status: StateFlow<AugmentFraming.ManifoldStatus?> = _status.asStateFlow()

    /** What the map draws: the picked KMZ as soon as it is parsed, re-aimed headings after the preview. */
    private val _map = MutableStateFlow<MissionMapData?>(null)
    val map: StateFlow<MissionMapData?> = _map.asStateFlow()

    /** Picked KMZ parsed once at pick time (map + augment share it). */
    private var parseJob: Deferred<WpmlParser.ParseResult?>? = null
    private var parsedCloud: PlyParser.XyzCloud? = null

    /** Mission id the pilot has already agreed to replace (two-tap confirm). */
    private var replaceAcknowledgedFor: String? = null

    val connection: StateFlow<Connection.State> = Connection.state
    val linkDrops: StateFlow<List<Long>> = Connection.drops
    val linkUpSince: StateFlow<Long?> = Connection.upSince

    private var augmentJob: Job? = null
    private var session: AugmentSession? = null
    private var statusJob: Job? = null

    fun onFilePicked(picked: PickedFile) {
        parseJob?.cancel()
        _map.value = null
        parsedCloud = null
        _ui.value = UiState.ParsingKmz(picked)
        val ctx = getApplication<Application>().applicationContext
        val job = viewModelScope.async(Dispatchers.Default) {
            val bytes = withContext(Dispatchers.IO) {
                runCatching { ctx.contentResolver.openInputStream(picked.uri)?.use { it.readBytes() } }.getOrNull()
            } ?: return@async null
            runCatching {
                WpmlParser.parseKmz(bytes, missionName = picked.displayName.removeSuffix(".kmz"))
            }.getOrNull()
        }
        parseJob = job
        viewModelScope.launch {
            val parsed = job.await()
            if (_ui.value !is UiState.ParsingKmz) return@launch
            if (parsed == null) {
                _ui.value = UiState.Error(picked, "Could not read ${picked.displayName} as a DJI KMZ (needs wpmz/waylines.wpml).")
                return@launch
            }
            val cloud = withContext(Dispatchers.Default) {
                parsed.cloudPlyBytes?.let { runCatching { PlyParser.read(it) }.getOrNull() }
            }
            parsedCloud = cloud
            _map.value = withContext(Dispatchers.Default) { buildMissionMap(parsed.intent, cloud) }
            _ui.value = UiState.Picked(picked)
        }
    }

    fun reset() {
        augmentJob?.cancel()
        augmentJob = null
        session = null
        _map.value = null
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

        // Fail fast on a known-bad readiness state (from the PING/STAT strip) —
        // don't upload a KMZ and run a doomed ~2-min augment when the Manifold has
        // already told us it can't succeed. An old scan is NOT a blocker: the chip
        // shows its age and the Manifold's age gate is bypassed on purpose.
        val blockReason = augmentBlockReason(
            connection.value, banner.value, lastStatus,
            drops = Connection.drops.value, upSinceMs = Connection.upSince.value,
        )
        if (blockReason != null) {
            _ui.value = UiState.Error(picked, blockReason)
            return
        }
        // A new augment always replaces the recorded mission on the Manifold.
        // If that mission is interrupted (battery swap), make the pilot say so
        // twice: the first tap explains, the second proceeds.
        val st = lastStatus
        if (st != null && st.interrupted && replaceAcknowledgedFor != st.missionId) {
            replaceAcknowledgedFor = st.missionId
            _ui.value = UiState.Error(
                picked,
                "Mission ${st.missionId} is interrupted at WP ${st.missionLastIndex}/${st.missionTotal}. " +
                    "To finish it, tap Continue then Fly on Pilot 2. To replace it with this new mission, tap Augment again.",
            )
            return
        }

        augmentJob = viewModelScope.launch {
            val ctx = getApplication<Application>().applicationContext

            // 1. The KMZ was parsed at pick time; reuse it (re-parse on retry after an error).
            val parseResult = parseJob?.let { runCatching { it.await() }.getOrNull() }
                ?: withContext(Dispatchers.IO) {
                    runCatching {
                        ctx.contentResolver.openInputStream(picked.uri)?.use { it.readBytes() }
                    }.getOrNull()
                }?.let { bytes ->
                    withContext(Dispatchers.Default) {
                        runCatching {
                            WpmlParser.parseKmz(bytes, missionName = picked.displayName.removeSuffix(".kmz"))
                        }.getOrNull()
                    }
                }
            val intent = parseResult?.intent
            val cloudBytes = parseResult?.cloudPlyBytes
            if (intent == null) {
                _ui.value = UiState.Error(picked, "KMZ parse failed — pick the file again.")
                return@launch
            }
            if (cloudBytes == null) {
                _ui.value = UiState.Error(picked,
                    "KMZ has no cloud.ply — required as the ICP target.")
                return@launch
            }
            if (_map.value == null) {
                _map.value = withContext(Dispatchers.Default) { buildMissionMap(intent, parsedCloud) }
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
                            // Re-aimed headings onto the map: parse the augmented KMZ
                            // (same waypoint count, new headings/pitches).
                            _map.value = withContext(Dispatchers.Default) {
                                val aug = runCatching {
                                    WpmlParser.parseKmz(ev.augmentedKmz, missionName = summary.name).intent
                                }.getOrNull()
                                buildMissionMap(
                                    intent, parsedCloud, aug, summary.flaggedIndices,
                                    facades = summary.facadeGeom, targets = summary.wpTargets,
                                )
                            }
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

            // Always allow an older scan: the strip shows its age, the pilot decides by tapping Augment.
            sess.sendAndAwaitPreview(intent, fingerprintBytes, allowStaleMesh = true)
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
            // First check shows "Checking…"; the automatic re-checks keep the
            // last result on screen (no flicker) and stop as soon as Ready.
            _banner.value = BannerState.Checking
            while (true) {
                val q = StatusSession().query()
                lastStatus = (q as? StatusSession.Result.Ok)?.status
                if (lastStatus != null) _status.value = lastStatus
                val result = bannerFor(q)
                _banner.value = result
                val wait = nextPollDelayMs(result) ?: break
                delay(wait)
                if (session != null || augmentJob?.isActive == true) break
                if (connection.value !is Connection.State.AircraftConnected) break
            }
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
