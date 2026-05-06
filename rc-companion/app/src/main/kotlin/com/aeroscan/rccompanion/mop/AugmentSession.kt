package com.aeroscan.rccompanion.mop

import android.util.Log
import com.aeroscan.rccompanion.wpml.ImportedKmz
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.mop.PipelineDeviceType
import dji.sdk.keyvalue.value.mop.TransmissionControlType
import dji.v5.manager.mop.DataResult
import dji.v5.manager.mop.Pipeline
import dji.v5.manager.mop.PipelineManager
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.withContext

/**
 * Owns one RC↔Manifold MOP session for the augment cycle:
 *
 *   1. open MOP pipeline on [MopConstants.AUGMENT_CHANNEL_ID]
 *   2. send AUGM frame (mission-intent JSON + 1 m cloud fingerprint)
 *   3. wait for PRVW frame (augmented KMZ + summary JSON, ~4 min Manifold-side)
 *   4. caller decides APPROVE / REJECT
 *      - APPROVE → send EXEC frame; Manifold uploads to aircraft
 *      - REJECT  → close pipeline, drop the session
 *   5. close pipeline
 *
 * Wire format is encoded by [AugmentFraming] — bytes-for-bytes compatible with
 * `aeroscan-psdk:src/manifold3_app/kmz_runner.c`.
 *
 * Threading: every entry point runs on Dispatchers.IO. The MSDK Pipeline
 * methods are synchronous (writeData/readData both block); the coroutine is
 * the cancellation surface.
 *
 * Progress + state are surfaced via [events] (SharedFlow). The UI (HomeViewModel
 * + PreviewScreen) collects this to drive the state machine.
 */
class AugmentSession(
    private val componentIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val channelId: Int = MopConstants.AUGMENT_CHANNEL_ID,
    private val deviceType: PipelineDeviceType = PipelineDeviceType.PAYLOAD,
    private val mode: TransmissionControlType = TransmissionControlType.STABLE,
) {

    sealed class Event {
        data object Connecting : Event()
        data class Sending(val bytesSent: Long, val totalBytes: Long) : Event()
        data object SendComplete : Event()
        data object WaitingForPreview : Event()
        data class PreviewReceived(val summaryJson: ByteArray, val augmentedKmz: ByteArray) : Event()
        data object ExecuteSent : Event()
        data class Failed(val cause: Throwable) : Event()
        data object Cancelled : Event()
        data object Closed : Event()
    }

    private val _events = MutableSharedFlow<Event>(replay = 0, extraBufferCapacity = 16)
    val events: SharedFlow<Event> = _events.asSharedFlow()

    /** Currently-held pipeline. Lives for the duration of the session, closed
     *  when [closeAndRelease] is called or the coroutine is cancelled. */
    private var pipeline: Pipeline? = null

    /**
     * Send the AUGM frame (intent + cloud fingerprint), then block-read until
     * a PRVW frame arrives. Returns the parsed preview body. Caller must then
     * call either [approve] (sends EXEC) or [closeAndRelease] (drops session).
     *
     * On any error or cancellation the pipeline is closed automatically.
     */
    suspend fun sendAndAwaitPreview(
        intent: ImportedKmz,
        cloudFingerprintPly: ByteArray,
    ): Event = withContext(Dispatchers.IO) {
        try {
            _events.emit(Event.Connecting)
            val mgr = PipelineManager.getInstance()
            val connectErr = mgr.connectPipeline(componentIndex, channelId, deviceType, mode)
            if (connectErr != null) {
                val ev = Event.Failed(IllegalStateException("MOP connect failed: $connectErr"))
                _events.emit(ev)
                return@withContext ev
            }
            val pl = mgr.pipelines[channelId]
                ?: run {
                    val ev = Event.Failed(IllegalStateException("Pipeline missing from manager map"))
                    _events.emit(ev); return@withContext ev
                }
            pipeline = pl

            // ---- AUGM uplink -----------------------------------------------------
            val intentJson = intent.toJsonString().toByteArray(Charsets.UTF_8)
            val body = AugmentFraming.buildAugmBody(intentJson, cloudFingerprintPly)
            val frame = AugmentFraming.frame(MopConstants.MAGIC_AUGM, body)
            writeAll(pl, frame) { sent -> _events.tryEmit(Event.Sending(sent, frame.size.toLong())) }
            _events.emit(Event.SendComplete)

            // ---- PRVW downlink ---------------------------------------------------
            _events.emit(Event.WaitingForPreview)
            val hdr = readExactly(pl, AugmentFraming.HEADER_LEN)
            val parsed = AugmentFraming.parseHeader(hdr)
            require(parsed.magic == MopConstants.MAGIC_PRVW) {
                "expected PRVW frame, got '${parsed.magic}'"
            }
            require(parsed.version == MopConstants.FRAME_VERSION) {
                "PRVW version mismatch: got ${parsed.version}, expected ${MopConstants.FRAME_VERSION}"
            }
            require(parsed.bodyLen in 1..AugmentFraming.MAX_BODY_LEN) {
                "PRVW bodyLen out of range: ${parsed.bodyLen}"
            }
            val previewBody = readExactly(pl, parsed.bodyLen)
            val preview = AugmentFraming.parsePreviewBody(previewBody)

            val ev = Event.PreviewReceived(preview.summaryJson, preview.augmentedKmz)
            _events.emit(ev)
            ev
        } catch (c: CancellationException) {
            closePipelineQuietly()
            _events.emit(Event.Cancelled)
            Event.Cancelled
        } catch (t: Throwable) {
            Log.e(TAG, "sendAndAwaitPreview failed", t)
            closePipelineQuietly()
            val ev = Event.Failed(t)
            _events.emit(ev)
            ev
        }
    }

    /**
     * Pilot tapped APPROVE — ship the EXEC frame so kmz_runner uploads the
     * augmented KMZ to the aircraft. Closes the session afterwards (the
     * pipeline's job is done; tap-to-fly happens on Pilot 2's widget, not over
     * this MOP channel).
     */
    suspend fun approve(missionId: String = ""): Event = withContext(Dispatchers.IO) {
        val pl = pipeline ?: run {
            val ev = Event.Failed(IllegalStateException("approve called without an open pipeline"))
            _events.emit(ev); return@withContext ev
        }
        try {
            val body = AugmentFraming.buildExecBody(missionId)
            val frame = AugmentFraming.frame(MopConstants.MAGIC_EXEC, body)
            writeAll(pl, frame) { /* small frame, no progress */ }
            _events.emit(Event.ExecuteSent)
            closePipelineQuietly()
            _events.emit(Event.Closed)
            Event.ExecuteSent
        } catch (c: CancellationException) {
            closePipelineQuietly()
            _events.emit(Event.Cancelled)
            Event.Cancelled
        } catch (t: Throwable) {
            Log.e(TAG, "approve failed", t)
            closePipelineQuietly()
            val ev = Event.Failed(t)
            _events.emit(ev)
            ev
        }
    }

    /** Pilot tapped REJECT — drop the session. */
    suspend fun closeAndRelease() = withContext(Dispatchers.IO) {
        closePipelineQuietly()
        _events.emit(Event.Closed)
    }

    // ---------------------------------------------------------------------
    // MSDK pipeline I/O helpers
    // ---------------------------------------------------------------------

    private fun writeAll(pipeline: Pipeline, data: ByteArray, onProgress: (Long) -> Unit) {
        var sent = 0
        while (sent < data.size) {
            val chunkSize = minOf(MopConstants.CHUNK_SIZE, data.size - sent)
            val slice = if (sent == 0 && chunkSize == data.size) data
                else data.copyOfRange(sent, sent + chunkSize)
            val result: DataResult = pipeline.writeData(slice)
            val written = result.length
            val err = result.error
            if (err != null) error("MOP writeData error: $err")
            if (written <= 0) error("MOP writeData returned non-positive: $written")
            sent += written
            onProgress(sent.toLong())
        }
    }

    /**
     * Block-read exactly [want] bytes off the pipeline. The MSDK
     * `Pipeline.readData(buf)` reads up to buf.size and returns DataResult
     * with the actual length; loop until we have the full payload. Manifold's
     * augment can take ~4 min; the MSDK should not time out the call (long
     * idle reads are normal for our workload), but if it does we surface the
     * error and let the caller decide.
     */
    private fun readExactly(pipeline: Pipeline, want: Int): ByteArray {
        val out = ByteArray(want)
        var got = 0
        val tmp = ByteArray(MopConstants.CHUNK_SIZE)
        while (got < want) {
            val capacity = minOf(tmp.size, want - got)
            val readBuf = if (capacity == tmp.size) tmp else ByteArray(capacity)
            val result: DataResult = pipeline.readData(readBuf)
            val read = result.length
            val err = result.error
            if (err != null) error("MOP readData error: $err")
            if (read <= 0) {
                // Some MSDK builds return 0 when the link is briefly idle;
                // the caller's coroutine cancellation is the actual loop-out.
                continue
            }
            System.arraycopy(readBuf, 0, out, got, read)
            got += read
        }
        return out
    }

    private fun closePipelineQuietly() {
        runCatching {
            PipelineManager.getInstance()
                .disconnectPipeline(componentIndex, channelId, deviceType, mode)
        }
        pipeline = null
    }

    companion object {
        private const val TAG = "AugmentSession"
    }
}
