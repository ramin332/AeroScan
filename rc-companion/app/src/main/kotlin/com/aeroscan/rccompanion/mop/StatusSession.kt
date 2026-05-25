package com.aeroscan.rccompanion.mop

import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.mop.PipelineDeviceType
import dji.sdk.keyvalue.value.mop.TransmissionControlType
import dji.v5.manager.mop.DataResult
import dji.v5.manager.mop.Pipeline
import dji.v5.manager.mop.PipelineManager
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * One-shot Manifold readiness query: connect → send PING → read STAT → close.
 *
 * Mirrors [AugmentSession]'s MOP pipeline I/O (same channel, same framing via
 * [AugmentFraming]) but is **bounded** — there is no multi-minute retry loop.
 * The whole round trip is capped at [MopConstants.STATUS_TIMEOUT_MS]; any
 * failure (connect refused, wrong magic, timeout, old Manifold that drops the
 * connection) resolves to [Result.Unreachable] rather than throwing, so the
 * banner can always render a definite state.
 *
 * Wire format is byte-for-byte compatible with
 * `aeroscan-psdk:src/manifold3_app/kmz_runner.c` (PING request) +
 * `kmzrun_status.c` (STAT JSON body).
 *
 * Concurrency: this opens its own pipeline on [MopConstants.AUGMENT_CHANNEL_ID]
 * and must NOT run while an [AugmentSession] holds the same channel. The
 * ViewModel serializes the two (status check only runs when no augment is in
 * flight).
 */
class StatusSession(
    private val componentIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val channelId: Int = MopConstants.AUGMENT_CHANNEL_ID,
    private val deviceType: PipelineDeviceType = PipelineDeviceType.PAYLOAD,
    private val mode: TransmissionControlType = TransmissionControlType.STABLE,
) {

    sealed class Result {
        data class Ok(val status: AugmentFraming.ManifoldStatus) : Result()
        data class Unreachable(val reason: String) : Result()
    }

    /**
     * Connect, send a PING, read the STAT reply, parse it, and close. Never
     * throws — all failure modes collapse to [Result.Unreachable].
     */
    suspend fun query(deadlineMs: Long = MopConstants.STATUS_TIMEOUT_MS): Result =
        withContext(Dispatchers.IO) {
            val mgr = PipelineManager.getInstance()
            val connectErr = mgr.connectPipeline(componentIndex, channelId, deviceType, mode)
            if (connectErr != null) {
                return@withContext Result.Unreachable("connect failed: $connectErr")
            }
            val pl = mgr.pipelines[channelId]
                ?: run {
                    closeQuietly()
                    return@withContext Result.Unreachable("pipeline missing from manager map")
                }
            try {
                writeAll(pl, AugmentFraming.buildPingFrame())

                val deadline = System.currentTimeMillis() + deadlineMs
                val hdrBytes = readExactly(pl, AugmentFraming.HEADER_LEN, deadline)
                    ?: return@withContext Result.Unreachable("no STAT header (timeout)")
                val hdr = AugmentFraming.parseHeader(hdrBytes)
                if (hdr.magic != MopConstants.MAGIC_STAT) {
                    return@withContext Result.Unreachable("expected STAT, got '${hdr.magic}'")
                }
                if (hdr.bodyLen !in 1..AugmentFraming.MAX_BODY_LEN) {
                    return@withContext Result.Unreachable("bad STAT body length ${hdr.bodyLen}")
                }
                val body = readExactly(pl, hdr.bodyLen, deadline)
                    ?: return@withContext Result.Unreachable("no STAT body (timeout)")
                Result.Ok(AugmentFraming.parseStatJson(body))
            } catch (t: Throwable) {
                Log.e(TAG, "status query failed", t)
                Result.Unreachable(t.message ?: t.javaClass.simpleName)
            } finally {
                closeQuietly()
            }
        }

    // ---------------------------------------------------------------------
    // MSDK pipeline I/O helpers (bounded variants of AugmentSession's)
    // ---------------------------------------------------------------------

    private fun writeAll(pipeline: Pipeline, data: ByteArray) {
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
        }
    }

    /**
     * Bounded read of exactly [want] bytes; returns null once [deadline]
     * (epoch ms) passes. Soft errors (per-call receive timeout, channel idle)
     * are retried — the same soft/hard-error classification as
     * [AugmentSession.readExactly], just with a wall-clock ceiling so the
     * banner can't hang.
     */
    private fun readExactly(pipeline: Pipeline, want: Int, deadline: Long): ByteArray? {
        val out = ByteArray(want)
        var got = 0
        val tmp = ByteArray(MopConstants.CHUNK_SIZE)
        while (got < want) {
            if (System.currentTimeMillis() > deadline) return null
            val capacity = minOf(tmp.size, want - got)
            val readBuf = if (capacity == tmp.size) tmp else ByteArray(capacity)
            val result: DataResult = pipeline.readData(readBuf)
            val read = result.length
            val err = result.error
            if (err != null) {
                val errStr = err.toString()
                val isHardClose = errStr.contains("CLOSE", ignoreCase = true) ||
                    errStr.contains("DISCONNECT", ignoreCase = true) ||
                    errStr.contains("RESET", ignoreCase = true)
                if (isHardClose) error("MOP readData hard error: $err")
                // Soft error (per-call timeout, idle) — keep waiting until our
                // own wall-clock deadline.
                continue
            }
            if (read <= 0) continue
            System.arraycopy(readBuf, 0, out, got, read)
            got += read
        }
        return out
    }

    private fun closeQuietly() {
        runCatching {
            PipelineManager.getInstance()
                .disconnectPipeline(componentIndex, channelId, deviceType, mode)
        }
    }

    companion object {
        private const val TAG = "StatusSession"
    }
}
