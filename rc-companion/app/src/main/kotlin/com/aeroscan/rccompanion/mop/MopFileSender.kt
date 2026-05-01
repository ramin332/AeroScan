package com.aeroscan.rccompanion.mop

import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.mop.PipelineDeviceType
import dji.sdk.keyvalue.value.mop.TransmissionControlType
import dji.v5.manager.mop.DataResult
import dji.v5.manager.mop.Pipeline
import dji.v5.manager.mop.PipelineManager
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.InputStream

/**
 * MSDK V5 MOP file sender. API surface verified against
 * `Mobile-SDK-Android-V5/SampleCode-V5/.../models/MopVM.kt` and
 * `.../data/MOPCmdHelper.java` on `dev-sdk-main`.
 *
 * Lifecycle:
 *   1. PipelineManager.connectPipeline(index, id, deviceType, transmissionType)
 *      → returns null on success, error otherwise
 *   2. PipelineManager.pipelines[id] → the connected Pipeline
 *   3. pipeline.writeData(bytes) → DataResult{length, error}, both synchronous
 *   4. PipelineManager.disconnectPipeline(...) when done
 *
 * Wire format is in [Framing] — header (magic + version + nameLen + size + name)
 * → payload bytes → 16-byte MD5. Both sides must agree on this layout.
 */
class MopFileSender(
    private val componentIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val channelId: Int = MopConstants.MOP_CHANNEL_ID,
    private val deviceType: PipelineDeviceType = PipelineDeviceType.PAYLOAD,
    private val mode: TransmissionControlType = TransmissionControlType.STABLE,
) {

    sealed class Result {
        data object Ok : Result()
        data class Failed(val cause: Throwable) : Result()
        data object Cancelled : Result()
    }

    fun interface ProgressListener {
        fun onProgress(bytesSent: Long, totalBytes: Long)
    }

    suspend fun send(
        filename: String,
        fileSize: Long,
        input: InputStream,
        progress: ProgressListener,
    ): Result = withContext(Dispatchers.IO) {
        val mgr = PipelineManager.getInstance()
        val connectErr = mgr.connectPipeline(componentIndex, channelId, deviceType, mode)
        if (connectErr != null) {
            return@withContext Result.Failed(IllegalStateException("MOP connect failed: $connectErr"))
        }
        val pipeline = mgr.pipelines[channelId]
            ?: return@withContext Result.Failed(IllegalStateException("Pipeline missing from manager map after connect"))

        try {
            writeAll(pipeline, Framing.buildHeader(filename, fileSize))
            val digest = Framing.newDigest()
            val buf = ByteArray(MopConstants.CHUNK_SIZE)
            var sent = 0L
            while (sent < fileSize) {
                val want = minOf(buf.size.toLong(), fileSize - sent).toInt()
                val read = input.read(buf, 0, want)
                if (read <= 0) error("input stream ended early at $sent / $fileSize")
                digest.update(buf, 0, read)
                val slice = if (read == buf.size) buf else buf.copyOfRange(0, read)
                writeAll(pipeline, slice)
                sent += read
                progress.onProgress(sent, fileSize)
            }
            writeAll(pipeline, Framing.finishDigest(digest))
            Result.Ok
        } catch (c: CancellationException) {
            Result.Cancelled
        } catch (t: Throwable) {
            Log.e(TAG, "send failed", t)
            Result.Failed(t)
        } finally {
            runCatching { mgr.disconnectPipeline(componentIndex, channelId, deviceType, mode) }
        }
    }

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
            if (written <= 0) error("MOP writeData returned non-positive length: $written")
            sent += written
        }
    }

    companion object {
        private const val TAG = "MopFileSender"
    }
}
