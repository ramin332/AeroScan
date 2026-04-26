package com.aeroscan.rccompanion.mop

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.security.MessageDigest

/**
 * Pure-Kotlin framing for the AeroScan KMZ-over-MOP file transfer.
 *
 * Wire layout (little-endian throughout):
 *
 *   header (16 + N bytes):
 *     u32  magic         = 0x41455253 ("AERS")
 *     u32  version       = 1
 *     u32  filenameLen   = N (bytes, UTF-8)
 *     u64  fileSize      = total payload size
 *     u8[N] filename     = UTF-8, no NUL terminator
 *
 *   payload (fileSize bytes): raw KMZ bytes, written in CHUNK_SIZE slabs
 *
 *   trailer (16 bytes): MD5 digest of the payload
 *
 * The PSDK side reads header → reads exactly fileSize bytes → reads 16 MD5
 * bytes → compares against its own running digest. Bytes-on-the-wire
 * compatibility with `samples/sample_c/module_sample/mop_channel/test_mop_channel.c`
 * is the source of truth — adjust this layout if the PSDK side diverges.
 */
object Framing {

    const val MD5_LEN: Int = 16
    const val HEADER_FIXED_LEN: Int = 4 + 4 + 4 + 8  // magic + version + nameLen + size

    fun buildHeader(filename: String, fileSize: Long): ByteArray {
        require(fileSize >= 0) { "fileSize must be non-negative, got $fileSize" }
        val nameBytes = filename.toByteArray(Charsets.UTF_8)
        require(nameBytes.size <= MAX_FILENAME_BYTES) {
            "filename too long: ${nameBytes.size} bytes > $MAX_FILENAME_BYTES"
        }
        val buf = ByteBuffer.allocate(HEADER_FIXED_LEN + nameBytes.size).order(ByteOrder.LITTLE_ENDIAN)
        buf.putInt(MopConstants.FRAME_MAGIC)
        buf.putInt(MopConstants.FRAME_VERSION)
        buf.putInt(nameBytes.size)
        buf.putLong(fileSize)
        buf.put(nameBytes)
        return buf.array()
    }

    fun newDigest(): MessageDigest = MessageDigest.getInstance("MD5")

    fun finishDigest(digest: MessageDigest): ByteArray {
        val out = digest.digest()
        check(out.size == MD5_LEN) { "MD5 digest length unexpected: ${out.size}" }
        return out
    }

    private const val MAX_FILENAME_BYTES: Int = 255
}
