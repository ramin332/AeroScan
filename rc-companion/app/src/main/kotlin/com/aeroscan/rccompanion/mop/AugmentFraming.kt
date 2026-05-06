package com.aeroscan.rccompanion.mop

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Wire-format codec for the augment-flow MOP frames. Pure encoding/decoding;
 * no MSDK or coroutine plumbing — that lives in [AugmentSession].
 *
 * All frames share a 16-byte header followed by a magic-specific body:
 *
 * ```
 * Header (16 bytes):
 *   0..3   magic ASCII (e.g. "AUGM"|"PRVW"|"EXEC")
 *   4..7   version uint32 LE  (== [MopConstants.FRAME_VERSION])
 *   8..11  body_len uint32 LE
 *   12..15 reserved (zero)
 *
 * AUGM body (RC → Manifold):
 *   intent_len(4) + intent_json + cloud_len(4) + cloud_ply
 *
 * PRVW body (Manifold → RC):
 *   summary_len(4) + summary_json + kmz_len(4) + augmented_kmz
 *
 * EXEC body (RC → Manifold):
 *   mission_id_len(4) + mission_id_utf8   (mission_id may be empty)
 * ```
 *
 * Bytes-on-the-wire compatibility with `aeroscan-psdk:kmz_runner.c` is the
 * source of truth — the C decoder must accept what we emit and our Kotlin
 * decoder must accept what the C encoder emits. Round-trip is exercised in
 * [AugmentFramingTest].
 */
object AugmentFraming {

    const val HEADER_LEN: Int = 16

    /** Cap matches the C-side KMZRUN_MAX_BODY_LEN (16 MB). */
    const val MAX_BODY_LEN: Int = 16 * 1024 * 1024

    fun buildAugmHeader(bodyLen: Int): ByteArray =
        encodeHeader(MopConstants.MAGIC_AUGM, bodyLen)

    fun buildExecHeader(bodyLen: Int): ByteArray =
        encodeHeader(MopConstants.MAGIC_EXEC, bodyLen)

    /** Length-prefixed concat of two byte slabs (intent+cloud or summary+kmz). */
    fun buildAugmBody(intentJson: ByteArray, cloudPly: ByteArray): ByteArray =
        buildLengthPrefixedPair(intentJson, cloudPly)

    /** Pack a small mission id as the EXEC body (may be empty). */
    fun buildExecBody(missionId: String): ByteArray {
        val idBytes = missionId.toByteArray(Charsets.UTF_8)
        val buf = ByteBuffer.allocate(4 + idBytes.size).order(ByteOrder.LITTLE_ENDIAN)
        buf.putInt(idBytes.size)
        buf.put(idBytes)
        return buf.array()
    }

    /** Frame = header || body. Caller streams this verbatim onto the MOP pipeline. */
    fun frame(magic: String, body: ByteArray): ByteArray {
        require(magic.length == 4) { "magic must be 4 ASCII bytes" }
        val out = ByteArray(HEADER_LEN + body.size)
        encodeHeader(magic, body.size).copyInto(out, 0)
        body.copyInto(out, HEADER_LEN)
        return out
    }

    /** Helpers for [AugmentSession] reading PRVW from the MOP downlink. */

    data class ParsedHeader(val magic: String, val version: Int, val bodyLen: Int)

    fun parseHeader(hdr: ByteArray): ParsedHeader {
        require(hdr.size == HEADER_LEN) { "header must be $HEADER_LEN bytes, got ${hdr.size}" }
        val magic = String(hdr, 0, 4, Charsets.US_ASCII)
        val bb = ByteBuffer.wrap(hdr, 4, 12).order(ByteOrder.LITTLE_ENDIAN)
        val version = bb.int
        val bodyLen = bb.int
        // 4 reserved bytes follow but we don't surface them.
        return ParsedHeader(magic, version, bodyLen)
    }

    /** PRVW body parsed into its two constituent slabs. */
    data class PreviewBody(val summaryJson: ByteArray, val augmentedKmz: ByteArray)

    fun parsePreviewBody(body: ByteArray): PreviewBody {
        val (a, b) = splitLengthPrefixedPair(body)
        return PreviewBody(summaryJson = a, augmentedKmz = b)
    }

    // ---------------------------------------------------------------------
    // internals
    // ---------------------------------------------------------------------

    private fun encodeHeader(magic: String, bodyLen: Int): ByteArray {
        require(bodyLen >= 0) { "bodyLen must be non-negative, got $bodyLen" }
        require(bodyLen <= MAX_BODY_LEN) { "bodyLen $bodyLen exceeds MAX_BODY_LEN $MAX_BODY_LEN" }
        val magicBytes = magic.toByteArray(Charsets.US_ASCII)
        require(magicBytes.size == 4) { "magic must be 4 ASCII bytes, got '$magic'" }
        val out = ByteBuffer.allocate(HEADER_LEN).order(ByteOrder.LITTLE_ENDIAN)
        out.put(magicBytes)
        out.putInt(MopConstants.FRAME_VERSION)
        out.putInt(bodyLen)
        out.putInt(0) // reserved
        return out.array()
    }

    private fun buildLengthPrefixedPair(a: ByteArray, b: ByteArray): ByteArray {
        val out = ByteBuffer.allocate(4 + a.size + 4 + b.size).order(ByteOrder.LITTLE_ENDIAN)
        out.putInt(a.size); out.put(a)
        out.putInt(b.size); out.put(b)
        return out.array()
    }

    private fun splitLengthPrefixedPair(body: ByteArray): Pair<ByteArray, ByteArray> {
        require(body.size >= 8) { "body too short for two length-prefixed slabs: ${body.size}" }
        val bb = ByteBuffer.wrap(body).order(ByteOrder.LITTLE_ENDIAN)
        val aLen = bb.int
        require(aLen in 0..(body.size - 8)) { "first slab len out of range: $aLen" }
        val a = ByteArray(aLen).also { bb.get(it) }
        val bLen = bb.int
        require(bLen in 0..(body.size - 8 - aLen)) { "second slab len out of range: $bLen" }
        val bVal = ByteArray(bLen).also { bb.get(it) }
        return a to bVal
    }
}
