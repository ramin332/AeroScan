package com.aeroscan.rccompanion.mop

object MopConstants {
    // Coordinate this with the Manifold-side PSDK app. DJI's reserved sample
    // default for arbitrary streams is 49152; the file service sample uses 49153.
    // Both repos must agree before any device test.
    const val MOP_CHANNEL_ID: Int = 49152

    // MSDK V5 docs recommend ≤ 1 KB per write call on the upstream channel.
    const val CHUNK_SIZE: Int = 1024

    // Magic prefix tagging the file-transfer header. Any value works as long as
    // the PSDK side matches it byte-for-byte.
    const val FRAME_MAGIC: Int = 0x41455253 // "AERS"

    // Header layout version. Bump if the framing changes incompatibly.
    const val FRAME_VERSION: Int = 1

    // Reliable upstream is 24–48 Kbps; a 1 MB file is ~3 minutes worst case.
    // Hard ceiling so a stuck transfer doesn't block forever.
    const val TRANSFER_TIMEOUT_MS: Long = 10 * 60 * 1000L

    // Connect handshake timeout. Reasonable since OcuSync handshake is sub-second.
    const val CONNECT_TIMEOUT_MS: Long = 10 * 1000L
}
