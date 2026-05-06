package com.aeroscan.rccompanion.mop

object MopConstants {
    // Legacy AERS file-transfer channel — kept while the MopFileSender path
    // is still around for compat / testing. Not used by the augment flow.
    const val MOP_CHANNEL_ID: Int = 49152

    // Augment-flow channel — must match KMZRUN_CHANNEL_ID in
    // aeroscan-psdk:src/manifold3_app/kmz_runner.c.
    const val AUGMENT_CHANNEL_ID: Int = 49154

    // MSDK V5 docs recommend ≤ 1 KB per write call on the upstream channel.
    const val CHUNK_SIZE: Int = 1024

    // Legacy AERS magic for the old file-transfer flow.
    const val FRAME_MAGIC: Int = 0x41455253 // "AERS"

    // Augment-flow magics. All 16-byte headers are: 4-byte ASCII magic +
    // 4-byte LE version + 4-byte LE body_len + 4 bytes reserved (zero).
    const val MAGIC_AUGM: String = "AUGM"  // RC → Manifold: intent + cloud
    const val MAGIC_PRVW: String = "PRVW"  // Manifold → RC: KMZ + summary
    const val MAGIC_EXEC: String = "EXEC"  // RC → Manifold: pilot approval

    // Header layout version. Bump if the framing changes incompatibly.
    const val FRAME_VERSION: Int = 1

    // Reliable upstream is 24–48 Kbps; a 1 MB file is ~3 minutes worst case.
    // Hard ceiling so a stuck transfer doesn't block forever.
    const val TRANSFER_TIMEOUT_MS: Long = 10 * 60 * 1000L

    // Connect handshake timeout. Reasonable since OcuSync handshake is sub-second.
    const val CONNECT_TIMEOUT_MS: Long = 10 * 1000L

    // Augment subprocess wall-clock can hit 4 min on a Mijande-class building.
    // Receive timeout for PRVW frames must comfortably exceed that.
    const val AUGMENT_PRVW_TIMEOUT_MS: Long = 10 * 60 * 1000L
}
