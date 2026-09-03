package com.aeroscan.rccompanion.ui

/**
 * Whether the aircraft link is being handed back and forth with another MSDK app.
 *
 * Background: only one app at a time may hold the aircraft link on the RC. DJI's
 * guidance for the RC Plus is to force-exit DJI Pilot before starting a
 * third-party MSDK app. We cannot do that for the pilot — and we do not want to,
 * because Pilot 2 is where the Fly widget lives — so instead we name the symptom
 * and give the remedy at the moment it bites, and we refuse to start a multi-
 * minute augment into a link that is about to drop.
 *
 * Measured on the RC 2026-09-03 with Pilot 2 alive in the background: drop →
 * reconnect every ~22 s. A single drop (walking out of range, a reboot) is
 * normal and must not trigger the warning, so the threshold is two drops inside
 * the window.
 */
object LinkHealth {
    /** Drops inside this window count toward "flapping". */
    const val WINDOW_MS = 90_000L

    /** Two drops in the window is a handover fight, not a one-off. */
    const val FLAP_DROPS = 2

    /** A fresh link needs to hold this long before a transfer may start. */
    const val SETTLE_MS = 4_000L

    fun dropsInWindow(drops: List<Long>, nowMs: Long): Int = drops.count { nowMs - it <= WINDOW_MS }

    fun isFlapping(drops: List<Long>, nowMs: Long): Boolean = dropsInWindow(drops, nowMs) >= FLAP_DROPS

    /** True once the link has been up long enough to trust with a long transfer. */
    fun isSettled(upSinceMs: Long?, nowMs: Long): Boolean =
        upSinceMs != null && nowMs - upSinceMs >= SETTLE_MS

    /** What to tell the pilot, or null when the link is healthy. */
    fun advice(drops: List<Long>, nowMs: Long): String? =
        if (isFlapping(drops, nowMs))
            "The aircraft link keeps switching between this app and DJI Pilot 2. " +
                "Close Pilot 2 from the recent-apps list while you upload, then reopen it to fly."
        else null
}
