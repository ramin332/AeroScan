package com.aeroscan.rccompanion.ui

/**
 * Whether the aircraft link is cycling instead of holding.
 *
 * Measured on the RC 2026-09-03. Every `Product connected` was followed within a
 * second by `initPSDKDevice error … PAYLOAD.FetchWidgetFile:-13` and `widgetSet
 * is null or data is empty!`, then a disconnect about 20 s later — over and over,
 * while the AeroScan payload app on the Manifold was NOT running. With the payload
 * app running the link held for a full three-minute watch, with DJI Pilot 2 alive
 * in the background the whole time.
 *
 * So the cause is the payload, not competition with Pilot 2. (An earlier reading
 * of the same log blamed Pilot 2 for taking the link back; the three-minute watch
 * with Pilot 2 running and the payload up disproved it.) This matches what the
 * pilot described: enable the payload in Pilot 2 and only then does the RC app
 * connect properly.
 *
 * A single drop (walking out of range, a reboot) is normal and must not trigger
 * the warning, so the threshold is two drops inside the window.
 */
object LinkHealth {
    /** Drops inside this window count toward "cycling". */
    const val WINDOW_MS = 90_000L

    /** Two drops in the window is a cycle, not a one-off. */
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
            "The aircraft link keeps dropping and coming back. Enable the AeroScan payload in " +
                "DJI Pilot 2 (camera view, payload panel) — the link cycles about every 20 s while " +
                "the payload app is not running."
        else null
}
