package com.aeroscan.rccompanion.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * The readiness banner keeps re-checking on its own while the Manifold app is
 * not up yet. Field observation (2026-07-10, 2026-09-02): after the pilot
 * switches apps in DJI Pilot 2 the AeroScan app takes 10–30 s to bind its MOP
 * channel; a single check at aircraft-connect time lands in that window and
 * the banner stayed "Unreachable" until someone tapped Retry.
 */
class ReadinessPollTest {

    @Test
    fun keeps_polling_while_the_app_is_not_reachable_or_not_ready() {
        assertEquals(READINESS_POLL_MS, nextPollDelayMs(BannerState.Unreachable("x")))
        assertEquals(READINESS_POLL_MS, nextPollDelayMs(BannerState.NoMesh("x")))
        assertEquals(READINESS_POLL_MS, nextPollDelayMs(BannerState.EnvError("x")))
    }

    @Test
    fun stops_polling_once_ready_and_never_polls_from_transient_states() {
        assertNull(nextPollDelayMs(BannerState.Ready("ok")))
        assertNull(nextPollDelayMs(BannerState.Checking))
        assertNull(nextPollDelayMs(BannerState.Idle))
    }

    @Test
    fun unreachable_banner_tells_the_pilot_what_to_do_in_pilot_2() {
        val b = bannerFor(com.aeroscan.rccompanion.mop.StatusSession.Result.Unreachable("connect failed"))
        val label = (b as BannerState.Unreachable).label
        assert(label.contains("Pilot 2")) { label }
        assert(label.contains("re-check", ignoreCase = true)) { label }
    }
}
