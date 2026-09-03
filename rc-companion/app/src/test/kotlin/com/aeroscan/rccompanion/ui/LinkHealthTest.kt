package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.mop.AugmentFraming
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * 2026-09-03: with Pilot 2 alive in the background the RC's logcat showed
 * CoreExistReceiver "setNeedTryConnect false" then a disconnect every ~22 s.
 * Only one MSDK app may hold the aircraft link at a time.
 */
class LinkHealthTest {
    private val t = 1_000_000L
    private val linked = Connection.State.AircraftConnected(productId = 1)
    private val ready = AugmentFraming.parseStatJson(
        """{"latest_flight":"f","mesh_present":true,"mesh_exists":true,"mesh_age_s":600,"env_ok":true,"blackbox_free_gb":42.0}""",
    )
    private val readyBanner = bannerFor(StatusSession.Result.Ok(ready))

    @Test
    fun one_drop_is_not_flapping_two_in_the_window_is() {
        assertTrue(!LinkHealth.isFlapping(listOf(t - 1_000), t))
        assertTrue(LinkHealth.isFlapping(listOf(t - 30_000, t - 1_000), t))
    }

    @Test
    fun old_drops_age_out_of_the_window() {
        val old = listOf(t - LinkHealth.WINDOW_MS - 1, t - LinkHealth.WINDOW_MS - 2)
        assertEquals(0, LinkHealth.dropsInWindow(old, t))
        assertNull(LinkHealth.advice(old, t))
    }

    @Test
    fun advice_names_pilot_2_and_the_remedy() {
        val a = LinkHealth.advice(listOf(t - 20_000, t - 2_000), t)!!
        assertTrue(a, a.contains("Pilot 2"))
        assertTrue(a, a.contains("Close"))
    }

    @Test
    fun a_flapping_link_shows_amber_and_blocks_the_augment() {
        val drops = listOf(t - 40_000, t - 5_000)
        val chip = panelStatusFor(linked, readyBanner, ready, drops, t).aircraft
        assertEquals(Tone.Warn, chip.tone)
        assertEquals("Link shared with Pilot 2", chip.label)
        val why = augmentBlockReason(linked, readyBanner, ready, drops, upSinceMs = t - 60_000, nowMs = t)
        assertTrue(why!!, why.contains("Pilot 2"))
    }

    @Test
    fun a_freshly_restored_link_waits_to_settle_then_allows_the_augment() {
        val justUp = augmentBlockReason(linked, readyBanner, ready, emptyList(), upSinceMs = t - 500, nowMs = t)
        assertTrue(justUp!!, justUp.contains("just came up"))
        assertNull(augmentBlockReason(linked, readyBanner, ready, emptyList(), upSinceMs = t - LinkHealth.SETTLE_MS - 1, nowMs = t))
    }

    @Test
    fun connection_records_drops_and_up_time_only_on_real_transitions() {
        Connection.publish(Connection.State.AircraftDisconnected, t)
        Connection.publish(Connection.State.AircraftConnected(1), t + 1_000)
        val before = Connection.drops.value.size
        Connection.publish(Connection.State.AircraftConnected(1), t + 2_000) // duplicate
        assertEquals(before, Connection.drops.value.size)
        assertEquals(t + 1_000, Connection.upSince.value)
        Connection.publish(Connection.State.AircraftDisconnected, t + 3_000)
        assertEquals(before + 1, Connection.drops.value.size)
        assertNull(Connection.upSince.value)
    }
}
