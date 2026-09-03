package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.mop.AugmentFraming
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PanelStatusTest {
    private val linked = Connection.State.AircraftConnected(productId = 1)
    private fun stat(json: String) = AugmentFraming.parseStatJson(json)
    private val fresh = stat("""{"latest_flight":"flight0080","mesh_present":true,"mesh_chunks":12,"n_points":9,
        "mesh_bytes":1,"blackbox_free_gb":42.0,"env_ok":true,"env_detail":"ok","mesh_exists":true,"mesh_age_s":900,"mesh_flight":"flight0080"}""")
    private val stale = stat("""{"latest_flight":"flight0081","mesh_present":false,"mesh_chunks":13,"n_points":1,
        "mesh_bytes":1,"blackbox_free_gb":42.0,"env_ok":true,"env_detail":"ok","mesh_exists":true,"mesh_age_s":4665600,"mesh_flight":"flight0072"}""")
    private val none = stat("""{"mesh_present":false,"mesh_exists":false,"env_ok":true}""")

    @Test
    fun fresh_scan_is_green_and_augment_allowed() {
        val p = panelStatusFor(linked, bannerFor(StatusSession.Result.Ok(fresh)), fresh)
        assertEquals(Tone.Good, p.aircraft.tone)
        assertEquals(Tone.Good, p.manifold.tone)
        assertEquals("Scan 15 min old", p.scan.label); assertEquals(Tone.Good, p.scan.tone)
        assertEquals("No mission on drone", p.mission.label)
        assertNull(augmentBlockReason(linked, bannerFor(StatusSession.Result.Ok(fresh)), fresh))
    }

    @Test
    fun old_scan_is_amber_but_never_blocks() {
        val b = bannerFor(StatusSession.Result.Ok(stale))
        val p = panelStatusFor(linked, b, stale)
        assertEquals(Tone.Warn, p.scan.tone)
        assertTrue(p.scan.label, p.scan.label.contains("7 w old"))
        assertTrue(p.scan.label, p.scan.label.contains("flight0072"))
        assertNull(augmentBlockReason(linked, b, stale))
    }

    @Test
    fun no_scan_blocks_with_a_one_liner() {
        val b = bannerFor(StatusSession.Result.Ok(none))
        assertEquals(Tone.Bad, panelStatusFor(linked, b, none).scan.tone)
        assertEquals("No scan on the drone. Fly a Smart3D scan first.", augmentBlockReason(linked, b, none))
    }

    @Test
    fun unreachable_manifold_and_missing_aircraft_block() {
        val un = bannerFor(StatusSession.Result.Unreachable("x"))
        assertEquals(Tone.Bad, panelStatusFor(linked, un, null).manifold.tone)
        assertTrue(augmentBlockReason(linked, un, null)!!.contains("Pilot 2"))
        assertTrue(augmentBlockReason(Connection.State.AircraftDisconnected, un, null)!!.contains("aircraft"))
    }

    @Test
    fun mission_chip_reads_the_recorded_state() {
        val s = stat("""{"mesh_present":true,"mesh_exists":true,"env_ok":true,"mission_id":"m1","mission_state":"interrupted",
            "mission_last_index":217,"mission_total":398,"mission_resume_from":217}""")
        val p = panelStatusFor(linked, bannerFor(StatusSession.Result.Ok(s)), s)
        assertEquals("Interrupted at 217/398", p.mission.label); assertEquals(Tone.Warn, p.mission.tone)
    }
}
