package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.mop.AugmentFraming
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The Manifold's STAT now carries the mission lifecycle (kmzrun_progress,
 * 2026-09-02). The banner must show it in one line, and an interrupted
 * mission must be recognisable so the pilot is asked before a new augment
 * replaces it.
 */
class MissionStatusTest {

    private val interrupted = """{"app_version":"0.5","flight_id":"the_latest_flight","latest_flight":"flight0072",
        "mesh_present":true,"mesh_chunks":13,"n_points":360000,"mesh_bytes":1,"blackbox_free_gb":42.0,
        "env_ok":true,"env_detail":"ok","mission_id":"20260710T103507Z_331","mission_state":"interrupted",
        "mission_last_index":217,"mission_total":398,"mission_resume_from":216}"""

    @Test
    fun parses_mission_fields_and_tolerates_their_absence() {
        val s = AugmentFraming.parseStatJson(interrupted)
        assertEquals("20260710T103507Z_331", s.missionId)
        assertEquals("interrupted", s.missionState)
        assertEquals(217, s.missionLastIndex)
        assertEquals(398, s.missionTotal)
        assertEquals(216, s.missionResumeFrom)
        assertTrue(s.interrupted)
        val old = AugmentFraming.parseStatJson("""{"latest_flight":"flight0019"}""")
        assertEquals("", old.missionState)
        assertEquals(-1, old.missionResumeFrom)
        assertTrue(!old.interrupted)
    }

    @Test
    fun banner_shows_the_interrupted_mission_on_a_ready_manifold() {
        val b = bannerFor(StatusSession.Result.Ok(AugmentFraming.parseStatJson(interrupted)))
        assertTrue(b is BannerState.Ready)
        val label = (b as BannerState.Ready).label
        assertTrue(label, label.contains("interrupted at WP 217/398"))
        assertTrue(label, label.contains("Continue"))
    }

    @Test
    fun completed_and_flying_have_their_own_lines_and_none_is_silent() {
        val done = AugmentFraming.parseStatJson(interrupted.replace("\"interrupted\"", "\"completed\"").replace("217", "398"))
        assertTrue(missionSuffix(done).contains("completed 398/398"))
        val flying = AugmentFraming.parseStatJson(interrupted.replace("\"interrupted\"", "\"flying\""))
        assertTrue(missionSuffix(flying).contains("flying at WP 217/398"))
        assertEquals("", missionSuffix(AugmentFraming.parseStatJson("""{"latest_flight":"x"}""")))
    }
}
