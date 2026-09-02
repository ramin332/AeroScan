package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.mop.AugmentFraming
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/** The Manifold refuses a mesh older than 6 h; the pilot may override it knowingly. */
class StaleMeshTest {
    private val stale = """{"latest_flight":"flight0076","mesh_present":false,"mesh_chunks":13,"n_points":1,
        "mesh_bytes":1,"blackbox_free_gb":42.0,"env_ok":true,"env_detail":"ok",
        "mesh_exists":true,"mesh_age_s":4665600,"mesh_flight":"flight0072"}"""

    @Test
    fun stale_scan_is_recognised_and_named_with_its_age() {
        val s = AugmentFraming.parseStatJson(stale)
        assertTrue(s.meshStale)
        assertEquals("flight0072", s.meshFlight)
        val b = bannerFor(StatusSession.Result.Ok(s)) as BannerState.NoMesh
        assertTrue(b.label, b.label.contains("flight0072 is 7 w old"))
        assertTrue(b.label, b.label.contains("Use old scan"))
    }

    @Test
    fun no_scan_at_all_is_not_stale_and_says_so() {
        val s = AugmentFraming.parseStatJson("""{"mesh_present":false,"mesh_exists":false,"env_ok":true}""")
        assertTrue(!s.meshStale)
        val b = bannerFor(StatusSession.Result.Ok(s)) as BannerState.NoMesh
        assertTrue(b.label, b.label.startsWith("No scan on the drone"))
    }

    @Test
    fun age_text_scales() {
        assertEquals("20 min", ageText(1200)); assertEquals("5 h", ageText(5 * 3600L))
        assertEquals("3 d", ageText(3 * 86400L)); assertEquals("8 w", ageText(8 * 7 * 86400L)); assertEquals("?", ageText(-1))
    }
}
