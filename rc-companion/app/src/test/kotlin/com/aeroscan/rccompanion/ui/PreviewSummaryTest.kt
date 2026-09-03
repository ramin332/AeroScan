package com.aeroscan.rccompanion.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PreviewSummaryTest {
    private val base = """"waypoints_total":398,"waypoints_aimed":390,"facades":181,
        "gimbal_stats":{"pitch_deg":{"min":-60,"max":20,"median":-8.5},"anomaly_counts":{"pitch_up":1,"pitch_down":2},
        "anomaly_indices":{"pitch_up":[3],"pitch_down":[10,11]}},"icp":{"icp_rmse_m":0.47},"elapsed_s":120.0"""

    @Test
    fun new_metrics_parse_and_flag_indices_union() {
        val json = """{"name":"m",$base,"stop_at_waypoint":true,
            "aim":{"reversals_gt90":14,"far_picks":0,"unaimed":8},
            "gsd":{"median_mm_per_px":1.97,"target_mm_per_px":2.0,"camera":"WIDE"},
            "validation":[{"severity":"warning","code":"gsd_out_of_spec","message":"GSD high","waypoint_indices":[20,21]},
                          {"severity":"info","code":"x","message":"ignored","waypoint_indices":[99]}]}"""
        val s = HomeViewModel.PreviewSummary.fromJson(json.toByteArray())
        assertEquals(1.97, s.gsdMedianMmPx!!, 1e-9); assertEquals(2.0, s.gsdTargetMmPx!!, 1e-9)
        assertEquals(14, s.flips); assertEquals(8, s.unaimed); assertEquals(1, s.warnings); assertEquals(0, s.errors)
        assertEquals(listOf("GSD high"), s.validationMessages)
        assertEquals(setOf(3, 10, 11, 20, 21), s.flaggedIndices)
    }

    @Test
    fun facade_geometry_and_targets_parse_for_the_mission_view() {
        val json = """{"name":"m",$base,
            "facade_geom":[{"v":[[0,0,0],[4,0,0],[4,0,3],[0,0,3]],"n":[0,-1,0]},
                           {"v":[[9,9,0],[10,9,0],[10,9,2],[9,9,2]],"n":[0,-1,0]}],
            "wp_target":[0,0,-1]}"""
        val s = HomeViewModel.PreviewSummary.fromJson(json.toByteArray())
        assertEquals(2, s.facadeGeom.size)
        assertEquals(4, s.facadeGeom[0].cornerCount)
        assertEquals(4.0, s.facadeGeom[0].v[3], 1e-9)
        assertEquals(-1.0, s.facadeGeom[0].n[1], 1e-9)
        assertEquals(listOf(0, 0, -1), s.wpTargets.toList())
    }

    @Test
    fun recognised_points_and_facet_evidence_parse() {
        val json = """{"name":"m",$base,
            "facade_geom":[{"v":[[0,0,0],[4,0,0],[4,0,3],[0,0,3]],"n":[0,-1,0],"pts":812,
                            "s":[[1,0.01,1],[2,-0.02,2]]},
                           {"v":[[9,9,0],[10,9,0],[10,9,2],[9,9,2]],"n":[0,-1,0],"pts":41}],
            "wp_target":[0,0,-1],
            "coverage":{"facets":219,"facets_targeted":37,"walls":140,"walls_unshot":92}}"""
        val s = HomeViewModel.PreviewSummary.fromJson(json.toByteArray())
        assertEquals(219, s.facets)
        assertEquals(37, s.facetsTargeted)
        assertEquals(92, s.wallsUnshot)
        assertEquals(812, s.facadeGeom[0].inlierCount)
        assertEquals(2, s.facadeGeom[0].sampleCount)
        assertEquals(0.01, s.facadeGeom[0].sample[1], 1e-9)
        // A facet the engine sent without a sample is still usable.
        assertEquals(41, s.facadeGeom[1].inlierCount)
        assertEquals(0, s.facadeGeom[1].sampleCount)
    }

    @Test
    fun old_manifold_summary_without_new_keys_still_parses() {
        val s = HomeViewModel.PreviewSummary.fromJson("""{"name":"m",$base}""".toByteArray())
        assertNull(s.gsdMedianMmPx); assertEquals(0, s.flips); assertTrue(s.stopAtWaypoint)
        assertTrue(s.facadeGeom.isEmpty()); assertEquals(0, s.wpTargets.size)
        assertEquals(setOf(3, 10, 11), s.flaggedIndices)
    }
}
