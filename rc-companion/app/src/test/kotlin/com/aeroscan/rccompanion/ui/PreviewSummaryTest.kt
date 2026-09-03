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
    fun old_manifold_summary_without_new_keys_still_parses() {
        val s = HomeViewModel.PreviewSummary.fromJson("""{"name":"m",$base}""".toByteArray())
        assertNull(s.gsdMedianMmPx); assertEquals(0, s.flips); assertTrue(s.stopAtWaypoint)
        assertEquals(setOf(3, 10, 11), s.flaggedIndices)
    }
}
