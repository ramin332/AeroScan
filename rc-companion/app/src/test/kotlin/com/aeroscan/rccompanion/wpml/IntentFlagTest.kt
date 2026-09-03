package com.aeroscan.rccompanion.wpml

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class IntentFlagTest {
    private val kmz = ImportedKmz(
        name = "t", refLat = 52.0, refLon = 4.9, refAlt = 40.0,
        waypoints = emptyList(), missionAreaWgs84 = emptyList(),
    )

    @Test
    fun allow_stale_mesh_is_only_written_when_set() {
        assertFalse(JSONObject(kmz.toJsonString()).has("allow_stale_mesh"))
        val o = JSONObject(kmz.toJsonString(allowStaleMesh = true))
        assertTrue(o.getBoolean("allow_stale_mesh"))
        assertEquals(MissionIntent.SCHEMA_VERSION, o.getInt("schema_version"))
    }
}

/** Planner knobs ride inside the intent so a new one needs no PSDK rebuild. */
class PlannerSettingsTest {
    private val intent = ImportedKmz("m", 52.0, 5.0, 30.0, emptyList(), emptyList())

    @Test
    fun settings_are_absent_unless_supplied() {
        val obj = org.json.JSONObject(intent.toJsonString())
        assertTrue(!obj.has("settings"))
    }

    @Test
    fun speed_and_stop_mode_reach_the_engine() {
        val json = intent.toJsonString(
            settings = PlannerSettings(inspectionSpeedMs = 1.0, stopAtWaypoint = true),
        )
        val s = org.json.JSONObject(json).getJSONObject("settings")
        assertEquals(1.0, s.getDouble("inspection_speed_ms"), 1e-9)
        assertTrue(s.getBoolean("stop_at_waypoint"))
    }

    @Test
    fun normal_detail_sends_no_detection_knobs() {
        // Normal must let the engine keep its cloud-derived estimate; freezing
        // those numbers would stop them adapting to point density.
        val s = org.json.JSONObject(intent.toJsonString(settings = PlannerSettings()))
            .getJSONObject("settings")
        assertTrue(!s.has("fd_min_points"))
        assertTrue(!s.has("fd_epsilon_m"))
    }

    @Test
    fun fine_and_coarse_send_the_full_detection_set() {
        for (d in listOf(PlannerSettings.Detail.Fine, PlannerSettings.Detail.Coarse)) {
            val s = org.json.JSONObject(intent.toJsonString(settings = PlannerSettings(detail = d)))
                .getJSONObject("settings")
            assertEquals(d.minPoints, s.getInt("fd_min_points"))
            assertEquals(d.epsilonM, s.getDouble("fd_epsilon_m"), 1e-9)
            assertEquals(d.clusterEpsilonM, s.getDouble("fd_cluster_epsilon_m"), 1e-9)
            assertEquals(d.minWallAreaM2, s.getDouble("fd_min_wall_area_m2"), 1e-9)
            assertEquals(d.minDensityPerM2, s.getDouble("fd_min_density_per_m2"), 1e-9)
        }
        // Fine must look harder than Coarse, or the labels lie.
        assertTrue(PlannerSettings.Detail.Fine.minPoints < PlannerSettings.Detail.Coarse.minPoints)
        assertTrue(PlannerSettings.Detail.Fine.minWallAreaM2 < PlannerSettings.Detail.Coarse.minWallAreaM2)
    }

    @Test
    fun detection_knobs_stay_inside_the_engine_clamps() {
        // mission_intent.SETTING_KEYS ranges.
        for (d in PlannerSettings.Detail.entries) {
            assertTrue(d.minPoints in 8..2000)
            assertTrue(d.epsilonM in 0.01..0.50)
            assertTrue(d.clusterEpsilonM in 0.05..2.00)
            assertTrue(d.minWallAreaM2 in 0.1..50.0)
            assertTrue(d.minDensityPerM2 in 1.0..400.0)
        }
    }

    @Test
    fun offered_speeds_stay_inside_the_engine_clamp() {
        // mission_intent.SETTING_KEYS clamps inspection_speed_ms to [0.3, 6.0].
        assertTrue(PlannerSettings.SPEED_CHOICES.all { it in 0.3..6.0 })
    }
}
