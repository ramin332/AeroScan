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
    fun a_min_height_gate_reaches_the_engine_and_off_sends_nothing() {
        val gated = org.json.JSONObject(intent.toJsonString(settings = PlannerSettings(minHeightM = 2.0)))
            .getJSONObject("settings")
        assertEquals(2.0, gated.getDouble("min_facade_height_m"), 1e-9)
        val off = org.json.JSONObject(intent.toJsonString(settings = PlannerSettings()))
            .getJSONObject("settings")
        assertTrue(!off.has("min_facade_height_m"))
        // Off must mean no gate, not a gate at zero.
        assertEquals(null, PlannerSettings.MIN_HEIGHT_CHOICES.first())
        assertEquals("Off", PlannerSettings.minHeightLabel(null))
        assertTrue(PlannerSettings.MIN_HEIGHT_CHOICES.filterNotNull().all { it in 0.0..20.0 })
    }

    @Test
    fun offered_speeds_stay_inside_the_engine_clamp() {
        // mission_intent.SETTING_KEYS clamps inspection_speed_ms to [0.3, 6.0].
        assertTrue(PlannerSettings.SPEED_CHOICES.all { it in 0.3..6.0 })
    }
}

/** Aim reach: the lever that actually bounds coverage. */
class ReachSettingTest {
    private val intent = ImportedKmz("m", 52.0, 5.0, 30.0, emptyList(), emptyList())

    private fun settings(json: String) = org.json.JSONObject(json).getJSONObject("settings")

    @Test
    fun auto_sends_nothing_so_the_engine_derives_it_from_the_gsd() {
        val s = settings(intent.toJsonString(settings = PlannerSettings(reachM = null)))
        assertTrue(!s.has("max_facade_distance_m"))
    }

    @Test
    fun an_explicit_reach_reaches_the_engine() {
        val s = settings(intent.toJsonString(settings = PlannerSettings(reachM = 20.0)))
        assertEquals(20.0, s.getDouble("max_facade_distance_m"), 1e-9)
    }

    @Test
    fun offered_reaches_stay_inside_the_engine_clamp_and_start_at_auto() {
        // mission_intent.SETTING_KEYS clamps max_facade_distance_m to [1, 100].
        assertEquals(null, PlannerSettings.REACH_CHOICES.first())
        assertTrue(PlannerSettings.REACH_CHOICES.filterNotNull().all { it in 1.0..100.0 })
        assertEquals("Auto", PlannerSettings.reachLabel(null))
        assertEquals("20 m", PlannerSettings.reachLabel(20.0))
    }
}
