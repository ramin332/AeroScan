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
    fun offered_speeds_stay_inside_the_engine_clamp() {
        // mission_intent.SETTING_KEYS clamps inspection_speed_ms to [0.3, 6.0].
        assertTrue(PlannerSettings.SPEED_CHOICES.all { it in 0.3..6.0 })
    }
}
