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
