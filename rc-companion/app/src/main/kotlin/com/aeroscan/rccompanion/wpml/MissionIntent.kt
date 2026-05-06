package com.aeroscan.rccompanion.wpml

import org.json.JSONArray
import org.json.JSONObject

/**
 * Compact mission intent — the JSON wire format shipped from rc-companion to
 * kmz_runner over MOP (AUGM frame body, prefixed by an int32 length).
 *
 * Schema mirrors `flight_planner.kmz_import.ParsedWaypoint` and
 * `flight_planner.mission_intent.imported_kmz_to_intent_dict` exactly so the
 * Python side can decode without translation. Source of truth for field
 * names: `flight_planner/mission_intent.py` (`SCHEMA_VERSION = 1`).
 *
 * Bumping [SCHEMA_VERSION] requires coordinated changes on the Python side.
 */
object MissionIntent {
    const val SCHEMA_VERSION = 1
}

data class SmartObliquePose(
    val pitchDeg: Double,
    val yawOffsetDeg: Double,
    val rollDeg: Double = 0.0,
) {
    fun toJson(): JSONObject = JSONObject().apply {
        put("pitch_deg", pitchDeg)
        put("yaw_offset_deg", yawOffsetDeg)
        put("roll_deg", rollDeg)
    }
}

data class ParsedWaypoint(
    val index: Int,
    val lon: Double,
    val lat: Double,
    val altEgm96: Double,
    val headingDeg: Double,
    val gimbalPitchDeg: Double,
    val speedMs: Double = 2.0,
    val gimbalYawRawDeg: Double = 0.0,
    val gimbalHeadingMode: String = "smoothTransition",
    val gimbalYawBase: String = "aircraft",
    val smartObliquePoses: List<SmartObliquePose> = emptyList(),
) {
    fun toJson(): JSONObject = JSONObject().apply {
        put("index", index)
        put("lon", lon)
        put("lat", lat)
        put("alt_egm96", altEgm96)
        put("heading_deg", headingDeg)
        put("gimbal_pitch_deg", gimbalPitchDeg)
        put("speed_ms", speedMs)
        put("gimbal_yaw_raw_deg", gimbalYawRawDeg)
        put("gimbal_heading_mode", gimbalHeadingMode)
        put("gimbal_yaw_base", gimbalYawBase)
        val poses = JSONArray()
        for (p in smartObliquePoses) poses.put(p.toJson())
        put("smart_oblique_poses", poses)
    }
}

data class ImportedKmz(
    val name: String,
    val refLat: Double,
    val refLon: Double,
    val refAlt: Double,
    val waypoints: List<ParsedWaypoint>,
    /** mission area polygon as [lon, lat, alt] triples, WGS84. */
    val missionAreaWgs84: List<DoubleArray>,
) {
    fun toJsonString(pretty: Boolean = false): String {
        val obj = JSONObject().apply {
            put("schema_version", MissionIntent.SCHEMA_VERSION)
            put("name", name)
            val ref = JSONObject().apply {
                put("lat", refLat); put("lon", refLon); put("alt", refAlt)
            }
            put("ref", ref)
            val poly = JSONArray()
            for (p in missionAreaWgs84) {
                val tri = JSONArray().apply { put(p[0]); put(p[1]); put(p[2]) }
                poly.put(tri)
            }
            put("mission_area_wgs84", poly)
            val wps = JSONArray()
            for (wp in waypoints) wps.put(wp.toJson())
            put("waypoints", wps)
        }
        return if (pretty) obj.toString(2) else obj.toString()
    }
}
