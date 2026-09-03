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
    /**
     * [allowStaleMesh]: the pilot chose to augment against a scan older than the
     * Manifold's age limit (6 h). The C runner greps this key and bypasses its
     * gate; the Python side ignores unknown keys.
     */
    fun toJsonString(
        pretty: Boolean = false,
        allowStaleMesh: Boolean = false,
        settings: PlannerSettings? = null,
    ): String {
        val obj = JSONObject().apply {
            put("schema_version", MissionIntent.SCHEMA_VERSION)
            put("name", name)
            if (allowStaleMesh) put("allow_stale_mesh", true)
            settings?.let { put("settings", it.toJson()) }
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


/**
 * Planner knobs the pilot chooses on the RC. They travel inside the mission
 * intent rather than the augment command line because the Manifold's C runner
 * builds a fixed argv — a new knob would otherwise need a PSDK rebuild and a
 * DPK reinstall. The engine clamps every value (mission_intent.SETTING_KEYS)
 * and ignores keys it does not know, so an older Manifold still flies.
 */
data class PlannerSettings(
    /** Transit speed between waypoints. In stop mode the aircraft halts to shoot
     *  regardless, so this trades mission time against nothing but time. */
    val inspectionSpeedMs: Double = 3.0,
    /** Stop at every waypoint to rotate and shoot (DJI toPointAndStop*). */
    val stopAtWaypoint: Boolean = true,
    /**
     * How far a waypoint may aim, metres, or null to let the engine derive it
     * from the target resolution (the distance at which GSD doubles, ~14.6 m
     * for the wide lens at 2 mm/px). Trades resolution against how many
     * waypoints get a target at all: measured 2026-09-03, 10 m gave 1.90 mm/px
     * but left 106 waypoints unaimed, while 20 m aimed all 398 at 2.16 mm/px.
     */
    val reachM: Double? = null,
    /**
     * Ignore facets whose centre sits below this height above the ground, or
     * null for no gate. Ground-level facets pull the aim down onto tarmac and
     * parked cars and waste frames.
     */
    val minHeightM: Double? = null,
    /**
     * Photos taken at each stop. The nose aims at the primary wall; extras pan
     * the gimbal within its travel to walls nothing else photographs. Needs the
     * stop — in fly-through the aircraft has moved on before the sequence ends,
     * which is how 104 of 398 photos were lost on 2026-07-10 with a single shot.
     */
    val shotsPerWaypoint: Int = 1,
) {
    fun toJson(): JSONObject = JSONObject().apply {
        put("inspection_speed_ms", inspectionSpeedMs)
        put("stop_at_waypoint", stopAtWaypoint)
        reachM?.let { put("max_facade_distance_m", it) }
        minHeightM?.let { put("min_facade_height_m", it) }
        put("shots_per_waypoint", if (stopAtWaypoint) shotsPerWaypoint else 1)
    }

    companion object {
        val SPEED_CHOICES = listOf(1.0, 2.0, 3.0)

        /** null = derive from the target GSD. The rest are inside the engine clamp. */
        val REACH_CHOICES: List<Double?> = listOf(null, 10.0, 20.0, 30.0)

        /**
         * One or two. Measured on the aircraft 2026-09-03: three produced a
         * byte-identical mission to two — with the nose on the primary wall
         * there is never a third unshot wall inside the gimbal's pan window.
         */
        val SHOT_CHOICES = listOf(1, 2)

        /** null = no height gate. */
        val MIN_HEIGHT_CHOICES: List<Double?> = listOf(null, 1.0, 2.0, 3.0)

        fun reachLabel(v: Double?): String = v?.let { "${it.toInt()} m" } ?: "Auto"

        fun minHeightLabel(v: Double?): String = v?.let { "${it.toInt()} m" } ?: "Off"
    }
}
