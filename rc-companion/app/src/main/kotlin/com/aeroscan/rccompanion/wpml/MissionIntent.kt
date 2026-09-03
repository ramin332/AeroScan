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
    /** How hard the extractor looks for facades. */
    val detail: Detail = Detail.Normal,
) {
    /**
     * Facade-detection presets. The engine takes five separate numbers; a pilot
     * on site can reason about one axis — "it missed the small stuff" versus
     * "it invented walls out of noise" — so the RC offers that axis and the
     * engine clamps whatever arrives.
     */
    enum class Detail(
        val label: String,
        val minPoints: Int,
        val epsilonM: Double,
        val clusterEpsilonM: Double,
        val minWallAreaM2: Double,
        val minDensityPerM2: Double,
    ) {
        /** Small features: window sills, parapets, balcony panels. More noise facets. */
        Fine("Fine", 20, 0.035, 0.18, 0.3, 15.0),
        /** The engine's own cloud-derived defaults. */
        Normal("Normal", 40, 0.05, 0.25, 0.5, 25.0),
        /**
         * Whole walls only. Fewer, larger, more certain facets.
         *
         * The cluster radius deliberately matches Normal. Measured on the
         * Manifold 2026-09-03: 0.40 took 3626 s and 0.25 took 16 s for the same
         * 48 facets. Coarseness comes from the filters, never from widening the
         * neighbour search.
         */
        Coarse("Coarse", 90, 0.08, 0.25, 1.5, 40.0),
    }

    fun toJson(): JSONObject = JSONObject().apply {
        put("inspection_speed_ms", inspectionSpeedMs)
        put("stop_at_waypoint", stopAtWaypoint)
        // Normal means "let the engine estimate from the cloud" — sending its
        // numbers would freeze an estimate that adapts to point density.
        if (detail != Detail.Normal) {
            put("fd_min_points", detail.minPoints)
            put("fd_epsilon_m", detail.epsilonM)
            put("fd_cluster_epsilon_m", detail.clusterEpsilonM)
            put("fd_min_wall_area_m2", detail.minWallAreaM2)
            put("fd_min_density_per_m2", detail.minDensityPerM2)
        }
    }

    companion object {
        val SPEED_CHOICES = listOf(1.0, 2.0, 3.0)
    }
}
