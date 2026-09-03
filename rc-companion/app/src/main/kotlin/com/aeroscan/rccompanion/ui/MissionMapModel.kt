package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.cloud.PlyParser
import com.aeroscan.rccompanion.wpml.ImportedKmz
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Pure, MSDK-free model behind the mission view (2D map and 3D scene).
 * Everything is in the mission's local-ENU frame — east/north/up metres around
 * the KMZ reference point. That is the same frame the KMZ's cloud.ply uses
 * (sfm_geo_desc ref) AND the frame the Manifold reports facades in, so cloud,
 * waypoints and facades line up without any registration on this side.
 */

// Metres per degree at a reference latitude. Same series the Python engine uses
// (flight_planner/models.py meters_per_deg) — a flat 111_320 approximation is
// ~0.4 m off over 200 m of easting at NL latitudes, which would visibly offset
// the Manifold's facades from our waypoints.
private const val WGS84_LAT_A = 111_132.92
private const val WGS84_LAT_B = -559.82
private const val WGS84_LAT_C = 1.175
private const val WGS84_LON_A = 111_412.84
private const val WGS84_LON_B = -93.5

fun metersPerDeg(refLatDeg: Double): DoubleArray {
    val r = refLatDeg * PI / 180.0
    return doubleArrayOf(
        WGS84_LAT_A + WGS84_LAT_B * cos(2 * r) + WGS84_LAT_C * cos(4 * r),
        WGS84_LON_A * cos(r) + WGS84_LON_B * cos(3 * r),
    )
}

/** WGS84 → east/north metres around the reference point. */
fun enuXY(lat: Double, lon: Double, refLat: Double, refLon: Double): DoubleArray {
    val m = metersPerDeg(refLat)
    return doubleArrayOf((lon - refLon) * m[1], (lat - refLat) * m[0])
}

/** One facade rectangle as the Manifold extracted it. */
data class FacadeQuad(
    /** Corner vertices, 3 doubles each, in mission ENU. */
    val v: DoubleArray,
    /** Outward normal. */
    val n: DoubleArray,
    /** How many waypoints are aimed at this facade. */
    val waypoints: Int,
    /** How many cloud points the extractor assigned to this facet. */
    val inlierCount: Int = 0,
    /** A sample of those points, 3 doubles each — what the 3D view draws. */
    val sample: DoubleArray = DoubleArray(0),
) {
    val sampleCount: Int get() = sample.size / 3
    val cornerCount: Int get() = v.size / 3
    val covered: Boolean get() = waypoints > 0
    fun cx(): Double { var s = 0.0; for (i in 0 until cornerCount) s += v[3 * i]; return s / cornerCount }
    fun cy(): Double { var s = 0.0; for (i in 0 until cornerCount) s += v[3 * i + 1]; return s / cornerCount }
    fun cz(): Double { var s = 0.0; for (i in 0 until cornerCount) s += v[3 * i + 2]; return s / cornerCount }
}

data class MissionMapData(
    /** E,N,U triples, one per waypoint, flight order. */
    val pathXYZ: DoubleArray,
    /** Aircraft heading per waypoint as the KMZ commands it (deg, 0 = north, clockwise). */
    val headingsOriginal: DoubleArray,
    /** Gimbal pitch per waypoint as the KMZ commands it (deg, 0 = level, −90 = down). */
    val pitchesOriginal: DoubleArray,
    /** Headings after augmentation, or null before the preview lands. */
    val headingsAugmented: DoubleArray?,
    /** Gimbal pitch after augmentation, or null before the preview lands. */
    val pitchesAugmented: DoubleArray?,
    /** Facade each waypoint is aimed at (index into [facades]), −1 = none. Empty before the preview. */
    val targets: IntArray,
    /** Facade rectangles from the Manifold. Empty before the preview. */
    val facades: List<FacadeQuad>,
    /** Waypoint indices the pilot should look at (anomalies + validation hits). */
    val flagged: Set<Int>,
    /** Mission-area polygon as E,N pairs (may be empty). */
    val polygonXY: DoubleArray,
    /** Decimated cloud as E,N,U triples (may be empty). */
    val cloudXYZ: FloatArray,
    val minE: Double,
    val minN: Double,
    val maxE: Double,
    val maxN: Double,
    val minU: Double,
    val maxU: Double,
) {
    val waypointCount: Int get() = pathXYZ.size / 3
    val widthM: Double get() = maxE - minE
    val heightM: Double get() = maxN - minN
    val cloudPointCount: Int get() = cloudXYZ.size / 3
    val uncoveredFacades: Int get() = facades.count { !it.covered }
    /** Points the extractor recognised as belonging to some facade. */
    val recognisedPoints: Int get() = facades.sumOf { it.inlierCount }

    fun wpE(i: Int) = pathXYZ[3 * i]
    fun wpN(i: Int) = pathXYZ[3 * i + 1]
    fun wpU(i: Int) = pathXYZ[3 * i + 2]

    /** Scene diagonal — the natural unit for camera distance and ray lengths. */
    val sceneSpanM: Double
        get() = sqrt(widthM * widthM + heightM * heightM + (maxU - minU) * (maxU - minU))
}

/** Cap on drawn cloud points — a Canvas with 400 K dots is unusable on the RC. */
const val MAP_CLOUD_MAX_POINTS = 4000

/**
 * If the cloud's centroid sits further than this from the waypoints' centroid
 * the cloud is in some other frame (no sfm_geo_desc, odd KMZ) — draw nothing
 * rather than a footprint in the wrong place.
 */
const val MAP_CLOUD_MAX_OFFSET_M = 300.0

/** Padding around the drawn extent, metres. */
const val MAP_PAD_M = 5.0

fun buildMissionMap(
    original: ImportedKmz,
    cloud: PlyParser.XyzCloud?,
    augmented: ImportedKmz? = null,
    flagged: Set<Int> = emptySet(),
    facades: List<FacadeQuad> = emptyList(),
    targets: IntArray = IntArray(0),
): MissionMapData {
    val refLat = original.refLat
    val refLon = original.refLon
    val refAlt = original.refAlt
    val m = metersPerDeg(refLat)
    val n = original.waypoints.size
    val path = DoubleArray(n * 3)
    val hOrig = DoubleArray(n)
    val pOrig = DoubleArray(n)
    var minE = Double.POSITIVE_INFINITY; var minN = Double.POSITIVE_INFINITY
    var maxE = Double.NEGATIVE_INFINITY; var maxN = Double.NEGATIVE_INFINITY
    var minU = Double.POSITIVE_INFINITY; var maxU = Double.NEGATIVE_INFINITY
    fun grow(e: Double, no: Double, u: Double) {
        minE = min(minE, e); maxE = max(maxE, e)
        minN = min(minN, no); maxN = max(maxN, no)
        minU = min(minU, u); maxU = max(maxU, u)
    }
    var sumE = 0.0; var sumN = 0.0
    original.waypoints.forEachIndexed { i, wp ->
        val e = (wp.lon - refLon) * m[1]
        val no = (wp.lat - refLat) * m[0]
        val u = wp.altEgm96 - refAlt
        path[3 * i] = e; path[3 * i + 1] = no; path[3 * i + 2] = u
        hOrig[i] = wp.headingDeg
        pOrig[i] = wp.gimbalPitchDeg
        sumE += e; sumN += no
        grow(e, no, u)
    }

    val poly = DoubleArray(original.missionAreaWgs84.size * 2)
    original.missionAreaWgs84.forEachIndexed { i, p ->
        val e = (p[0] - refLon) * m[1]
        val no = (p[1] - refLat) * m[0]
        poly[2 * i] = e; poly[2 * i + 1] = no
        grow(e, no, 0.0)
    }

    // Augmented angles only line up if the augmenter kept the waypoint count
    // (it does — it rewrites headings/pitches in place). Anything else: ignore.
    val aug = augmented?.takeIf { it.waypoints.size == n }
    val hAug: DoubleArray? = aug?.let { a -> DoubleArray(n) { a.waypoints[it].headingDeg } }
    val pAug: DoubleArray? = aug?.let { a -> DoubleArray(n) { a.waypoints[it].gimbalPitchDeg } }

    // Facade coverage from the per-waypoint targets the Manifold reported.
    val hits = IntArray(facades.size)
    for (t in targets) if (t in facades.indices) hits[t]++
    val facadesWithCoverage = facades.mapIndexed { i, f -> f.copy(waypoints = hits[i]) }
    for (f in facadesWithCoverage) {
        for (i in 0 until f.cornerCount) grow(f.v[3 * i], f.v[3 * i + 1], f.v[3 * i + 2])
    }

    var cloudXYZ = FloatArray(0)
    if (cloud != null && cloud.pointCount > 0 && n > 0) {
        val total = cloud.pointCount
        val step = max(1, (total + MAP_CLOUD_MAX_POINTS - 1) / MAP_CLOUD_MAX_POINTS)
        val kept = (total + step - 1) / step
        val out = FloatArray(kept * 3)
        var cE = 0.0; var cN = 0.0; var k = 0
        var i = 0
        while (i < total) {
            out[3 * k] = cloud.xyz[3 * i]
            out[3 * k + 1] = cloud.xyz[3 * i + 1]
            out[3 * k + 2] = cloud.xyz[3 * i + 2]
            cE += out[3 * k]; cN += out[3 * k + 1]; k++
            i += step
        }
        val offset = hypot(cE / k - sumE / n, cN / k - sumN / n)
        if (offset <= MAP_CLOUD_MAX_OFFSET_M) {
            cloudXYZ = out
            for (j in 0 until k) grow(out[3 * j].toDouble(), out[3 * j + 1].toDouble(), out[3 * j + 2].toDouble())
        }
    }

    if (minE > maxE) { minE = -1.0; maxE = 1.0; minN = -1.0; maxN = 1.0 }
    if (minU > maxU) { minU = 0.0; maxU = 1.0 }
    return MissionMapData(
        pathXYZ = path,
        headingsOriginal = hOrig,
        pitchesOriginal = pOrig,
        headingsAugmented = hAug,
        pitchesAugmented = pAug,
        targets = if (targets.size == n) targets else IntArray(0),
        facades = facadesWithCoverage,
        flagged = flagged.filter { it in 0 until n }.toSet(),
        polygonXY = poly,
        cloudXYZ = cloudXYZ,
        minE = minE - MAP_PAD_M, minN = minN - MAP_PAD_M,
        maxE = maxE + MAP_PAD_M, maxN = maxN + MAP_PAD_M,
        minU = minU, maxU = maxU,
    )
}

// ------------------------------------------------------------------ aiming

/**
 * Unit direction the camera looks along, from an aircraft heading (deg, 0 = north,
 * clockwise) and a gimbal pitch (deg, 0 = level, −90 = straight down). Returns
 * east, north, up. The nose carries azimuth — the M4E ignores a commanded gimbal
 * yaw (2026-07-10), which is why heading alone sets the bearing here.
 */
fun aimDirection(headingDeg: Double, pitchDeg: Double): DoubleArray {
    val h = headingDeg * PI / 180.0
    val p = pitchDeg * PI / 180.0
    val horiz = cos(p)
    return doubleArrayOf(sin(h) * horiz, cos(h) * horiz, sin(p))
}

/** Where the camera ray ends: at its target facade if it has one, else a fixed reach. */
fun aimRayLength(data: MissionMapData, wp: Int, fallbackM: Double): Double {
    if (wp !in data.targets.indices) return fallbackM
    val t = data.targets[wp]
    if (t !in data.facades.indices) return fallbackM
    val f = data.facades[t]
    val d = sqrt(
        (f.cx() - data.wpE(wp)).let { it * it } +
            (f.cy() - data.wpN(wp)).let { it * it } +
            (f.cz() - data.wpU(wp)).let { it * it },
    )
    return if (d.isFinite() && d > 0.5) d else fallbackM
}

// ------------------------------------------------------------- 3D camera

/**
 * Orbit camera for the 3D scene. Pure maths so it can be unit-tested without a
 * Canvas: [project] maps a mission-ENU point to viewport pixels plus a depth.
 */
data class OrbitCamera(
    /** Degrees; 0 = eye south of the scene looking north, increasing clockwise. */
    val azimuthDeg: Double = 30.0,
    /** Degrees above the horizon; clamped to a sane band by [withDelta]. */
    val elevationDeg: Double = 22.0,
    /** Multiplier on the scene span for the eye distance. */
    val zoom: Double = 1.0,
) {
    /**
     * Apply a drag and a pinch. The scene follows the finger: dragging right
     * turns the scene to the right, which means orbiting the eye the other way —
     * hence the sign flip on the horizontal pan. Dragging down lifts the eye, so
     * you look further down onto the site.
     */
    fun withDrag(panX: Float, panY: Float, zoomFactor: Float) = withDelta(
        dAzimuth = panX / DRAG_DEG_PER_PX,
        dElevation = panY / DRAG_DEG_PER_PX,
        dZoom = zoomFactor.toDouble(),
    )

    fun withDelta(dAzimuth: Double, dElevation: Double, dZoom: Double) = OrbitCamera(
        azimuthDeg = ((azimuthDeg + dAzimuth) % 360.0 + 360.0) % 360.0,
        elevationDeg = (elevationDeg + dElevation).coerceIn(MIN_ELEVATION_DEG, MAX_ELEVATION_DEG),
        zoom = (zoom * dZoom).coerceIn(MIN_ZOOM, MAX_ZOOM),
    )

    companion object {
        /** Pixels of drag per degree of orbit. */
        const val DRAG_DEG_PER_PX = 4.0
        const val MIN_ELEVATION_DEG = 2.0
        const val MAX_ELEVATION_DEG = 85.0
        const val MIN_ZOOM = 0.25
        const val MAX_ZOOM = 4.0
    }
}

/** Screen position (x, y) plus camera-space depth; depth ≤ 0 means behind the eye. */
data class Projected(val x: Float, val y: Float, val depth: Double)

class SceneProjector(
    data: MissionMapData,
    private val camera: OrbitCamera,
    private val viewW: Float,
    private val viewH: Float,
) {
    private val cx = (data.minE + data.maxE) / 2.0
    private val cy = (data.minN + data.maxN) / 2.0
    private val cz = (data.minU + data.maxU) / 2.0
    private val span = max(1.0, data.sceneSpanM)
    private val eyeDist = span * 1.05 / camera.zoom

    // Camera basis. Forward points from the eye toward the scene centre, so at
    // azimuth 0 the eye sits south of the scene looking north — the same
    // orientation as the top-down map, where north is up.
    //   forward = (sin az cos el,  cos az cos el, -sin el)
    //   right   = forward × world-up, normalised = (cos az, -sin az, 0)
    //   up      = right × forward = (sin az sin el, cos az sin el, cos el)
    private val az = camera.azimuthDeg * PI / 180.0
    private val el = camera.elevationDeg * PI / 180.0
    private val fe = sin(az) * cos(el)
    private val fn = cos(az) * cos(el)
    private val fu = -sin(el)
    private val re = cos(az)
    private val rn = -sin(az)
    private val ue = sin(az) * sin(el)
    private val un = cos(az) * sin(el)
    private val uu = cos(el)

    /** Focal length in pixels: fits the scene span across the smaller viewport axis. */
    private val focal = (min(viewW, viewH) * 1.05 / (span / eyeDist)).toDouble()

    private val eyeE = cx - fe * eyeDist
    private val eyeN = cy - fn * eyeDist
    private val eyeU = cz - fu * eyeDist

    fun project(e: Double, n: Double, u: Double): Projected {
        val de = e - eyeE; val dn = n - eyeN; val du = u - eyeU
        val depth = de * fe + dn * fn + du * fu
        if (depth <= 0.1) return Projected(0f, 0f, depth)
        val rx = de * re + dn * rn
        val ry = de * ue + dn * un + du * uu
        val s = focal / depth
        return Projected(
            (viewW / 2.0 + rx * s).toFloat(),
            (viewH / 2.0 - ry * s).toFloat(),
            depth,
        )
    }

    /** Metres-per-pixel at a given depth — used to size ground markers sensibly. */
    fun scaleAt(depth: Double): Double = if (depth > 0.1) focal / depth else 0.0
}

/**
 * How far off a facade the camera is pointing, in degrees, or null when the
 * waypoint has no target. Drives the "check this" colouring.
 */
fun aimErrorDeg(data: MissionMapData, wp: Int, headings: DoubleArray, pitches: DoubleArray): Double? {
    if (wp !in data.targets.indices) return null
    val t = data.targets[wp]
    if (t !in data.facades.indices) return null
    val f = data.facades[t]
    val de = f.cx() - data.wpE(wp)
    val dn = f.cy() - data.wpN(wp)
    val du = f.cz() - data.wpU(wp)
    val len = sqrt(de * de + dn * dn + du * du)
    if (len < 1e-6) return null
    val a = aimDirection(headings[wp], pitches[wp])
    val dot = ((a[0] * de + a[1] * dn + a[2] * du) / len).coerceIn(-1.0, 1.0)
    return abs(kotlin.math.acos(dot) * 180.0 / PI)
}
