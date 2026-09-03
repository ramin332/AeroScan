package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.cloud.PlyParser
import com.aeroscan.rccompanion.wpml.ImportedKmz
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/**
 * Pure, MSDK-free model behind the top-down mission map on the home screen.
 * Everything is in local east/north metres around the KMZ reference point —
 * the same frame the KMZ's cloud.ply uses (sfm_geo_desc ref), so the cloud
 * footprint and the waypoints line up without any registration.
 */

/** Equirectangular WGS84 → east/north metres. Fine for a sub-2 km site. */
fun enuXY(lat: Double, lon: Double, refLat: Double, refLon: Double): DoubleArray {
    val m = 111_320.0
    val e = (lon - refLon) * m * cos(refLat * PI / 180.0)
    val n = (lat - refLat) * m
    return doubleArrayOf(e, n)
}

data class MissionMapData(
    /** E,N pairs, one per waypoint, flight order. */
    val pathXY: DoubleArray,
    /** Aircraft heading per waypoint as the KMZ commands it (deg, 0 = north, clockwise). */
    val headingsOriginal: DoubleArray,
    /** Headings after augmentation, same length as [headingsOriginal], or null before the preview. */
    val headingsAugmented: DoubleArray?,
    /** Waypoint indices the pilot should look at (anomalies + validation hits). */
    val flagged: Set<Int>,
    /** Mission-area polygon as E,N pairs (may be empty). */
    val polygonXY: DoubleArray,
    /** Decimated cloud footprint as E,N pairs (may be empty). */
    val cloudXY: FloatArray,
    val minE: Double,
    val minN: Double,
    val maxE: Double,
    val maxN: Double,
) {
    val waypointCount: Int get() = pathXY.size / 2
    val widthM: Double get() = maxE - minE
    val heightM: Double get() = maxN - minN
    val cloudPointCount: Int get() = cloudXY.size / 2
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
): MissionMapData {
    val refLat = original.refLat
    val refLon = original.refLon
    val n = original.waypoints.size
    val path = DoubleArray(n * 2)
    val hOrig = DoubleArray(n)
    var minE = Double.POSITIVE_INFINITY; var minN = Double.POSITIVE_INFINITY
    var maxE = Double.NEGATIVE_INFINITY; var maxN = Double.NEGATIVE_INFINITY
    fun grow(e: Double, no: Double) {
        minE = min(minE, e); maxE = max(maxE, e); minN = min(minN, no); maxN = max(maxN, no)
    }
    var sumE = 0.0; var sumN = 0.0
    original.waypoints.forEachIndexed { i, wp ->
        val en = enuXY(wp.lat, wp.lon, refLat, refLon)
        path[2 * i] = en[0]; path[2 * i + 1] = en[1]
        hOrig[i] = wp.headingDeg
        sumE += en[0]; sumN += en[1]
        grow(en[0], en[1])
    }

    val poly = DoubleArray(original.missionAreaWgs84.size * 2)
    original.missionAreaWgs84.forEachIndexed { i, p ->
        val en = enuXY(p[1], p[0], refLat, refLon)
        poly[2 * i] = en[0]; poly[2 * i + 1] = en[1]
        grow(en[0], en[1])
    }

    // Augmented headings only line up if the augmenter kept the waypoint count
    // (it does — it rewrites headings/pitches in place). Anything else: ignore.
    val hAug: DoubleArray? = augmented?.takeIf { it.waypoints.size == n }
        ?.let { a -> DoubleArray(n) { a.waypoints[it].headingDeg } }

    var cloudXY = FloatArray(0)
    if (cloud != null && cloud.pointCount > 0 && n > 0) {
        val total = cloud.pointCount
        val step = max(1, (total + MAP_CLOUD_MAX_POINTS - 1) / MAP_CLOUD_MAX_POINTS)
        val kept = (total + step - 1) / step
        val out = FloatArray(kept * 2)
        var cE = 0.0; var cN = 0.0; var k = 0
        var i = 0
        while (i < total) {
            val x = cloud.xyz[3 * i]; val y = cloud.xyz[3 * i + 1]
            out[2 * k] = x; out[2 * k + 1] = y
            cE += x; cN += y; k++
            i += step
        }
        val pathCentroidE = sumE / n; val pathCentroidN = sumN / n
        val offset = hypot(cE / k - pathCentroidE, cN / k - pathCentroidN)
        if (offset <= MAP_CLOUD_MAX_OFFSET_M) {
            cloudXY = out
            for (j in 0 until k) grow(out[2 * j].toDouble(), out[2 * j + 1].toDouble())
        }
    }

    if (minE > maxE) { minE = -1.0; maxE = 1.0; minN = -1.0; maxN = 1.0 }
    return MissionMapData(
        pathXY = path,
        headingsOriginal = hOrig,
        headingsAugmented = hAug,
        flagged = flagged.filter { it in 0 until n }.toSet(),
        polygonXY = poly,
        cloudXY = cloudXY,
        minE = minE - MAP_PAD_M, minN = minN - MAP_PAD_M,
        maxE = maxE + MAP_PAD_M, maxN = maxN + MAP_PAD_M,
    )
}
