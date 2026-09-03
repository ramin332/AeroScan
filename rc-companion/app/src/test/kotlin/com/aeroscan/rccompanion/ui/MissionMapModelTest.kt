package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.cloud.PlyParser
import com.aeroscan.rccompanion.wpml.ImportedKmz
import com.aeroscan.rccompanion.wpml.ParsedWaypoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class MissionMapModelTest {
    private val refLat = 52.0
    private val refLon = 5.0

    private fun wp(i: Int, dE: Double, dN: Double, heading: Double) = ParsedWaypoint(
        index = i,
        lon = refLon + dE / (111_320.0 * Math.cos(Math.toRadians(refLat))),
        lat = refLat + dN / 111_320.0,
        altEgm96 = 30.0, headingDeg = heading, gimbalPitchDeg = -10.0,
    )

    private fun kmz(vararg wps: ParsedWaypoint, poly: List<DoubleArray> = emptyList()) =
        ImportedKmz("t", refLat, refLon, 0.0, wps.toList(), poly)

    @Test
    fun enu_projection_round_trips_metres() {
        val en = enuXY(wp(0, 10.0, -20.0, 0.0).lat, wp(0, 10.0, -20.0, 0.0).lon, refLat, refLon)
        assertEquals(10.0, en[0], 0.01); assertEquals(-20.0, en[1], 0.01)
    }

    @Test
    fun path_polygon_and_bounds() {
        val m = buildMissionMap(kmz(wp(0, 0.0, 0.0, 90.0), wp(1, 10.0, 0.0, 180.0),
            poly = listOf(doubleArrayOf(refLon, refLat, 0.0), doubleArrayOf(refLon, refLat + 30 / 111_320.0, 0.0))), null)
        assertEquals(2, m.waypointCount)
        assertEquals(90.0, m.headingsOriginal[0], 0.0)
        assertNull(m.headingsAugmented)
        assertEquals(4, m.polygonXY.size)
        // bounds = union(path, polygon) + 5 m pad
        assertEquals(-5.0, m.minE, 0.01); assertEquals(15.0, m.maxE, 0.01)
        assertEquals(-5.0, m.minN, 0.01); assertEquals(35.0, m.maxN, 0.01)
    }

    @Test
    fun augmented_headings_only_when_counts_match() {
        val orig = kmz(wp(0, 0.0, 0.0, 0.0), wp(1, 5.0, 0.0, 0.0))
        val aug = kmz(wp(0, 0.0, 0.0, 45.0), wp(1, 5.0, 0.0, 135.0))
        val m = buildMissionMap(orig, null, aug, flagged = setOf(1, 7))
        assertNotNull(m.headingsAugmented)
        assertEquals(135.0, m.headingsAugmented!![1], 0.0)
        assertEquals(setOf(1), m.flagged)  // 7 is out of range → dropped
        val mismatch = buildMissionMap(orig, null, kmz(wp(0, 0.0, 0.0, 45.0)))
        assertNull(mismatch.headingsAugmented)
    }

    @Test
    fun cloud_is_decimated_and_kept_when_near_the_path() {
        val n = 10_000
        val xyz = FloatArray(n * 3) { i -> when (i % 3) { 0 -> (i / 3 % 100).toFloat(); 1 -> (i / 3 / 100).toFloat(); else -> 0f } }
        val m = buildMissionMap(kmz(wp(0, 50.0, 50.0, 0.0)), PlyParser.XyzCloud(xyz))
        assertTrue("decimated to <= cap", m.cloudPointCount in 1..MAP_CLOUD_MAX_POINTS)
        assertTrue("bounds grew to the cloud", m.maxE >= 99.0 - 1e-6 + MAP_PAD_M - MAP_PAD_M)
    }

    @Test
    fun cloud_in_another_frame_is_dropped() {
        val xyz = floatArrayOf(5000f, 5000f, 0f, 5001f, 5000f, 0f)
        val m = buildMissionMap(kmz(wp(0, 0.0, 0.0, 0.0)), PlyParser.XyzCloud(xyz))
        assertEquals(0, m.cloudPointCount)
        assertTrue(m.maxE < 100)
    }
}
