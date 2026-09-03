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
    private val refAlt = 30.0
    private val m = metersPerDeg(refLat)

    private fun wp(i: Int, dE: Double, dN: Double, heading: Double, pitch: Double = -20.0, dU: Double = 0.0) =
        ParsedWaypoint(
            index = i,
            lon = refLon + dE / m[1],
            lat = refLat + dN / m[0],
            altEgm96 = refAlt + dU,
            headingDeg = heading,
            gimbalPitchDeg = pitch,
        )

    private fun kmz(vararg wps: ParsedWaypoint, poly: List<DoubleArray> = emptyList()) =
        ImportedKmz("t", refLat, refLon, refAlt, wps.toList(), poly)

    @Test
    fun enu_matches_the_python_engine_series_not_a_flat_approximation() {
        // flight_planner/models.py meters_per_deg at 52°: 111267.3153 / 68678.0099
        assertEquals(111_267.3153, m[0], 1e-3)
        assertEquals(68_678.0099, m[1], 1e-3)
        val en = enuXY(wp(0, 10.0, -20.0, 0.0).lat, wp(0, 10.0, -20.0, 0.0).lon, refLat, refLon)
        assertEquals(10.0, en[0], 0.01); assertEquals(-20.0, en[1], 0.01)
    }

    @Test
    fun path_altitude_is_relative_to_the_kmz_reference() {
        val d = buildMissionMap(kmz(wp(0, 0.0, 0.0, 0.0, dU = 12.0)), null)
        assertEquals(12.0, d.wpU(0), 1e-6)
    }

    @Test
    fun path_polygon_and_bounds() {
        val d = buildMissionMap(
            kmz(
                wp(0, 0.0, 0.0, 90.0), wp(1, 10.0, 0.0, 180.0),
                poly = listOf(doubleArrayOf(refLon, refLat, 0.0), doubleArrayOf(refLon, refLat + 30 / m[0], 0.0)),
            ),
            null,
        )
        assertEquals(2, d.waypointCount)
        assertEquals(90.0, d.headingsOriginal[0], 0.0)
        assertNull(d.headingsAugmented)
        assertEquals(4, d.polygonXY.size)
        assertEquals(-5.0, d.minE, 0.01); assertEquals(15.0, d.maxE, 0.01)
        assertEquals(-5.0, d.minN, 0.01); assertEquals(35.0, d.maxN, 0.01)
    }

    @Test
    fun augmented_headings_and_pitches_only_when_counts_match() {
        val orig = kmz(wp(0, 0.0, 0.0, 0.0), wp(1, 5.0, 0.0, 0.0))
        val aug = kmz(wp(0, 0.0, 0.0, 45.0, pitch = -5.0), wp(1, 5.0, 0.0, 135.0, pitch = -80.0))
        val d = buildMissionMap(orig, null, aug, flagged = setOf(1, 7))
        assertNotNull(d.headingsAugmented)
        assertEquals(135.0, d.headingsAugmented!![1], 0.0)
        assertEquals(-80.0, d.pitchesAugmented!![1], 0.0)
        assertEquals(setOf(1), d.flagged) // 7 is out of range → dropped
        assertNull(buildMissionMap(orig, null, kmz(wp(0, 0.0, 0.0, 45.0))).headingsAugmented)
    }

    @Test
    fun facade_coverage_comes_from_the_per_waypoint_targets() {
        val quad = { e: Double ->
            FacadeQuad(
                v = doubleArrayOf(e, 5.0, 0.0, e + 4, 5.0, 0.0, e + 4, 5.0, 3.0, e, 5.0, 3.0),
                n = doubleArrayOf(0.0, -1.0, 0.0), waypoints = 0,
            )
        }
        val d = buildMissionMap(
            kmz(wp(0, 0.0, 0.0, 0.0), wp(1, 5.0, 0.0, 0.0)), null,
            facades = listOf(quad(0.0), quad(20.0)),
            targets = intArrayOf(0, 0),
        )
        assertEquals(2, d.facades[0].waypoints)
        assertEquals(0, d.facades[1].waypoints)
        assertTrue(d.facades[0].covered)
        assertEquals(1, d.uncoveredFacades)
        // Bounds grew to include the far facade.
        assertTrue("maxE ${d.maxE}", d.maxE >= 24.0)
    }

    @Test
    fun targets_of_the_wrong_length_are_ignored() {
        val d = buildMissionMap(kmz(wp(0, 0.0, 0.0, 0.0)), null, targets = intArrayOf(0, 1, 2))
        assertEquals(0, d.targets.size)
    }

    @Test
    fun cloud_is_decimated_and_keeps_its_z() {
        val n = 10_000
        val xyz = FloatArray(n * 3) { i ->
            when (i % 3) { 0 -> (i / 3 % 100).toFloat(); 1 -> (i / 3 / 100).toFloat(); else -> 7f }
        }
        val d = buildMissionMap(kmz(wp(0, 50.0, 50.0, 0.0)), PlyParser.XyzCloud(xyz))
        assertTrue(d.cloudPointCount in 1..MAP_CLOUD_MAX_POINTS)
        assertEquals(7f, d.cloudXYZ[2], 0f)
        assertTrue(d.maxU >= 7.0)
    }

    @Test
    fun cloud_in_another_frame_is_dropped() {
        val d = buildMissionMap(
            kmz(wp(0, 0.0, 0.0, 0.0)),
            PlyParser.XyzCloud(floatArrayOf(5000f, 5000f, 0f, 5001f, 5000f, 0f)),
        )
        assertEquals(0, d.cloudPointCount)
        assertTrue(d.maxE < 100)
    }
}
