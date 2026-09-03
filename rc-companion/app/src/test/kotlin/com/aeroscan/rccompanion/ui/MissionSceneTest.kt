package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.wpml.ImportedKmz
import com.aeroscan.rccompanion.wpml.ParsedWaypoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** The 3D scene's maths: aim direction, ray length, orbit camera, projection. */
class MissionSceneTest {
    private val m = metersPerDeg(52.0)

    private fun wp(i: Int, dE: Double, dN: Double, heading: Double, pitch: Double, dU: Double = 0.0) =
        ParsedWaypoint(
            index = i, lon = 5.0 + dE / m[1], lat = 52.0 + dN / m[0], altEgm96 = 30.0 + dU,
            headingDeg = heading, gimbalPitchDeg = pitch,
        )

    private fun scene(
        facades: List<FacadeQuad> = emptyList(),
        targets: IntArray = IntArray(0),
        vararg wps: ParsedWaypoint,
    ) = buildMissionMap(
        ImportedKmz("t", 52.0, 5.0, 30.0, wps.toList(), emptyList()),
        null, null, emptySet(), facades, targets,
    )

    @Test
    fun aim_direction_follows_heading_clockwise_from_north_and_pitch_down() {
        val north = aimDirection(0.0, 0.0)
        assertEquals(0.0, north[0], 1e-9); assertEquals(1.0, north[1], 1e-9); assertEquals(0.0, north[2], 1e-9)
        val east = aimDirection(90.0, 0.0)
        assertEquals(1.0, east[0], 1e-9); assertEquals(0.0, east[1], 1e-9)
        val nadir = aimDirection(0.0, -90.0)
        assertEquals(-1.0, nadir[2], 1e-9)
        assertEquals(0.0, nadir[1], 1e-9)
        // Looking up stays positive in Z, and never exceeds the gimbal's +35°.
        assertTrue(aimDirection(0.0, 30.0)[2] > 0)
    }

    private fun wallAt(n: Double) = FacadeQuad(
        v = doubleArrayOf(-2.0, n, 0.0, 2.0, n, 0.0, 2.0, n, 4.0, -2.0, n, 4.0),
        n = doubleArrayOf(0.0, -1.0, 0.0), waypoints = 0,
    )

    @Test
    fun ray_length_reaches_the_target_facade_and_falls_back_without_one() {
        val d = scene(listOf(wallAt(10.0)), intArrayOf(0), wp(0, 0.0, 0.0, 0.0, 0.0, dU = 2.0))
        // Facade centre is 10 m north, 2 m up → distance from the WP at (0,0,2).
        assertEquals(10.0, aimRayLength(d, 0, 99.0), 0.01)
        val none = scene(wps = arrayOf(wp(0, 0.0, 0.0, 0.0, 0.0)))
        assertEquals(7.5, aimRayLength(none, 0, 7.5), 1e-9)
    }

    @Test
    fun aim_error_is_zero_when_pointed_at_the_target_and_large_when_not() {
        val d = scene(listOf(wallAt(10.0)), intArrayOf(0), wp(0, 0.0, 0.0, 0.0, 0.0, dU = 2.0))
        val h = d.headingsOriginal; val p = d.pitchesOriginal
        assertEquals(0.0, aimErrorDeg(d, 0, h, p)!!, 0.5)
        val off = scene(listOf(wallAt(10.0)), intArrayOf(0), wp(0, 0.0, 0.0, 90.0, 0.0, dU = 2.0))
        assertEquals(90.0, aimErrorDeg(off, 0, off.headingsOriginal, off.pitchesOriginal)!!, 0.5)
        // 2026-07-10: the gimbal sat at the ±60° pan stop while the target was
        // elsewhere — an error this size is invisible on the FPV feed.
        val pinned = scene(listOf(wallAt(10.0)), intArrayOf(0), wp(0, 0.0, 0.0, 60.0, 0.0, dU = 2.0))
        assertTrue(aimErrorDeg(pinned, 0, pinned.headingsOriginal, pinned.pitchesOriginal)!! > 45.0)
        assertNull(aimErrorDeg(scene(wps = arrayOf(wp(0, 0.0, 0.0, 0.0, 0.0))), 0, doubleArrayOf(0.0), doubleArrayOf(0.0)))
    }

    @Test
    fun orbit_camera_clamps_elevation_and_zoom_and_wraps_azimuth() {
        val c = OrbitCamera(azimuthDeg = 350.0, elevationDeg = 80.0, zoom = 1.0)
        val up = c.withDelta(20.0, 40.0, 10.0)
        assertEquals(10.0, up.azimuthDeg, 1e-9)
        assertEquals(OrbitCamera.MAX_ELEVATION_DEG, up.elevationDeg, 1e-9)
        assertEquals(OrbitCamera.MAX_ZOOM, up.zoom, 1e-9)
        val down = c.withDelta(-360.0, -200.0, 0.001)
        assertEquals(350.0, down.azimuthDeg, 1e-9)
        assertEquals(OrbitCamera.MIN_ELEVATION_DEG, down.elevationDeg, 1e-9)
        assertEquals(OrbitCamera.MIN_ZOOM, down.zoom, 1e-9)
    }

    @Test
    fun projection_puts_the_scene_centre_mid_screen_and_orders_depth() {
        val d = scene(wps = arrayOf(wp(0, -10.0, 0.0, 0.0, 0.0), wp(1, 10.0, 0.0, 0.0, 0.0)))
        val p = SceneProjector(d, OrbitCamera(azimuthDeg = 0.0, elevationDeg = 20.0, zoom = 1.0), 400f, 300f)
        val centre = p.project((d.minE + d.maxE) / 2, (d.minN + d.maxN) / 2, (d.minU + d.maxU) / 2)
        assertEquals(200f, centre.x, 0.5f)
        assertEquals(150f, centre.y, 0.5f)
        assertTrue(centre.depth > 0)
        // Azimuth 0 = eye on the south side, so a point further north is deeper.
        val near = p.project(0.0, -5.0, 0.0)
        val far = p.project(0.0, 5.0, 0.0)
        assertTrue("near ${near.depth} far ${far.depth}", far.depth > near.depth)
    }

    @Test
    fun dragging_right_orbits_the_eye_to_the_right() {
        // 2026-09-03, pilot on the RC: "rotating is counter intuitive in 3D
        // (right is left)". The shipped mapping moved the scene with the finger
        // (Google-Earth style); the pilot expects the eye to move with the
        // finger instead, so a rightward drag now raises the azimuth and the
        // scene swings the other way. One constant (the sign in withDrag)
        // switches it back if that reads wrong in the air.
        val c = OrbitCamera(azimuthDeg = 0.0, elevationDeg = 20.0, zoom = 1.0)
        val d = scene(wps = arrayOf(wp(0, 0.0, 10.0, 0.0, 0.0), wp(1, 0.0, -10.0, 0.0, 0.0)))
        val dragged = c.withDrag(panX = 40f, panY = 0f, zoomFactor = 1f)
        assertEquals(10.0, dragged.azimuthDeg, 1e-9)
        val before = SceneProjector(d, c, 400f, 300f).project(10.0, 0.0, 0.0)
        val after = SceneProjector(d, dragged, 400f, 300f).project(10.0, 0.0, 0.0)
        assertTrue("scene should swing left, was ${before.x} now ${after.x}", after.x < before.x)
    }

    @Test
    fun dragging_down_looks_further_down_on_the_site() {
        val c = OrbitCamera(azimuthDeg = 0.0, elevationDeg = 20.0, zoom = 1.0)
        assertTrue(c.withDrag(0f, 40f, 1f).elevationDeg > c.elevationDeg)
        assertTrue(c.withDrag(0f, -40f, 1f).elevationDeg < c.elevationDeg)
    }

    @Test
    fun points_behind_the_eye_report_non_positive_depth() {
        val d = scene(wps = arrayOf(wp(0, 0.0, 0.0, 0.0, 0.0)))
        val p = SceneProjector(d, OrbitCamera(azimuthDeg = 0.0, elevationDeg = 10.0, zoom = 1.0), 400f, 300f)
        assertTrue(p.project(0.0, -10_000.0, 0.0).depth <= 0.1)
    }
}
