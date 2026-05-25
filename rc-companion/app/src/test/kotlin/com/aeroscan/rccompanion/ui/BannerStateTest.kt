package com.aeroscan.rccompanion.ui

import com.aeroscan.rccompanion.mop.AugmentFraming.ManifoldStatus
import com.aeroscan.rccompanion.mop.StatusSession
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Exercises the pure [bannerFor] mapping — the priority rule (EnvError outranks
 * NoMesh outranks Ready) and the Unreachable pass-through. No MSDK needed.
 */
class BannerStateTest {

    private fun status(mesh: Boolean, env: Boolean) = ManifoldStatus(
        appVersion = "0.4.0",
        flightId = "the_latest_flight",
        latestFlight = "flight0048",
        meshPresent = mesh,
        meshChunks = if (mesh) 25 else 0,
        nPoints = if (mesh) 1_200_000L else 0L,
        meshBytes = if (mesh) 1_048_576L else 0L,
        blackboxFreeGb = 12.4,
        envOk = env,
        envDetail = if (env) "ok" else "import failed (exit 1)",
    )

    @Test
    fun envError_outranks_noMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = false, env = false)))
        assertTrue(b is BannerState.EnvError)
    }

    @Test
    fun noMesh_whenEnvOkButNoMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = false, env = true)))
        assertTrue(b is BannerState.NoMesh)
    }

    @Test
    fun ready_whenEnvOkAndMesh() {
        val b = bannerFor(StatusSession.Result.Ok(status(mesh = true, env = true)))
        assertTrue(b is BannerState.Ready)
    }

    @Test
    fun unreachable_passesThrough() {
        val b = bannerFor(StatusSession.Result.Unreachable("connect: x"))
        assertTrue(b is BannerState.Unreachable)
    }
}
