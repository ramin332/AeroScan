package com.aeroscan.rccompanion.cloud

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

class PlyVoxelDownsampleTest {

    @Test
    fun `binary little-endian PLY round-trips through reader`() {
        val src = floatArrayOf(
            0f, 0f, 0f,
            1f, 2f, 3f,
            -10f, 5.5f, 100f,
        )
        val cloud = PlyParser.XyzCloud(src)
        val bytes = PlyParser.writeBinary(cloud)
        val parsed = PlyParser.read(bytes)
        assertNotNull(parsed)
        assertEquals(3, parsed!!.pointCount)
        assertEquals(src[0], parsed.xyz[0], 0f)
        assertEquals(src[8], parsed.xyz[8], 0f)
    }

    @Test
    fun `voxel-downsample collapses points within one cell`() {
        // 3 points all in the same 1m cell + 2 points in distinct cells
        val xyz = floatArrayOf(
            0.1f, 0.2f, 0.3f,        // cell (0,0,0)
            0.5f, 0.5f, 0.5f,        // cell (0,0,0)
            0.9f, 0.0f, 0.99f,       // cell (0,0,0)
            5.5f, 5.5f, 5.5f,        // cell (5,5,5)
            -3.3f, 7.0f, 2.0f,       // cell (-4,7,2)
        )
        val out = PlyVoxelDownsample.downsample(PlyParser.XyzCloud(xyz), 1.0)
        assertEquals(3, out.pointCount)  // 3 distinct cells
    }

    @Test
    fun `voxel size scales density predictably`() {
        // Grid of 100 points in a 10×10 cm-sized region; voxel-downsample
        // at 5 cm should leave ~4 points (one per 5cm subcell).
        val side = 10
        val xyz = FloatArray(side * side * 3)
        var idx = 0
        for (i in 0 until side) {
            for (j in 0 until side) {
                xyz[idx++] = i * 0.01f
                xyz[idx++] = j * 0.01f
                xyz[idx++] = 0f
            }
        }
        val cloud = PlyParser.XyzCloud(xyz)
        val downsampled = PlyVoxelDownsample.downsample(cloud, 0.05)
        // 10 cm × 10 cm region / 5 cm voxel = 2 cells per axis = 4 cells
        // → 4 output points (with one Z slice).
        assertEquals(4, downsampled.pointCount)
    }

    @Test
    fun `voxel-downsample then write-binary then read round-trip`() {
        // 1m voxel: stamp 100 points scattered across 5m
        val xyz = FloatArray(100 * 3)
        for (i in 0 until 100) {
            xyz[i * 3]     = (i % 5).toFloat() + 0.4f
            xyz[i * 3 + 1] = ((i / 5) % 4).toFloat() + 0.5f
            xyz[i * 3 + 2] = 0f
        }
        val downsampled = PlyVoxelDownsample.downsample(PlyParser.XyzCloud(xyz), 1.0)
        val plyBytes = PlyParser.writeBinary(downsampled)
        val reread = PlyParser.read(plyBytes)
        assertNotNull(reread)
        assertEquals(downsampled.pointCount, reread!!.pointCount)
        // Output PLY should be a valid header.
        val text = String(plyBytes.copyOfRange(0, 80), Charsets.US_ASCII)
        assertTrue(text.startsWith("ply"))
        assertTrue(text.contains("binary_little_endian"))
        assertTrue(text.contains("element vertex ${downsampled.pointCount}"))
    }
}
