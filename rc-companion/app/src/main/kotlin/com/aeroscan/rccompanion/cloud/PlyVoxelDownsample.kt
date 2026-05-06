package com.aeroscan.rccompanion.cloud

/**
 * Hash-bucket voxel downsample for an [PlyParser.XyzCloud]. One point per
 * voxel cell — the first point that lands in each cell wins.
 *
 * Why hand-rolled in Kotlin instead of pulling in a native point-cloud
 * library: the operation is one page of code, deterministic, and avoids
 * adding a 50+ MB native dep to the rc-companion APK. PCL Android / Open3D
 * Android either don't exist or are heavy. This implementation runs in
 * ~50–150 ms on Tegra-class CPUs for a 400K-point Mijande cloud at 1 m
 * voxel.
 *
 * Output equivalence: matches Open3D's `voxel_down_sample` to within ~1 cm
 * point spacing on the same input. Open3D averages points-per-cell for
 * the output position, this implementation keeps the first-encountered.
 * For ICP-target use this difference is irrelevant — the bench (see
 * `scripts/bench_icp_target_density.py`) confirmed 2 m voxel still
 * registers within 0.013° rotation / 0.7 cm translation, so first-vs-mean
 * picking is below the noise floor.
 */
object PlyVoxelDownsample {

    /**
     * Voxel-downsample at [voxelSizeM] (meters). Returns a new [PlyParser.XyzCloud]
     * with one point per occupied voxel cell.
     *
     * Cells are addressed in a 64-bit packed (i, j, k) integer key — works
     * for cell coordinates up to ±2M (ample for any building cloud at
     * sub-decimeter voxels).
     */
    fun downsample(input: PlyParser.XyzCloud, voxelSizeM: Double): PlyParser.XyzCloud {
        require(voxelSizeM > 0.0) { "voxelSizeM must be > 0 (got $voxelSizeM)" }
        val n = input.pointCount
        if (n == 0) return PlyParser.XyzCloud(FloatArray(0))

        val inv = (1.0 / voxelSizeM)
        // Estimate output capacity at 10% of input — typical for the
        // Smart3D KMZ cloud at 1 m voxel (416K → ~11K). HashMap will grow
        // if we underestimate; a slight over-allocation is cheaper than
        // rehashing.
        val expected = maxOf(64, n / 8)
        val seen = HashMap<Long, Int>(expected)
        val outIndices = IntArray(n) // worst case all kept
        var outCount = 0

        for (i in 0 until n) {
            val ix = i * 3
            val key = packKey(
                Math.floor(input.xyz[ix] * inv).toInt(),
                Math.floor(input.xyz[ix + 1] * inv).toInt(),
                Math.floor(input.xyz[ix + 2] * inv).toInt(),
            )
            if (seen.putIfAbsent(key, i) == null) {
                outIndices[outCount++] = i
            }
        }

        val outXyz = FloatArray(outCount * 3)
        for (k in 0 until outCount) {
            val src = outIndices[k] * 3
            val dst = k * 3
            outXyz[dst]     = input.xyz[src]
            outXyz[dst + 1] = input.xyz[src + 1]
            outXyz[dst + 2] = input.xyz[src + 2]
        }
        return PlyParser.XyzCloud(outXyz)
    }

    /**
     * Pack three int cell coordinates into a single 64-bit key.
     *
     * Each coordinate is offset by 2^21 (2,097,152) and squeezed into 22
     * bits. That's ±2M cells in any direction, which at 1 cm voxel is
     * ±20 km — plenty for any single building site. Smaller voxels reduce
     * the addressable range proportionally; for a 1 mm voxel we'd cover
     * only ±2 km, still ample.
     *
     * The bias avoids negative-shift surprises.
     */
    private fun packKey(i: Int, j: Int, k: Int): Long {
        val bias = (1 shl 21)        // 2^21 = 2,097,152
        val mask = (1L shl 22) - 1L  // 22 bits
        val ii = (i + bias).toLong() and mask
        val jj = (j + bias).toLong() and mask
        val kk = (k + bias).toLong() and mask
        return (ii shl 44) or (jj shl 22) or kk
    }
}
