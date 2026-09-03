package com.aeroscan.rccompanion.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp

/**
 * Orbit view of the mission: the scanned cloud, the facades the Manifold found,
 * the flight path, and one camera ray per waypoint. This is the view that
 * answers "is the camera actually pointed at the wall" — the 2D map can only
 * show bearing, and a bearing that looks right can still be aimed at the ground.
 *
 * Drag rotates, pinch zooms. All drawing is plain Canvas; no GL, works offline.
 */
@Composable
fun MissionScene3D(
    data: MissionMapData?,
    modifier: Modifier = Modifier,
    heightDp: Int = 280,
    layer: MapLayer = MapLayer.Both,
    showRecognised: Boolean = true,
) {
    val cs = MaterialTheme.colorScheme
    var camera by remember { mutableStateOf(OrbitCamera()) }
    val showOrig = layer != MapLayer.Augmented || data?.headingsAugmented == null
    val showAug = layer != MapLayer.Original

    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(heightDp.dp)
            .clip(RoundedCornerShape(8.dp))
            .background(cs.surfaceVariant.copy(alpha = 0.5f))
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    camera = camera.withDrag(pan.x, pan.y, zoom)
                }
            },
        contentAlignment = Alignment.Center,
    ) {
        if (data == null || data.waypointCount == 0) {
            Text(
                "Pick a KMZ to see the mission",
                style = MaterialTheme.typography.bodyMedium,
                color = cs.onSurfaceVariant,
            )
            return@Box
        }
        Canvas(modifier = Modifier.fillMaxSize()) {
            val p = SceneProjector(data, camera, size.width, size.height)
            // With no facade targets yet (before the augment) the rays need a
            // length of their own; too short and they vanish into the waypoint dots.
            val fallbackRay = data.sceneSpanM * 0.12

            // 1. Cloud — the scanned surface, drawn first so everything sits on it.
            if (data.cloudPointCount > 0) {
                val dot = 1.4.dp.toPx()
                for (i in 0 until data.cloudPointCount) {
                    val q = p.project(
                        data.cloudXYZ[3 * i].toDouble(),
                        data.cloudXYZ[3 * i + 1].toDouble(),
                        data.cloudXYZ[3 * i + 2].toDouble(),
                    )
                    if (q.depth > 0.1) drawCircle(MissionPalette.cloud.copy(alpha = 0.5f), dot, Offset(q.x, q.y))
                }
            }

            // 2. The points the extractor actually recognised, coloured by the
            //    facet that claimed them. This is what "recognised" means: the
            //    grey cloud is everything scanned, these are the parts that
            //    became facades.
            if (showRecognised) {
                val dot = 2.2.dp.toPx()
                data.facades.forEachIndexed { fi, f ->
                    val col = MissionPalette.facadeColor(fi)
                    for (i in 0 until f.sampleCount) {
                        val q = p.project(f.sample[3 * i], f.sample[3 * i + 1], f.sample[3 * i + 2])
                        if (q.depth > 0.1) drawCircle(col, dot, Offset(q.x, q.y))
                    }
                }
            }

            // 3. Facades, painter's algorithm (far first) so near walls occlude.
            if (data.facades.isNotEmpty()) {
                val order = data.facades.indices.sortedByDescending { i ->
                    p.project(data.facades[i].cx(), data.facades[i].cy(), data.facades[i].cz()).depth
                }
                for (i in order) {
                    val f = data.facades[i]
                    val path = Path()
                    var visible = true
                    for (c in 0 until f.cornerCount) {
                        val q = p.project(f.v[3 * c], f.v[3 * c + 1], f.v[3 * c + 2])
                        if (q.depth <= 0.1) { visible = false; break }
                        if (c == 0) path.moveTo(q.x, q.y) else path.lineTo(q.x, q.y)
                    }
                    if (!visible) continue
                    path.close()
                    val col = if (f.covered) MissionPalette.facadeCovered else MissionPalette.facadeUncovered
                    drawPath(path, col.copy(alpha = if (f.covered) 0.22f else 0.38f))
                    drawPath(path, col.copy(alpha = 0.9f), style = Stroke(width = 1.dp.toPx()))
                }
            }

            // 4. Flight path.
            val line = Path()
            var started = false
            for (i in 0 until data.waypointCount) {
                val q = p.project(data.wpE(i), data.wpN(i), data.wpU(i))
                if (q.depth <= 0.1) { started = false; continue }
                if (!started) { line.moveTo(q.x, q.y); started = true } else line.lineTo(q.x, q.y)
            }
            drawPath(line, MissionPalette.path.copy(alpha = 0.75f), style = Stroke(width = 1.dp.toPx()))

            // 5. Camera rays — the point of this view.
            for (i in 0 until data.waypointCount) {
                val a = p.project(data.wpE(i), data.wpN(i), data.wpU(i))
                if (a.depth <= 0.1) continue
                val len = aimRayLength(data, i, fallbackRay)
                if (showOrig) {
                    // Smart3D does not shoot one frame per waypoint — it cycles a
                    // 3–5 pose rosette (±30° yaw, tens of degrees of pitch) while
                    // flying. Draw every pose; a single ray would be a fiction.
                    val poses = data.posesOriginal[i]
                    val rays = if (poses.isNotEmpty()) {
                        (0 until poses.size / 2).map {
                            aimDirection(data.headingsOriginal[i] + poses[2 * it + 1], poses[2 * it])
                        }
                    } else {
                        listOf(aimDirection(data.headingsOriginal[i], data.pitchesOriginal[i]))
                    }
                    for (d in rays) {
                        val b = p.project(
                            data.wpE(i) + d[0] * len, data.wpN(i) + d[1] * len, data.wpU(i) + d[2] * len,
                        )
                        if (b.depth > 0.1) {
                            drawLine(MissionPalette.aimOriginal.copy(alpha = 0.55f), Offset(a.x, a.y), Offset(b.x, b.y),
                                strokeWidth = 1.dp.toPx())
                        }
                    }
                }
                if (showAug) {
                    val hs = data.headingsAugmented
                    val ps = data.pitchesAugmented
                    if (hs != null && ps != null) {
                        val d = aimDirection(hs[i], ps[i])
                        val b = p.project(
                            data.wpE(i) + d[0] * len, data.wpN(i) + d[1] * len, data.wpU(i) + d[2] * len,
                        )
                        if (b.depth > 0.1) {
                            drawLine(MissionPalette.pitchColor(ps[i]), Offset(a.x, a.y), Offset(b.x, b.y),
                                strokeWidth = 1.5.dp.toPx())
                        }
                    }
                }
            }

            // 6. Waypoints, flags, start.
            for (i in 0 until data.waypointCount) {
                val q = p.project(data.wpE(i), data.wpN(i), data.wpU(i))
                if (q.depth <= 0.1) continue
                drawCircle(MissionPalette.path, 1.dp.toPx(), Offset(q.x, q.y))
                if (i in data.flagged) {
                    drawCircle(MissionPalette.flag, 4.5.dp.toPx(), Offset(q.x, q.y), style = Stroke(width = 1.5.dp.toPx()))
                }
            }
            val s = p.project(data.wpE(0), data.wpN(0), data.wpU(0))
            if (s.depth > 0.1) drawCircle(MissionPalette.start, 4.dp.toPx(), Offset(s.x, s.y))
        }
        Text(
            "drag to rotate · pinch to zoom",
            style = MaterialTheme.typography.labelSmall,
            color = cs.onSurfaceVariant,
            modifier = Modifier.align(Alignment.BottomStart).padding(8.dp),
        )
        if (data.facades.isNotEmpty()) {
            Text(
                "${data.facades.size - data.uncoveredFacades}/${data.facades.size} facades covered · " +
                    "${data.recognisedPoints} points recognised",
                style = MaterialTheme.typography.labelSmall,
                color = cs.onSurfaceVariant,
                modifier = Modifier.align(Alignment.TopEnd).padding(8.dp),
            )
        }
    }
}
