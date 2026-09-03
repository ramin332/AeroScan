package com.aeroscan.rccompanion.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.PointMode
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

enum class MapLayer { Original, Augmented, Both }
enum class MapView { Top, Orbit }

/**
 * Top-down view of what the pilot is about to upload: cloud footprint, the
 * facades the Manifold found (red = no waypoint photographs it), the DJI
 * mission polygon, the flight path, and one heading tick per waypoint — grey
 * for what the KMZ said, pitch-coloured for what the augmenter re-aimed it to
 * (light = level at a wall, dark = steeply down). Red rings mark waypoints the
 * summary flagged. Pure Canvas, works offline.
 */
@Composable
fun MissionMap(
    data: MissionMapData?,
    modifier: Modifier = Modifier,
    heightDp: Int = 320,
    layer: MapLayer = MapLayer.Both,
) {
    val cs = MaterialTheme.colorScheme
    val showOrig = layer != MapLayer.Augmented || data?.headingsAugmented == null
    val showAug = layer != MapLayer.Original

    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(heightDp.dp)
            .clip(RoundedCornerShape(8.dp))
            .background(cs.surfaceVariant.copy(alpha = 0.5f)),
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
            val pad = 10.dp.toPx()
            val w = size.width - 2 * pad
            val h = size.height - 2 * pad
            val scale = min(w / data.widthM, h / data.heightM).toFloat()
            val offX = pad + (w - (data.widthM * scale).toFloat()) / 2f
            val offY = pad + (h - (data.heightM * scale).toFloat()) / 2f
            fun px(e: Double, n: Double) = Offset(
                offX + ((e - data.minE) * scale).toFloat(),
                size.height - offY - ((n - data.minN) * scale).toFloat(),
            )

            // 1. Cloud footprint.
            if (data.cloudPointCount > 0) {
                val pts = ArrayList<Offset>(data.cloudPointCount)
                for (i in 0 until data.cloudPointCount) {
                    pts.add(px(data.cloudXYZ[3 * i].toDouble(), data.cloudXYZ[3 * i + 1].toDouble()))
                }
                drawPoints(pts, PointMode.Points, MissionPalette.cloud.copy(alpha = 0.45f), strokeWidth = 2.dp.toPx())
            }

            // 2. Facades — footprint of every wall the Manifold found.
            for (f in data.facades) {
                val path = Path()
                for (c in 0 until f.cornerCount) {
                    val o = px(f.v[3 * c], f.v[3 * c + 1])
                    if (c == 0) path.moveTo(o.x, o.y) else path.lineTo(o.x, o.y)
                }
                path.close()
                val col = if (f.covered) MissionPalette.facadeCovered else MissionPalette.facadeUncovered
                drawPath(path, col.copy(alpha = if (f.covered) 0.18f else 0.35f))
                drawPath(path, col.copy(alpha = 0.8f), style = Stroke(width = 1.dp.toPx()))
            }

            // 3. Mission polygon (dashed).
            if (data.polygonXY.size >= 6) {
                val p = Path()
                for (i in 0 until data.polygonXY.size / 2) {
                    val o = px(data.polygonXY[2 * i], data.polygonXY[2 * i + 1])
                    if (i == 0) p.moveTo(o.x, o.y) else p.lineTo(o.x, o.y)
                }
                p.close()
                drawPath(
                    p, cs.outline,
                    style = Stroke(
                        width = 1.5.dp.toPx(),
                        pathEffect = PathEffect.dashPathEffect(floatArrayOf(8f, 6f)),
                    ),
                )
            }

            // 4. Flight path.
            val path = Path()
            for (i in 0 until data.waypointCount) {
                val o = px(data.wpE(i), data.wpN(i))
                if (i == 0) path.moveTo(o.x, o.y) else path.lineTo(o.x, o.y)
            }
            drawPath(path, MissionPalette.path.copy(alpha = 0.7f), style = Stroke(width = 1.dp.toPx()))

            // 5. Heading ticks. Heading is clockwise from north: east = sin, north = cos.
            val tick = maxOf(6.dp.toPx(), (2.5 * scale).toFloat())
            fun tickEnd(o: Offset, headingDeg: Double, len: Float): Offset {
                val r = headingDeg * PI / 180.0
                return Offset(o.x + (sin(r) * len).toFloat(), o.y - (cos(r) * len).toFloat())
            }
            for (i in 0 until data.waypointCount) {
                val o = px(data.wpE(i), data.wpN(i))
                if (showOrig) {
                    // Smart3D cycles a fan of poses around the waypoint heading,
                    // so draw the fan, not one ray that never gets shot alone.
                    val poses = data.posesOriginal[i]
                    if (poses.isNotEmpty()) {
                        for (k in 0 until poses.size / 2) {
                            val shorten = cos(poses[2 * k] * PI / 180.0).toFloat().coerceIn(0.15f, 1f)
                            drawLine(
                                MissionPalette.aimOriginal.copy(alpha = 0.7f), o,
                                tickEnd(o, data.headingsOriginal[i] + poses[2 * k + 1], tick * shorten),
                                strokeWidth = 1.dp.toPx(),
                            )
                        }
                    } else {
                        drawLine(MissionPalette.aimOriginal, o, tickEnd(o, data.headingsOriginal[i], tick),
                            strokeWidth = 1.5.dp.toPx())
                    }
                }
                val hs = data.headingsAugmented
                val ps = data.pitchesAugmented
                if (showAug && hs != null && ps != null) {
                    // Length shrinks with pitch: a tick aimed steeply down covers
                    // little ground, so a short tick is the honest picture.
                    val foreshorten = cos(ps[i] * PI / 180.0).toFloat().coerceIn(0.15f, 1f)
                    drawLine(MissionPalette.pitchColor(ps[i]), o, tickEnd(o, hs[i], tick * 1.3f * foreshorten),
                        strokeWidth = 2.dp.toPx())
                }
            }

            // 6. Waypoint dots + flags on top.
            for (i in 0 until data.waypointCount) {
                drawCircle(MissionPalette.path, radius = 1.5.dp.toPx(), center = px(data.wpE(i), data.wpN(i)))
            }
            for (i in data.flagged) {
                drawCircle(MissionPalette.flag, radius = 5.dp.toPx(), center = px(data.wpE(i), data.wpN(i)),
                    style = Stroke(width = 1.5.dp.toPx()))
            }

            // 7. Start marker + 10 m scale bar.
            drawCircle(MissionPalette.start, radius = 4.dp.toPx(), center = px(data.wpE(0), data.wpN(0)))
            val y = size.height - pad
            drawLine(cs.onSurface, Offset(pad, y), Offset(pad + (10.0 * scale).toFloat(), y),
                strokeWidth = 2.dp.toPx())
        }
        Text(
            "10 m",
            style = MaterialTheme.typography.labelSmall,
            color = cs.onSurface,
            modifier = Modifier.align(Alignment.BottomStart).padding(start = 12.dp, bottom = 12.dp),
        )
        Text(
            "N ↑",
            style = MaterialTheme.typography.labelSmall,
            color = cs.onSurface,
            modifier = Modifier.align(Alignment.TopEnd).padding(8.dp),
        )
    }
}

@Composable
fun MissionLegend(data: MissionMapData?, modifier: Modifier = Modifier) {
    Row(
        modifier = modifier,
        horizontalArrangement = Arrangement.spacedBy(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        LegendItem(MissionPalette.cloud, "scan")
        LegendItem(
            MissionPalette.aimOriginal,
            if (data != null && data.rosetteWaypoints > 0) "DJI rosette" else "DJI aim",
        )
        LegendItem(MissionPalette.aimAugmentedLevel, "at wall")
        LegendItem(MissionPalette.aimAugmentedSteep, "steep down")
        if (data != null && data.facades.isNotEmpty()) {
            LegendItem(MissionPalette.facadeUncovered, "unshot walls: ${data.uncoveredFacades}")
        }
        LegendItem(MissionPalette.flag, "check")
    }
}

@Composable
private fun LegendItem(color: Color, label: String) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(Modifier.size(10.dp).clip(CircleShape).background(color))
        Spacer(Modifier.width(4.dp))
        Text(label, style = MaterialTheme.typography.labelSmall)
    }
}
