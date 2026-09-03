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

/**
 * Top-down view of what the pilot is about to upload: cloud footprint, the
 * DJI mission polygon, the waypoint path, and one heading tick per waypoint —
 * grey for what the KMZ said, blue for what the augmenter re-aimed it to.
 * Red rings mark waypoints the summary flagged. Pure Canvas, works offline.
 */
enum class MapLayer { Original, Augmented, Both }

@Composable
fun MissionMap(
    data: MissionMapData?,
    modifier: Modifier = Modifier,
    heightDp: Int = 320,
    layer: MapLayer = MapLayer.Both,
) {
    val showOrig = layer != MapLayer.Augmented || data?.headingsAugmented == null
    val showAug = layer != MapLayer.Original
    val cs = MaterialTheme.colorScheme
    val cloudColor = cs.onSurface.copy(alpha = 0.28f)
    val polyColor = cs.outline
    val pathColor = cs.onSurfaceVariant.copy(alpha = 0.7f)
    val origColor = cs.onSurface.copy(alpha = 0.35f)
    val augColor = cs.primary
    val flagColor = cs.error

    Column(modifier = modifier) {
        Box(
            modifier = Modifier
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
            } else {
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
                            pts.add(px(data.cloudXY[2 * i].toDouble(), data.cloudXY[2 * i + 1].toDouble()))
                        }
                        drawPoints(pts, PointMode.Points, cloudColor, strokeWidth = 2.dp.toPx())
                    }

                    // 2. Mission polygon (dashed).
                    if (data.polygonXY.size >= 6) {
                        val p = Path()
                        for (i in 0 until data.polygonXY.size / 2) {
                            val o = px(data.polygonXY[2 * i], data.polygonXY[2 * i + 1])
                            if (i == 0) p.moveTo(o.x, o.y) else p.lineTo(o.x, o.y)
                        }
                        p.close()
                        drawPath(
                            p, polyColor,
                            style = Stroke(
                                width = 1.5.dp.toPx(),
                                pathEffect = PathEffect.dashPathEffect(floatArrayOf(8f, 6f)),
                            ),
                        )
                    }

                    // 3. Flight path.
                    val path = Path()
                    for (i in 0 until data.waypointCount) {
                        val o = px(data.pathXY[2 * i], data.pathXY[2 * i + 1])
                        if (i == 0) path.moveTo(o.x, o.y) else path.lineTo(o.x, o.y)
                    }
                    drawPath(path, pathColor, style = Stroke(width = 1.dp.toPx()))

                    // 4. Heading ticks. Heading is clockwise from north: east = sin, north = cos.
                    val tick = maxOf(6.dp.toPx(), (2.5 * scale).toFloat())
                    fun tickEnd(o: Offset, headingDeg: Double, len: Float): Offset {
                        val r = headingDeg * PI / 180.0
                        return Offset(o.x + (sin(r) * len).toFloat(), o.y - (cos(r) * len).toFloat())
                    }
                    for (i in 0 until data.waypointCount) {
                        val o = px(data.pathXY[2 * i], data.pathXY[2 * i + 1])
                        if (showOrig) {
                            drawLine(origColor, o, tickEnd(o, data.headingsOriginal[i], tick), strokeWidth = 1.5.dp.toPx())
                        }
                        if (showAug) data.headingsAugmented?.let {
                            drawLine(augColor, o, tickEnd(o, it[i], tick * 1.3f), strokeWidth = 2.dp.toPx())
                        }
                    }

                    // 5. Waypoint dots + flags on top.
                    for (i in 0 until data.waypointCount) {
                        val o = px(data.pathXY[2 * i], data.pathXY[2 * i + 1])
                        drawCircle(pathColor, radius = 1.5.dp.toPx(), center = o)
                    }
                    for (i in data.flagged) {
                        val o = px(data.pathXY[2 * i], data.pathXY[2 * i + 1])
                        drawCircle(flagColor, radius = 5.dp.toPx(), center = o, style = Stroke(width = 1.5.dp.toPx()))
                    }

                    // 6. Start marker.
                    val s = px(data.pathXY[0], data.pathXY[1])
                    drawCircle(augColor, radius = 4.dp.toPx(), center = s)

                    // 7. Scale bar: 10 m.
                    val barLen = (10.0 * scale).toFloat()
                    val y = size.height - pad
                    drawLine(cs.onSurface, Offset(pad, y), Offset(pad + barLen, y), strokeWidth = 2.dp.toPx())
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
        Spacer(Modifier.height(6.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(14.dp), verticalAlignment = Alignment.CenterVertically) {
            LegendItem(cloudColor, "scan")
            LegendItem(pathColor, "path")
            LegendItem(origColor, "DJI heading")
            LegendItem(augColor, "AeroScan heading")
            LegendItem(flagColor, "check")
        }
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
