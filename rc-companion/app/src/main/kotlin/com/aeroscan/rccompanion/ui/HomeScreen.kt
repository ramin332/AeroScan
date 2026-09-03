package com.aeroscan.rccompanion.ui

import android.content.Intent
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
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.filepick.rememberKmzPicker

/**
 * Control panel, top to bottom: status strip (4 chips), stepper, then a row
 * with the mission map on the left and stat tiles + the one action that
 * matters right now on the right. Prose is limited to a single line under
 * the action button; everything else is a number, a chip, or the map.
 */
@Composable
fun HomeScreen(viewModel: HomeViewModel = viewModel()) {
    val ui by viewModel.ui.collectAsStateWithLifecycle()
    val conn by viewModel.connection.collectAsStateWithLifecycle()
    val banner by viewModel.banner.collectAsStateWithLifecycle()
    val status by viewModel.status.collectAsStateWithLifecycle()
    val map by viewModel.map.collectAsStateWithLifecycle()
    val ctx = LocalContext.current
    val pick = rememberKmzPicker { viewModel.onFilePicked(it) }

    // Probe the Manifold when the aircraft link comes up; the ViewModel keeps
    // polling until it is Ready.
    LaunchedEffect(conn) {
        if (conn is Connection.State.AircraftConnected) viewModel.checkStatus()
    }

    val panel = panelStatusFor(conn, banner, status)
    val blockReason = augmentBlockReason(conn, banner, status)

    Scaffold { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(horizontal = 16.dp, vertical = 12.dp)
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("AeroScan", style = MaterialTheme.typography.titleLarge, fontWeight = FontWeight.SemiBold)
                Spacer(Modifier.width(24.dp))
                Stepper(current = stepOf(ui))
            }
            StatusStrip(
                status = panel,
                checking = banner is BannerState.Checking,
                onRefresh = viewModel::checkStatus,
            )
            Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                Column(modifier = Modifier.weight(1.45f)) {
                    MissionMap(data = map)
                }
                Column(modifier = Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                    StatTiles(ui = ui, map = map)
                    ActionPanel(
                        ui = ui,
                        blockReason = blockReason,
                        onPick = pick,
                        onAugment = viewModel::augment,
                        onCancel = viewModel::cancel,
                        onApprove = viewModel::approve,
                        onReject = viewModel::reject,
                        onOpenPilot2 = { openPilot2(ctx) },
                        onReset = viewModel::reset,
                    )
                }
            }
        }
    }
}

// ---------------------------------------------------------------- stepper

private val STEPS = listOf("Pick", "Augment", "Review", "Fly")

fun stepOf(ui: HomeViewModel.UiState): Int = when (ui) {
    HomeViewModel.UiState.Idle, is HomeViewModel.UiState.ParsingKmz, is HomeViewModel.UiState.Picked -> 0
    is HomeViewModel.UiState.Uploading, is HomeViewModel.UiState.AwaitingPreview -> 1
    is HomeViewModel.UiState.ReviewReady, is HomeViewModel.UiState.Approving -> 2
    is HomeViewModel.UiState.ReadyToFly -> 3
    is HomeViewModel.UiState.Error -> if (ui.file == null) 0 else 1
}

@Composable
private fun Stepper(current: Int) {
    val cs = MaterialTheme.colorScheme
    Row(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
        STEPS.forEachIndexed { i, name ->
            val active = i == current
            val done = i < current
            Text(
                "${i + 1} $name",
                style = MaterialTheme.typography.labelLarge,
                color = when {
                    active -> cs.onPrimaryContainer
                    done -> cs.onSurface
                    else -> cs.onSurfaceVariant
                },
                fontWeight = if (active) FontWeight.Bold else FontWeight.Normal,
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(if (active) cs.primaryContainer else cs.surface)
                    .padding(horizontal = 10.dp, vertical = 4.dp),
            )
            if (i < STEPS.lastIndex) Text("›", color = cs.onSurfaceVariant)
        }
    }
}

// ---------------------------------------------------------------- stat tiles

private data class Tile(val label: String, val value: String, val tone: Tone? = null)

private fun tilesFor(ui: HomeViewModel.UiState, map: MissionMapData?): List<Tile> {
    val s = when (ui) {
        is HomeViewModel.UiState.ReviewReady -> ui.summary
        is HomeViewModel.UiState.Approving -> ui.summary
        is HomeViewModel.UiState.ReadyToFly -> ui.summary
        else -> null
    }
    val wps = s?.waypointCount ?: map?.waypointCount
    val gsd = s?.gsdMedianMmPx?.let { "%.1f".format(it) }
    val gsdTone = if (s?.gsdMedianMmPx != null && s.gsdTargetMmPx != null)
        (if (s.gsdMedianMmPx <= s.gsdTargetMmPx * 1.25) Tone.Good else Tone.Warn) else null
    val aimed = if (s != null && s.waypointCount > 0) "${(100 * s.waypointsAimed / s.waypointCount)}%" else null
    val issues = s?.let { it.warnings + it.errors }
    return listOf(
        Tile("Waypoints", wps?.toString() ?: "—"),
        Tile("Facades", s?.facadeCount?.toString() ?: "—"),
        Tile("GSD mm/px", gsd ?: "—", gsdTone),
        Tile("Aimed", aimed ?: "—", s?.let { if (it.unaimed == 0) Tone.Good else Tone.Warn }),
        Tile("Warnings", issues?.toString() ?: "—", issues?.let { if (it == 0) Tone.Good else if (s.errors > 0) Tone.Bad else Tone.Warn }),
        Tile("Flips >90°", s?.flips?.toString() ?: "—", s?.let { if (it.flips == 0) Tone.Good else Tone.Warn }),
    )
}

@Composable
private fun StatTiles(ui: HomeViewModel.UiState, map: MissionMapData?) {
    val tiles = tilesFor(ui, map)
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        tiles.chunked(3).forEach { row ->
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                row.forEach { t -> StatTile(t, Modifier.weight(1f)) }
            }
        }
    }
}

@Composable
private fun StatTile(t: Tile, modifier: Modifier = Modifier) {
    val cs = MaterialTheme.colorScheme
    Column(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(cs.surfaceVariant.copy(alpha = 0.6f))
            .padding(horizontal = 10.dp, vertical = 8.dp),
    ) {
        Text(t.label.uppercase(), style = MaterialTheme.typography.labelSmall, color = cs.onSurfaceVariant)
        Text(
            t.value,
            style = MaterialTheme.typography.headlineSmall,
            color = t.tone?.let { toneColor(it) } ?: cs.onSurface,
            fontWeight = FontWeight.SemiBold,
        )
    }
}

// ---------------------------------------------------------------- action panel

@Composable
private fun ActionPanel(
    ui: HomeViewModel.UiState,
    blockReason: String?,
    onPick: () -> Unit,
    onAugment: () -> Unit,
    onCancel: () -> Unit,
    onApprove: () -> Unit,
    onReject: () -> Unit,
    onOpenPilot2: () -> Unit,
    onReset: () -> Unit,
) {
    val cs = MaterialTheme.colorScheme
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(cs.surfaceVariant.copy(alpha = 0.6f))
            .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        when (ui) {
            HomeViewModel.UiState.Idle -> {
                Primary("Pick KMZ", onPick)
                Hint("Smart3D mission exported from the RC.")
            }
            is HomeViewModel.UiState.ParsingKmz -> {
                FileLine(ui.file.displayName)
                LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                Hint("Reading waypoints and scan…")
            }
            is HomeViewModel.UiState.Picked -> {
                FileLine(ui.file.displayName)
                Primary("Augment on drone", onAugment, enabled = blockReason == null)
                Hint(blockReason ?: "Re-aims every waypoint at the scanned facades. ~4 min.")
                Secondary("Different file", onPick)
            }
            is HomeViewModel.UiState.Uploading -> {
                FileLine(ui.file.displayName)
                val pct = if (ui.total > 0) (ui.sent.toFloat() / ui.total).coerceIn(0f, 1f) else 0f
                LinearProgressIndicator(progress = { pct }, modifier = Modifier.fillMaxWidth())
                Hint("Uploading ${formatSize(ui.sent)} / ${formatSize(ui.total)}")
                Secondary("Cancel", onCancel)
            }
            is HomeViewModel.UiState.AwaitingPreview -> {
                FileLine(ui.file.displayName)
                LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                Hint("Augmenting on the drone — registering scan, extracting facades, re-aiming. ~4 min.")
                Secondary("Cancel", onCancel)
            }
            is HomeViewModel.UiState.ReviewReady -> {
                FileLine(ui.file.displayName)
                Primary("Approve & upload to aircraft", onApprove)
                val s = ui.summary
                val notes = buildList {
                    if (!s.stopAtWaypoint) add("fly-through mode (no stop per waypoint)")
                    if (s.anomalyPitchUp > 0) add("${s.anomalyPitchUp} looking up ≥25°")
                    if (s.anomalyPitchDown > 0) add("${s.anomalyPitchDown} looking down ≤−85°")
                    if (s.farPicks > 0) add("${s.farPicks} aimed too far")
                    addAll(s.validationMessages.take(3))
                }
                if (notes.isEmpty()) Hint("No issues. Red rings on the map = none.")
                else notes.forEach { Hint("• $it", warn = true) }
                Hint("ICP %.2f m · pitch %.0f…%.0f° · %.0f s".format(s.icpRmseM, s.pitchMin, s.pitchMax, s.elapsedSec))
                Secondary("Reject", onReject)
            }
            is HomeViewModel.UiState.Approving -> {
                FileLine(ui.file.displayName)
                LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                Hint("Uploading mission to the aircraft…")
            }
            is HomeViewModel.UiState.ReadyToFly -> {
                FileLine(ui.file.displayName)
                Primary("Open DJI Pilot 2", onOpenPilot2)
                Hint("${ui.summary.waypointCount} waypoints on the aircraft. Tap the AeroScan Fly widget in the live view.")
                Secondary("New mission", onReset)
            }
            is HomeViewModel.UiState.Error -> {
                ui.file?.let { FileLine(it.displayName) }
                Hint(ui.message, warn = true)
                if (ui.file != null) Primary("Retry", onAugment)
                Secondary("Pick another file", onPick)
            }
        }
    }
}

@Composable
private fun Primary(label: String, onClick: () -> Unit, enabled: Boolean = true) {
    Button(onClick = onClick, enabled = enabled, modifier = Modifier.fillMaxWidth()) { Text(label) }
}

@Composable
private fun Secondary(label: String, onClick: () -> Unit) {
    OutlinedButton(onClick = onClick, modifier = Modifier.fillMaxWidth()) { Text(label) }
}

@Composable
private fun FileLine(name: String) {
    Text(name, style = MaterialTheme.typography.titleSmall, maxLines = 1)
}

@Composable
private fun Hint(text: String, warn: Boolean = false) {
    Text(
        text,
        style = MaterialTheme.typography.bodySmall,
        color = if (warn) toneColor(Tone.Warn) else MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

private fun formatSize(bytes: Long): String = when {
    bytes < 0 -> "?"
    bytes < 1024 -> "$bytes B"
    bytes < 1024 * 1024 -> "${bytes / 1024} KB"
    else -> "%.2f MB".format(bytes / 1024.0 / 1024.0)
}

private fun openPilot2(ctx: android.content.Context) {
    // Industry Edition (M4E ships with this) is `com.dji.industry.pilot`;
    // consumer Pilot 2 is `dji.go.v5`. Try Industry first, fall back to consumer.
    val pkgs = listOf("com.dji.industry.pilot", "dji.go.v5")
    val intent = pkgs.firstNotNullOfOrNull { ctx.packageManager.getLaunchIntentForPackage(it) }
        ?: Intent(Intent.ACTION_MAIN).addCategory(Intent.CATEGORY_LAUNCHER)
    intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK
    runCatching { ctx.startActivity(intent) }
}
