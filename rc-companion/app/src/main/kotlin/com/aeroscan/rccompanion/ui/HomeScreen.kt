package com.aeroscan.rccompanion.ui

import android.content.Intent
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.filepick.rememberKmzPicker

@Composable
fun HomeScreen(viewModel: HomeViewModel = viewModel()) {
    val ui by viewModel.ui.collectAsStateWithLifecycle()
    val conn by viewModel.connection.collectAsStateWithLifecycle()
    val banner by viewModel.banner.collectAsStateWithLifecycle()
    val ctx = LocalContext.current

    val pick = rememberKmzPicker { viewModel.onFilePicked(it) }

    // Probe the Manifold once when the screen first appears so the pilot sees
    // readiness without an extra tap. Re-probes whenever the aircraft link
    // (re)connects, since the PING can't reach the Manifold until it's up.
    LaunchedEffect(conn) {
        if (conn is Connection.State.AircraftConnected) viewModel.checkStatus()
    }

    Scaffold { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(16.dp)
                // ReviewCard with anomaly section + Approve/Reject buttons
                // is taller than typical RC screens (768 dp) — without scroll
                // the buttons clip below the visible area. Verified live
                // on-device 2026-05-06 after first successful PRVW receive.
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Text("AeroScan RC", style = MaterialTheme.typography.headlineMedium)
            ConnectionBanner(conn)
            ReadinessBanner(state = banner, onRefresh = viewModel::checkStatus)

            when (val s = ui) {
                is HomeViewModel.UiState.Idle -> IdleCard(onPick = pick)
                is HomeViewModel.UiState.Picked -> PickedCard(
                    name = s.file.displayName,
                    size = s.file.sizeBytes,
                    onPick = pick,
                    onAugment = viewModel::augment,
                )
                is HomeViewModel.UiState.ParsingKmz -> SimpleStatusCard(
                    title = "Parsing KMZ…",
                    body = "${s.file.displayName} — extracting waypoints + downsampling cloud to 1 m",
                )
                is HomeViewModel.UiState.Uploading -> UploadingCard(
                    name = s.file.displayName,
                    sent = s.sent,
                    total = s.total,
                    onCancel = viewModel::cancel,
                )
                is HomeViewModel.UiState.AwaitingPreview -> SimpleStatusCard(
                    title = "Augmenting on Manifold…",
                    body = "Typically 4 minutes for a Mijande-class building. " +
                            "The Manifold is registering the perception cloud, " +
                            "extracting facades, and re-aiming gimbals.",
                )
                is HomeViewModel.UiState.ReviewReady -> ReviewCard(
                    name = s.file.displayName,
                    summary = s.summary,
                    savedKmzPath = s.savedKmzPath,
                    onApprove = viewModel::approve,
                    onReject = viewModel::reject,
                )
                is HomeViewModel.UiState.Approving -> SimpleStatusCard(
                    title = "Approving…",
                    body = "Telling the Manifold to upload the augmented mission to the aircraft.",
                )
                is HomeViewModel.UiState.ReadyToFly -> ReadyToFlyCard(
                    name = s.file.displayName,
                    summary = s.summary,
                    onOpenPilot2 = { openPilot2(ctx) },
                    onAnother = viewModel::reset,
                )
                is HomeViewModel.UiState.Error -> ErrorCard(
                    message = s.message,
                    canRetry = s.file != null,
                    onRetry = viewModel::augment,
                    onPickAnother = pick,
                )
            }
        }
    }
}

@Composable
private fun ConnectionBanner(state: Connection.State) {
    val (text, container) = when (state) {
        is Connection.State.Initializing -> "Initializing App..." to MaterialTheme.colorScheme.surfaceVariant
        is Connection.State.Registered -> "App registered, waiting for aircraft" to MaterialTheme.colorScheme.surfaceVariant
        is Connection.State.RegisterFailed -> "App registration failed: ${state.reason}" to MaterialTheme.colorScheme.errorContainer
        is Connection.State.AircraftConnected -> "Your drone has connected!)" to MaterialTheme.colorScheme.primaryContainer
        is Connection.State.AircraftDisconnected -> "Aircraft disconnected" to MaterialTheme.colorScheme.errorContainer
    }
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = container),
    ) {
        Text(text, modifier = Modifier.padding(12.dp))
    }
}

@Composable
private fun ReadinessBanner(state: BannerState, onRefresh: () -> Unit) {
    // Colour + glyph encode the readiness verdict at a glance. Greens/reds are
    // explicit hex (not theme roles) so the verdict reads the same regardless
    // of the active Material colour scheme.
    val ready = Color(0xFF2E7D32)
    val problem = Color(0xFFC62828)
    val neutral = Color(0xFF455A64)
    val (container, label) = when (state) {
        is BannerState.Idle -> neutral to "Manifold readiness — tap Check"
        is BannerState.Checking -> neutral to "Checking Manifold…"
        is BannerState.Ready -> ready to "🟢 ${state.label}"
        is BannerState.NoMesh -> problem to "🔴 ${state.label}"
        is BannerState.EnvError -> problem to "🟠 ${state.label}"
        is BannerState.Unreachable -> neutral to "⚪ ${state.label}"
    }
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = container),
    ) {
        Column(modifier = Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text(
                label,
                color = Color.White,
                style = MaterialTheme.typography.bodyMedium,
            )
            TextButton(
                onClick = onRefresh,
                enabled = state !is BannerState.Checking,
            ) {
                Text(
                    if (state is BannerState.Checking) "Checking…" else "Check Manifold",
                    color = Color.White,
                )
            }
        }
    }
}

@Composable
private fun IdleCard(onPick: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Step 1 — Pick a KMZ from RC storage", style = MaterialTheme.typography.titleMedium)
            Text(
                "After upload completes, switch to DJI Pilot 2 and tap the AeroScan widget on the live-flight view to fly the mission.",
                style = MaterialTheme.typography.bodyMedium,
            )
            Button(onClick = onPick) { Text("Pick KMZ…") }
        }
    }
}

@Composable
private fun PickedCard(name: String, size: Long, onPick: () -> Unit, onAugment: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Picked", style = MaterialTheme.typography.titleMedium)
            Text(name, style = MaterialTheme.typography.bodyLarge)
            Text(formatSize(size), style = MaterialTheme.typography.bodySmall)
            Spacer(Modifier.height(4.dp))
            Button(onClick = onAugment, modifier = Modifier.fillMaxWidth()) { Text("Augment Mission") }
            OutlinedButton(onClick = onPick, modifier = Modifier.fillMaxWidth()) { Text("Pick a different file") }
        }
    }
}

@Composable
private fun SimpleStatusCard(title: String, body: String) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
            Text(body, style = MaterialTheme.typography.bodyMedium)
        }
    }
}

@Composable
private fun ReviewCard(
    name: String,
    summary: HomeViewModel.PreviewSummary,
    savedKmzPath: String?,
    onApprove: () -> Unit,
    onReject: () -> Unit,
) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Review augmented mission", style = MaterialTheme.typography.titleMedium)
            Text(name, style = MaterialTheme.typography.bodyLarge)

            Spacer(Modifier.height(4.dp))
            // Summary stats
            Text("• Waypoints: ${summary.waypointCount} (${summary.waypointsAimed} aimed at facades)",
                 style = MaterialTheme.typography.bodyMedium)
            Text("• Facades extracted: ${summary.facadeCount}", style = MaterialTheme.typography.bodyMedium)
            Text("• Gimbal pitch range: %.0f° to %.0f° (median %.1f°)".format(
                summary.pitchMin, summary.pitchMax, summary.pitchMedian),
                 style = MaterialTheme.typography.bodyMedium)
            Text("• ICP RMSE: %.3f m  •  Augmented in %.0f s".format(summary.icpRmseM, summary.elapsedSec),
                 style = MaterialTheme.typography.bodySmall)

            // Anomaly flags — these are the WPs the pilot should sanity-check.
            if (summary.anomalyPitchUp > 0 || summary.anomalyPitchDown > 0) {
                Spacer(Modifier.height(4.dp))
                Card(
                    modifier = Modifier.fillMaxWidth(),
                    colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.tertiaryContainer),
                ) {
                    Column(modifier = Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text("Anomalies (review before approving)",
                             style = MaterialTheme.typography.titleSmall)
                        if (summary.anomalyPitchUp > 0) {
                            Text("• ${summary.anomalyPitchUp} WPs pitched ≥ +25° (overhang/eave OR sky shot)",
                                 style = MaterialTheme.typography.bodySmall)
                        }
                        if (summary.anomalyPitchDown > 0) {
                            Text("• ${summary.anomalyPitchDown} WPs pitched ≤ −85° (base/ground OR floor)",
                                 style = MaterialTheme.typography.bodySmall)
                        }
                    }
                }
            }

            // Saved-to-disk note. Helps the pilot find the augmented KMZ via
            // USB / Files app to inspect on a laptop later.
            if (savedKmzPath != null) {
                Spacer(Modifier.height(4.dp))
                Text(
                    "Saved: $savedKmzPath",
                    style = MaterialTheme.typography.bodySmall,
                )
            } else {
                Spacer(Modifier.height(4.dp))
                Text(
                    "Save to device storage failed (KMZ still in memory until reject).",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.error,
                )
            }

            Spacer(Modifier.height(8.dp))
            Button(onClick = onApprove, modifier = Modifier.fillMaxWidth()) {
                Text("Approve & upload to aircraft")
            }
            OutlinedButton(onClick = onReject, modifier = Modifier.fillMaxWidth()) {
                Text("Reject")
            }
        }
    }
}

@Composable
private fun ReadyToFlyCard(
    name: String,
    summary: HomeViewModel.PreviewSummary,
    onOpenPilot2: () -> Unit,
    onAnother: () -> Unit,
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer),
    ) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Ready to fly ✓", style = MaterialTheme.typography.titleMedium)
            Text("$name — ${summary.waypointCount} waypoints uploaded to the aircraft.",
                 style = MaterialTheme.typography.bodyMedium)
            Text("Switch to DJI Pilot 2 and tap the AeroScan: Fly widget on the live-flight view.",
                 style = MaterialTheme.typography.bodyMedium)
            Button(onClick = onOpenPilot2, modifier = Modifier.fillMaxWidth()) { Text("Open DJI Pilot 2") }
            OutlinedButton(onClick = onAnother, modifier = Modifier.fillMaxWidth()) { Text("Augment another mission") }
        }
    }
}

@Composable
private fun UploadingCard(name: String, sent: Long, total: Long, onCancel: () -> Unit) {
    val pct = if (total > 0) (sent.toFloat() / total.toFloat()).coerceIn(0f, 1f) else 0f
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Uploading", style = MaterialTheme.typography.titleMedium)
            Text(name, style = MaterialTheme.typography.bodyMedium)
            LinearProgressIndicator(progress = { pct }, modifier = Modifier.fillMaxWidth())
            Text("${formatSize(sent)} / ${formatSize(total)}  (${(pct * 100).toInt()}%)")
            OutlinedButton(onClick = onCancel) { Text("Cancel") }
        }
    }
}

@Composable
private fun ErrorCard(message: String, canRetry: Boolean, onRetry: () -> Unit, onPickAnother: () -> Unit) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.errorContainer),
    ) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Upload failed", style = MaterialTheme.typography.titleMedium)
            Text(message, style = MaterialTheme.typography.bodyMedium)
            if (canRetry) Button(onClick = onRetry, modifier = Modifier.fillMaxWidth()) { Text("Retry") }
            OutlinedButton(onClick = onPickAnother, modifier = Modifier.fillMaxWidth()) { Text("Pick a different file") }
        }
    }
}

private fun formatSize(bytes: Long): String = when {
    bytes < 0 -> "unknown size"
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
