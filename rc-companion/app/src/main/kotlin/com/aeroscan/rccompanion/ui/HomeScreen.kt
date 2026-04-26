package com.aeroscan.rccompanion.ui

import android.content.Intent
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
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
    val ctx = LocalContext.current

    val pick = rememberKmzPicker { viewModel.onFilePicked(it) }

    Scaffold { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Text("AeroScan RC", style = MaterialTheme.typography.headlineMedium)
            ConnectionBanner(conn)

            when (val s = ui) {
                is HomeViewModel.UiState.Idle -> IdleCard(onPick = pick)
                is HomeViewModel.UiState.Picked -> PickedCard(
                    name = s.file.displayName,
                    size = s.file.sizeBytes,
                    onPick = pick,
                    onSend = viewModel::send,
                )
                is HomeViewModel.UiState.Uploading -> UploadingCard(
                    name = s.file.displayName,
                    sent = s.sent,
                    total = s.total,
                    onCancel = viewModel::cancel,
                )
                is HomeViewModel.UiState.Done -> DoneCard(
                    name = s.file.displayName,
                    onOpenPilot2 = { openPilot2(ctx) },
                    onAnother = viewModel::reset,
                )
                is HomeViewModel.UiState.Error -> ErrorCard(
                    message = s.message,
                    canRetry = s.file != null,
                    onRetry = viewModel::send,
                    onPickAnother = pick,
                )
            }
        }
    }
}

@Composable
private fun ConnectionBanner(state: Connection.State) {
    val (text, container) = when (state) {
        is Connection.State.Initializing -> "Initializing MSDK…" to MaterialTheme.colorScheme.surfaceVariant
        is Connection.State.Registered -> "MSDK registered, waiting for aircraft" to MaterialTheme.colorScheme.surfaceVariant
        is Connection.State.RegisterFailed -> "MSDK registration failed: ${state.reason}" to MaterialTheme.colorScheme.errorContainer
        is Connection.State.AircraftConnected -> "M4E connected (id ${state.productId})" to MaterialTheme.colorScheme.primaryContainer
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
private fun PickedCard(name: String, size: Long, onPick: () -> Unit, onSend: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Picked", style = MaterialTheme.typography.titleMedium)
            Text(name, style = MaterialTheme.typography.bodyLarge)
            Text(formatSize(size), style = MaterialTheme.typography.bodySmall)
            Spacer(Modifier.height(4.dp))
            Button(onClick = onSend, modifier = Modifier.fillMaxWidth()) { Text("Send to Manifold") }
            OutlinedButton(onClick = onPick, modifier = Modifier.fillMaxWidth()) { Text("Pick a different file") }
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
private fun DoneCard(name: String, onOpenPilot2: () -> Unit, onAnother: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Sent ✓", style = MaterialTheme.typography.titleMedium)
            Text("$name is on Manifold. Switch to DJI Pilot 2 and tap the AeroScan widget to fly.", style = MaterialTheme.typography.bodyMedium)
            Button(onClick = onOpenPilot2, modifier = Modifier.fillMaxWidth()) { Text("Open DJI Pilot 2") }
            OutlinedButton(onClick = onAnother, modifier = Modifier.fillMaxWidth()) { Text("Send another file") }
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
    val pkg = "dji.go.v5"  // DJI Pilot 2 package name on RC Plus 2
    val intent = ctx.packageManager.getLaunchIntentForPackage(pkg)
        ?: Intent(Intent.ACTION_VIEW)
    intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK
    runCatching { ctx.startActivity(intent) }
}
