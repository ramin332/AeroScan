package com.aeroscan.rccompanion.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.mop.AugmentFraming

/**
 * The four things a pilot needs to know before tapping Augment, as chips:
 * aircraft link, Manifold app, newest scan, recorded mission. Pure mapping
 * ([panelStatusFor]) kept MSDK-free so it is unit-testable.
 */
enum class Tone { Good, Warn, Bad, Neutral }

data class Chip(val label: String, val tone: Tone)

data class PanelStatus(val aircraft: Chip, val manifold: Chip, val scan: Chip, val mission: Chip)

/** Manifold's own mesh age limit (kmz_runner AEROSCAN_MESH_MAX_AGE_S default). */
const val MESH_FRESH_LIMIT_S = 6 * 3600L

fun panelStatusFor(
    conn: Connection.State,
    banner: BannerState,
    status: AugmentFraming.ManifoldStatus?,
): PanelStatus {
    val aircraft = when (conn) {
        is Connection.State.AircraftConnected -> Chip("Aircraft linked", Tone.Good)
        is Connection.State.AircraftDisconnected -> Chip("Aircraft off", Tone.Bad)
        is Connection.State.Registered -> Chip("Waiting for aircraft", Tone.Neutral)
        is Connection.State.Initializing -> Chip("Starting…", Tone.Neutral)
        is Connection.State.RegisterFailed -> Chip("App not registered", Tone.Bad)
    }
    val manifold = when (banner) {
        is BannerState.Unreachable -> Chip("AeroScan off — enable in Pilot 2", Tone.Bad)
        is BannerState.EnvError -> Chip("Manifold env error", Tone.Bad)
        is BannerState.Checking, BannerState.Idle -> Chip("Manifold…", Tone.Neutral)
        is BannerState.Ready, is BannerState.NoMesh -> Chip(
            if (status != null) "Manifold ok · %.0f GB free".format(status.blackboxFreeGb) else "Manifold ok",
            Tone.Good,
        )
    }
    val scan = when {
        status == null -> Chip("Scan ?", Tone.Neutral)
        status.meshPresent -> Chip(
            if (status.meshAgeS >= 0) "Scan ${ageText(status.meshAgeS)} old" else "Scan fresh",
            Tone.Good,
        )
        status.meshStale -> Chip("Scan ${ageText(status.meshAgeS)} old (${status.meshFlight})", Tone.Warn)
        else -> Chip("No scan — fly Smart3D first", Tone.Bad)
    }
    val mission = when (status?.missionState) {
        null, "" -> Chip("No mission on drone", Tone.Neutral)
        "ready" -> Chip("Uploaded — tap Fly", Tone.Good)
        "flying" -> Chip("Flying ${status.missionLastIndex}/${status.missionTotal}", Tone.Good)
        "paused" -> Chip("Paused ${status.missionLastIndex}/${status.missionTotal}", Tone.Warn)
        "interrupted" -> Chip("Interrupted at ${status.missionLastIndex}/${status.missionTotal}", Tone.Warn)
        "completed" -> Chip("Done ${status.missionLastIndex}/${status.missionTotal}", Tone.Good)
        else -> Chip(status.missionState, Tone.Neutral)
    }
    return PanelStatus(aircraft, manifold, scan, mission)
}

/**
 * Why Augment is disabled right now, or null when it may run. Old scans are
 * allowed (the Manifold's age gate is bypassed on purpose — the chip shows
 * the age so the pilot knows); only "no scan at all" blocks.
 */
fun augmentBlockReason(
    conn: Connection.State,
    banner: BannerState,
    status: AugmentFraming.ManifoldStatus?,
): String? = when {
    conn !is Connection.State.AircraftConnected -> "Power on the aircraft and pair the RC."
    banner is BannerState.Unreachable -> "AeroScan app is not running on the drone. Pilot 2 → camera view → payload panel → enable AeroScan."
    banner is BannerState.EnvError -> "Manifold environment error — augment cannot run."
    banner is BannerState.NoMesh && (status == null || !status.meshExists) -> "No scan on the drone. Fly a Smart3D scan first."
    banner is BannerState.Checking || banner is BannerState.Idle -> "Checking the drone…"
    else -> null
}

@Composable
fun StatusStrip(
    status: PanelStatus,
    checking: Boolean,
    onRefresh: () -> Unit,
    modifier: Modifier = Modifier,
) {
    Row(
        modifier = modifier
            .fillMaxWidth()
            .clickable(onClick = onRefresh),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        StatusChip(status.aircraft)
        StatusChip(status.manifold)
        StatusChip(status.scan)
        StatusChip(status.mission)
        if (checking) CircularProgressIndicator(modifier = Modifier.size(16.dp), strokeWidth = 2.dp)
    }
}

fun toneColor(tone: Tone): Color = when (tone) {
    Tone.Good -> Color(0xFF2E7D32)
    Tone.Warn -> Color(0xFFEF8F00)
    Tone.Bad -> Color(0xFFC62828)
    Tone.Neutral -> Color(0xFF78909C)
}

@Composable
private fun RowScope.StatusChip(chip: Chip) {
    val cs = MaterialTheme.colorScheme
    Row(
        modifier = Modifier
            .weight(1f)
            .clip(RoundedCornerShape(6.dp))
            .background(cs.surfaceVariant)
            .padding(horizontal = 10.dp, vertical = 6.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.size(10.dp).clip(CircleShape).background(toneColor(chip.tone)))
        Spacer(Modifier.width(8.dp))
        Text(
            chip.label,
            style = MaterialTheme.typography.labelMedium,
            maxLines = 1,
            overflow = TextOverflow.Ellipsis,
        )
    }
}
