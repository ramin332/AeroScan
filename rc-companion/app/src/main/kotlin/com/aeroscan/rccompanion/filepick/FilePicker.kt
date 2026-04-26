package com.aeroscan.rccompanion.filepick

import android.content.Context
import android.database.Cursor
import android.net.Uri
import android.provider.OpenableColumns
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.platform.LocalContext

data class PickedFile(
    val uri: Uri,
    val displayName: String,
    val sizeBytes: Long,
)

@Composable
fun rememberKmzPicker(onPicked: (PickedFile) -> Unit): () -> Unit {
    val ctx = LocalContext.current
    val launcher = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri != null) onPicked(uriToPickedFile(uri, ctx))
    }
    return remember(launcher) {
        {
            // SAF mime filter is best-effort — DJI Pilot 2 saves KMZ as
            // application/zip on some devices, and RC storage may report
            // application/octet-stream. The "Send" path validates extension.
            launcher.launch(arrayOf(
                "application/vnd.google-earth.kmz",
                "application/zip",
                "application/octet-stream",
                "*/*",
            ))
        }
    }
}

private fun uriToPickedFile(uri: Uri, ctx: Context): PickedFile {
    var name = uri.lastPathSegment ?: "selected.kmz"
    var size: Long = -1
    val cursor: Cursor? = ctx.contentResolver.query(uri, null, null, null, null)
    cursor?.use { c ->
        if (c.moveToFirst()) {
            val nIx = c.getColumnIndex(OpenableColumns.DISPLAY_NAME)
            val sIx = c.getColumnIndex(OpenableColumns.SIZE)
            if (nIx >= 0) name = c.getString(nIx) ?: name
            if (sIx >= 0 && !c.isNull(sIx)) size = c.getLong(sIx)
        }
    }
    return PickedFile(uri = uri, displayName = name, sizeBytes = size)
}
