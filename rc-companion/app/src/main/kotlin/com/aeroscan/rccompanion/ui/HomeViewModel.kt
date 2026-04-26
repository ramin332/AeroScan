package com.aeroscan.rccompanion.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.aeroscan.rccompanion.Connection
import com.aeroscan.rccompanion.filepick.PickedFile
import com.aeroscan.rccompanion.mop.MopFileSender
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class HomeViewModel(app: Application) : AndroidViewModel(app) {

    sealed interface UiState {
        data object Idle : UiState
        data class Picked(val file: PickedFile) : UiState
        data class Uploading(val file: PickedFile, val sent: Long, val total: Long) : UiState
        data class Done(val file: PickedFile) : UiState
        data class Error(val file: PickedFile?, val message: String) : UiState
    }

    private val _ui = MutableStateFlow<UiState>(UiState.Idle)
    val ui: StateFlow<UiState> = _ui.asStateFlow()

    val connection: StateFlow<Connection.State> = Connection.state

    private var sendJob: Job? = null

    fun onFilePicked(picked: PickedFile) {
        _ui.value = UiState.Picked(picked)
    }

    fun reset() {
        sendJob?.cancel()
        sendJob = null
        _ui.value = UiState.Idle
    }

    fun cancel() {
        sendJob?.cancel()
    }

    fun send() {
        val current = _ui.value
        val picked = when (current) {
            is UiState.Picked -> current.file
            is UiState.Error -> current.file
            else -> return
        } ?: return

        if (connection.value !is Connection.State.AircraftConnected) {
            _ui.value = UiState.Error(picked, "M4E not connected. Power on the aircraft and pair the RC.")
            return
        }

        sendJob = viewModelScope.launch {
            _ui.value = UiState.Uploading(picked, 0, picked.sizeBytes.coerceAtLeast(0))
            val ctx = getApplication<Application>().applicationContext
            val sender = MopFileSender()

            val input = runCatching { ctx.contentResolver.openInputStream(picked.uri) }
                .getOrNull()
            if (input == null) {
                _ui.value = UiState.Error(picked, "Could not open the picked file. Try re-selecting.")
                return@launch
            }

            input.use { stream ->
                val total = if (picked.sizeBytes > 0) picked.sizeBytes else 0L
                val result = sender.send(picked.displayName, total, stream) { sent, totalBytes ->
                    _ui.value = UiState.Uploading(picked, sent, totalBytes)
                }
                _ui.value = when (result) {
                    is MopFileSender.Result.Ok -> UiState.Done(picked)
                    is MopFileSender.Result.Cancelled -> UiState.Idle
                    is MopFileSender.Result.Failed -> UiState.Error(
                        picked,
                        result.cause.message ?: "Upload failed for an unknown reason."
                    )
                }
            }
        }
    }
}
