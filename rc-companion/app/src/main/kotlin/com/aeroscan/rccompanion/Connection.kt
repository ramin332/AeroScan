package com.aeroscan.rccompanion

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

object Connection {

    sealed interface State {
        data object Initializing : State
        data object Registered : State
        data class RegisterFailed(val reason: String) : State
        data class AircraftConnected(val productId: Int) : State
        data object AircraftDisconnected : State
    }

    private val _state = MutableStateFlow<State>(State.Initializing)
    val state: StateFlow<State> = _state.asStateFlow()

    fun publish(s: State) {
        _state.value = s
    }
}
