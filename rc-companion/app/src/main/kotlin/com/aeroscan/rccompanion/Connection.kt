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

    /**
     * Timestamps (ms) of the aircraft-link drops we have seen. Only one MSDK app
     * may hold the aircraft link at a time — DJI's guidance for the RC Plus is to
     * force-exit Pilot 2 before running a third-party MSDK app. With Pilot 2 alive
     * in the background the MSDK core hands the link back and forth: the RC's
     * logcat on 2026-09-03 shows CoreExistReceiver "setNeedTryConnect false"
     * followed by a disconnect roughly every 22 s while our app was in front.
     * [LinkHealth] turns that history into something the pilot can act on.
     */
    private val _drops = MutableStateFlow<List<Long>>(emptyList())
    val drops: StateFlow<List<Long>> = _drops.asStateFlow()

    /** When the current link came up (ms), or null while it is down. */
    private val _upSince = MutableStateFlow<Long?>(null)
    val upSince: StateFlow<Long?> = _upSince.asStateFlow()

    fun publish(s: State, nowMs: Long = System.currentTimeMillis()) {
        val was = _state.value
        _state.value = s
        when {
            s is State.AircraftConnected && was !is State.AircraftConnected -> _upSince.value = nowMs
            s !is State.AircraftConnected && was is State.AircraftConnected -> {
                _upSince.value = null
                _drops.value = (_drops.value + nowMs).takeLast(LINK_DROP_HISTORY)
            }
        }
    }

    const val LINK_DROP_HISTORY = 12
}
