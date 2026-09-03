package com.aeroscan.rccompanion.ui

import androidx.compose.ui.graphics.Color

/**
 * One place for the mission view's colours so the 2D map and the 3D scene
 * cannot drift apart. Gimbal pitch is a magnitude, so it gets a single-hue
 * sequential ramp (level → light, straight down → dark); facade coverage is a
 * state, so it gets the status pair.
 */
object MissionPalette {
    val aimAugmentedLevel = Color(0xFF7FB3FF)
    val aimAugmentedSteep = Color(0xFF0B3D91)
    val aimOriginal = Color(0xFF9AA3AD)
    val facadeCovered = Color(0xFF2E7D32)
    val facadeUncovered = Color(0xFFC62828)
    val cloud = Color(0xFF8D99A6)
    val path = Color(0xFF5A6673)
    val flag = Color(0xFFC62828)
    val start = Color(0xFF1F6FEB)

    /**
     * Colour for a gimbal pitch. [pitchDeg] runs 0 (level, at a wall) to −90
     * (straight down, at the ground); positive is looking up. The ramp is
     * lightness only — one hue — so a glance separates wall shots from ground
     * shots without adding a second meaning to hue.
     */
    fun pitchColor(pitchDeg: Double): Color {
        val t = ((-pitchDeg) / 90.0).coerceIn(0.0, 1.0).toFloat()
        return lerp(aimAugmentedLevel, aimAugmentedSteep, t)
    }

    private fun lerp(a: Color, b: Color, t: Float) = Color(
        red = a.red + (b.red - a.red) * t,
        green = a.green + (b.green - a.green) * t,
        blue = a.blue + (b.blue - a.blue) * t,
        alpha = 1f,
    )
}
