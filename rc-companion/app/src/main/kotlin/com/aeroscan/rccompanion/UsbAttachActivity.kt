package com.aeroscan.rccompanion

import android.app.Activity
import android.content.Intent
import android.os.Bundle

/**
 * Catches `android.hardware.usb.action.USB_ACCESSORY_ATTACHED` so the RC Plus 2
 * routes the OcuSync-link USB accessory to this app. Re-launches MainActivity
 * and finishes — DJI's MSDK keys off the accessory attach to detect the aircraft.
 *
 * Mirrors `UsbAttachActivity` from `Mobile-SDK-Android-V5` sample.
 */
class UsbAttachActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val intent = Intent(this, MainActivity::class.java).apply {
            addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK)
        }
        startActivity(intent)
        finish()
    }
}
