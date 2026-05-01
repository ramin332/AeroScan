package com.aeroscan.rccompanion

import android.app.Activity
import android.os.Bundle

/**
 * Silent absorber for `USB_ACCESSORY_ATTACHED`. The RC Plus 2's OcuSync radio is
 * an internal USB accessory — MSDK V5 only binds the aircraft channel after the
 * system delivers this intent to *some* Activity in the package. We intentionally
 * do not relaunch MainActivity here: doing so reorders the task and drops MSDK's
 * live-view surface, which presents as a Connected→Disconnected flicker.
 */
class UsbAttachActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        finish()
    }
}
