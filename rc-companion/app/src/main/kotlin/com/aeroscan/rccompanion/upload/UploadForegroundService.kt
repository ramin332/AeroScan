package com.aeroscan.rccompanion.upload

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat

/**
 * Foreground service stub. The MOP transfer itself runs in HomeViewModel's
 * coroutine scope today; promoting it to this service is Phase 5 work that
 * lets a context switch to Pilot 2 mid-upload not abort the transfer.
 *
 * Wiring sketch when Phase 5 lands:
 *   - HomeViewModel starts this service with the picked Uri + filename
 *   - Service runs MopFileSender.send() in IO scope
 *   - Service posts progress to a notification and a process-wide StateFlow
 *     that the UI re-binds to on resume
 *   - Service self-stops on Done / Cancelled / Failed
 */
class UploadForegroundService : Service() {

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        ensureChannel(this)
        startForeground(NOTIFICATION_ID, buildNotification("Preparing upload…", null))
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        return START_NOT_STICKY
    }

    private fun buildNotification(text: String, progressPct: Int?): Notification {
        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("AeroScan RC")
            .setContentText(text)
            .setSmallIcon(android.R.drawable.stat_sys_upload)
            .setOngoing(true)
            .setOnlyAlertOnce(true)
        if (progressPct != null) builder.setProgress(100, progressPct, false)
        return builder.build()
    }

    companion object {
        private const val CHANNEL_ID = "aeroscan_rc_upload"
        private const val NOTIFICATION_ID = 1001

        private fun ensureChannel(ctx: Context) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                val mgr = ctx.getSystemService(NotificationManager::class.java)
                if (mgr.getNotificationChannel(CHANNEL_ID) == null) {
                    mgr.createNotificationChannel(
                        NotificationChannel(CHANNEL_ID, "Upload progress", NotificationManager.IMPORTANCE_LOW)
                    )
                }
            }
        }
    }
}
