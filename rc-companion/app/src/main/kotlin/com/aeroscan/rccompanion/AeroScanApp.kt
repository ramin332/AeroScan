package com.aeroscan.rccompanion

import android.app.Application
import android.content.Context
import android.util.Log
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.network.DJINetworkManager

class AeroScanApp : Application() {

    @Volatile private var sdkInitialized = false

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        // DJI's native-library loader. Required by the aircraft variant of MSDK V5
        // — matches the call in Mobile-SDK-Android-V5/.../DJIAircraftApplication.kt.
        com.cySdkyc.clx.Helper.install(this)
    }

    override fun onCreate() {
        super.onCreate()
        SDKManager.getInstance().init(this, object : SDKManagerCallback {
            override fun onRegisterSuccess() {
                Log.i(TAG, "MSDK registered")
                Connection.publish(Connection.State.Registered)
            }

            override fun onRegisterFailure(error: IDJIError) {
                Log.e(TAG, "MSDK register failed: ${error.errorCode()} ${error.description()}")
                Connection.publish(Connection.State.RegisterFailed(error.description() ?: "unknown"))
            }

            override fun onProductConnect(productId: Int) {
                Log.i(TAG, "Product connected: $productId")
                Connection.publish(Connection.State.AircraftConnected(productId))
            }

            override fun onProductDisconnect(productId: Int) {
                Log.i(TAG, "Product disconnected: $productId")
                Connection.publish(Connection.State.AircraftDisconnected)
            }

            override fun onProductChanged(productId: Int) = Unit

            override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    sdkInitialized = true
                    SDKManager.getInstance().registerApp()
                }
            }

            override fun onDatabaseDownloadProgress(current: Long, total: Long) = Unit
        })

        DJINetworkManager.getInstance().addNetworkStatusListener { available ->
            if (sdkInitialized && available && !SDKManager.getInstance().isRegistered) {
                SDKManager.getInstance().registerApp()
            }
        }
    }

    companion object {
        private const val TAG = "AeroScanApp"
    }
}
