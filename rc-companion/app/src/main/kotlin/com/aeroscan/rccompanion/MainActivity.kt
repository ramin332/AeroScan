package com.aeroscan.rccompanion

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import com.aeroscan.rccompanion.ui.HomeScreen
import com.aeroscan.rccompanion.ui.theme.AeroScanRCTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            AeroScanRCTheme {
                HomeScreen()
            }
        }
    }
}
