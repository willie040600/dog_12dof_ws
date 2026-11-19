package com.robotdog.phone

import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.Modifier
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.semantics.testTagsAsResourceId

class MainActivity : ComponentActivity() {
    private val model: RobotViewModel by viewModels()

    @OptIn(ExperimentalComposeUiApi::class)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)  // the user watches the dog while it walks
        enableEdgeToEdge()
        askForLocalNetwork()
        setContent {
            MaterialTheme {
                // test tags as resource ids: the emulator checks find the buttons with uiautomator
                Surface(Modifier.fillMaxSize().semantics { testTagsAsResourceId = true }) { App(model) }
            }
        }
    }

    private val localNetworkRequest =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { model.localNetworkAllowed = it }

    /** Android 17 (API 37) and later block private addresses, such as the PC's, without this permission. */
    private fun askForLocalNetwork() {
        if (Build.VERSION.SDK_INT < LOCAL_NETWORK_SDK) return
        if (checkSelfPermission(LOCAL_NETWORK) == PackageManager.PERMISSION_GRANTED) return
        model.localNetworkAllowed = false
        localNetworkRequest.launch(LOCAL_NETWORK)
    }

    override fun onStart() {
        super.onStart()
        model.resume()
    }

    override fun onStop() {
        model.pause()  // no polling, no camera in the background
        super.onStop()
    }
}

private const val LOCAL_NETWORK = "android.permission.ACCESS_LOCAL_NETWORK"
private const val LOCAL_NETWORK_SDK = 37

@Composable
fun App(model: RobotViewModel) {
    if (model.connection == Connection.NeedsSetup) SettingsScreen(model) else MainScreen(model)
}
