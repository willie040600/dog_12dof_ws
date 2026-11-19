package com.robotdog.phone

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.launch

const val NO_LOCAL_NETWORK = "沒有「區域網路裝置」權限就連不到電腦：請到手機的「設定 > 應用程式 > 機器狗 > 權限」允許。"

/** The PC's address and the pairing code (spec 2026-11-19, 5.2). */
@Composable
fun SettingsScreen(model: RobotViewModel) {
    var address by remember { mutableStateOf(model.settings.address) }
    var code by remember { mutableStateOf(model.settings.code) }
    var message by remember { mutableStateOf(model.setupMessage) }
    var connecting by remember { mutableStateOf(false) }
    val scope = rememberCoroutineScope()

    Column(
        Modifier.fillMaxSize().systemBarsPadding().imePadding().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        Text("連到電腦", style = MaterialTheme.typography.headlineSmall)
        Text("在電腦上執行 ros2 run robot_dog_agent phone，終端機會印出位址和配對碼。")
        OutlinedTextField(
            address, { address = it }, Modifier.fillMaxWidth().testTag("address"),
            label = { Text("電腦位址（IP:port）") }, singleLine = true,
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Uri),
        )
        OutlinedTextField(
            code, { code = it.filter(Char::isDigit).take(6) }, Modifier.fillMaxWidth().testTag("code"),
            label = { Text("配對碼（6 位數字）") }, singleLine = true,
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.NumberPassword),
        )
        message?.let { Text(it, color = Color(0xFFC62828)) }
        if (!model.localNetworkAllowed) {
            Text(NO_LOCAL_NETWORK, color = Color(0xFFC62828))
        }
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Button(
                onClick = {
                    connecting = true
                    message = null
                    scope.launch {
                        message = model.connect(address, code)?.message
                        connecting = false
                    }
                },
                enabled = !connecting && address.isNotBlank() && code.length == 6,
                modifier = Modifier.testTag("connect"),
            ) { Text(if (connecting) "連線中…" else "連線") }
            if (model.settings.code.isNotEmpty() && model.setupMessage == null) {
                OutlinedButton(onClick = { model.closeSetup() }) { Text("取消") }
            }
        }
    }
}
