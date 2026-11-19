package com.robotdog.phone

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import java.util.Locale

const val NO_CAMERA = "沒有相機畫面"
const val RETRYING = "連不上電腦，重試中…"
private val GREEN = Color(0xFF2E7D32)
private val ORANGE = Color(0xFFEF6C00)
private val RED = Color(0xFFC62828)

/** The phone's version of the window (spec 2026-11-19, 5.1), portrait. */
@Composable
fun MainScreen(model: RobotViewModel) {
    var cameraBig by rememberSaveable { mutableStateOf(false) }
    val state = model.state
    val connected = model.connection == Connection.Connected

    Column(Modifier.fillMaxSize().systemBarsPadding().imePadding()) {
        StatusBar(model)
        Box(Modifier.fillMaxWidth().aspectRatio(5f / 3.4f)) {
            val map = @Composable { modifier: Modifier, big: Boolean ->
                MapCanvas(
                    model.mapInfo, model.mapImage, state, model.hello?.places.orEmpty(), model.pendingTarget,
                    onTap = if (big) model::tapMap else null, modifier = modifier.testTag("map"),
                )
            }
            val camera = @Composable { modifier: Modifier ->
                CameraView(model, modifier.testTag("camera"))
            }
            val small = Modifier.align(Alignment.BottomEnd).padding(6.dp).fillMaxWidth(0.34f).aspectRatio(4f / 3f)
                .border(1.dp, Color.White).clickable { cameraBig = !cameraBig }
            if (cameraBig) {
                camera(Modifier.fillMaxSize())
                map(small, false)
            } else {
                map(Modifier.fillMaxSize(), true)
                camera(small)
            }
        }
        model.pendingTarget?.let { target ->
            Row(
                Modifier.fillMaxWidth().background(Color(0xFFF5F5F5)).padding(horizontal = 12.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text(String.format(Locale.ROOT, "去這裡 (%.2f, %.2f)", target.x, target.y), Modifier.weight(1f))
                Button(onClick = model::confirmTarget, Modifier.testTag("go_here")) { Text("去") }
                Spacer(Modifier.width(8.dp))
                OutlinedButton(onClick = model::cancelTarget, Modifier.testTag("cancel_target")) { Text("取消") }
            }
        }
        ChatList(model.chat, Modifier.weight(1f))
        HorizontalDivider()
        val idle = model.idle
        LazyRow(
            contentPadding = PaddingValues(horizontal = 8.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            items(model.hello?.places.orEmpty(), key = { it.id }) { place ->
                OutlinedButton(
                    onClick = { model.goToPlace(place) }, enabled = idle,
                    modifier = Modifier.testTag("place_" + place.id), contentPadding = PaddingValues(horizontal = 12.dp),
                ) { Text(place.short) }
            }
            if (model.hello?.canSurvey == true) {
                item {
                    OutlinedButton(
                        onClick = model::survey, enabled = idle, modifier = Modifier.testTag("survey"),
                        contentPadding = PaddingValues(horizontal = 12.dp),
                    ) { Text("巡邏") }
                }
            }
        }
        var text by rememberSaveable { mutableStateOf("") }
        val send = { if (text.isNotBlank()) model.send(text) { text = "" } }
        Row(Modifier.padding(horizontal = 8.dp), verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(
                text, { text = it }, Modifier.weight(1f).testTag("input"), enabled = idle, singleLine = true,
                placeholder = { Text("輸入指令，例如：去廚房") },
                keyboardOptions = KeyboardOptions(imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(onSend = { send() }),
            )
            Spacer(Modifier.width(8.dp))
            Button(onClick = { send() }, enabled = idle && text.isNotBlank(), modifier = Modifier.testTag("send")) {
                Text("送出")
            }
        }
        Button(
            onClick = model::stop, enabled = connected && model.busy,
            modifier = Modifier.fillMaxWidth().padding(8.dp).height(52.dp).testTag("stop"),
            colors = ButtonDefaults.buttonColors(containerColor = RED, contentColor = Color.White),
        ) { Text("停 止", fontWeight = FontWeight.Bold, fontSize = 18.sp) }
    }
}

@Composable
private fun StatusBar(model: RobotViewModel) {
    val state = model.state
    val connected = model.connection == Connection.Connected
    val (color, text) = when {
        !connected -> RED to RETRYING
        state == null -> ORANGE to "連線中…"
        state.state == "idle" -> GREEN to state.status
        state.state == "failed" -> RED to state.status
        else -> ORANGE to state.status
    }
    Row(Modifier.fillMaxWidth().padding(start = 12.dp), verticalAlignment = Alignment.CenterVertically) {
        Box(Modifier.size(12.dp).background(color, CircleShape))
        Column(Modifier.weight(1f).padding(start = 8.dp, top = 4.dp, bottom = 4.dp).testTag("status")) {
            Text(text, fontWeight = FontWeight.Bold, maxLines = 1, overflow = TextOverflow.Ellipsis)
            Text(state?.poseText ?: "位置未知", fontSize = 12.sp, color = Color(0xFF606060), maxLines = 1)
        }
        TextButton(onClick = model::openSetup, modifier = Modifier.testTag("settings")) { Text("⚙", fontSize = 22.sp) }
    }
}

@Composable
private fun CameraView(model: RobotViewModel, modifier: Modifier) {
    Box(modifier.background(Color(0xFF202020)), contentAlignment = Alignment.Center) {
        val frame = model.frame
        if (frame != null) {
            Image(frame, "相機畫面", Modifier.fillMaxSize(), contentScale = ContentScale.Fit)
        } else {
            Text(NO_CAMERA, color = Color(0xFFC0C0C0), fontSize = 12.sp)
        }
    }
}

@Composable
private fun ChatList(lines: List<String>, modifier: Modifier) {
    val list = rememberLazyListState()
    LaunchedEffect(lines.size) { if (lines.isNotEmpty()) list.animateScrollToItem(lines.size - 1) }
    LazyColumn(modifier.fillMaxWidth().padding(horizontal = 12.dp), state = list) {
        items(lines.size) { i ->
            val line = lines[i]
            val color = when {
                line.startsWith("[錯誤]") -> RED
                line.startsWith("你：") -> Color(0xFF1565C0)
                line.startsWith("狗：") -> Color.Black
                else -> Color(0xFF606060)
            }
            Text(line, color = color, fontSize = 14.sp, modifier = Modifier.padding(vertical = 2.dp))
        }
    }
}
