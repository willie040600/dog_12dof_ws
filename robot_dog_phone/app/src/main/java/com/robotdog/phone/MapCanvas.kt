package com.robotdog.phone

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.FilterQuality
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PointMode
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.sp
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin

// the colours of the window's map (map_widget.py)
private val DOG = Color(0xFF1565C0)
private val PATH = Color(0xFF2E7D32)
private val TARGET = Color(0xFFD32F2F)
private val PLACE = Color(0xFF6D4C41)
private val THING = Color(0xFFEF6C00)
private val PENDING = Color(0xFF757575)
private const val DOG_SIZE = 0.35  // m from the dog's centre to the tip of its triangle

const val NO_MAP = "等地圖…（nav.launch.py 開了嗎？）"

/** The apartment, the places, the remembered things, the path, the target and the dog; onTap gets map points. */
@Composable
fun MapCanvas(
    info: MapInfo?, image: ImageBitmap?, state: RobotState?, places: List<Place>, pending: Point?,
    onTap: ((Point) -> Unit)?, modifier: Modifier = Modifier,
) {
    val measurer = rememberTextMeasurer()
    Box(modifier.background(Color(0xFFE0E0E0))) {
        if (info == null || image == null) {
            Text(NO_MAP, Modifier.align(Alignment.Center), color = Color(0xFF606060))
            return@Box
        }
        val tap = if (onTap == null) Modifier else Modifier.pointerInput(info, onTap) {
            detectTapGestures { offset ->
                MapGeometry(info, size.width.toFloat(), size.height.toFloat()).toWorld(offset.x, offset.y)?.let(onTap)
            }
        }
        Canvas(Modifier.fillMaxSize().then(tap)) {
            val g = MapGeometry(info, size.width, size.height)
            fun at(x: Double, y: Double) = g.toPixel(x, y).let { Offset(it.first, it.second) }
            drawImage(
                image, dstOffset = IntOffset(g.left.roundToInt(), g.top.roundToInt()),
                dstSize = IntSize((info.width * g.scale).roundToInt(), (info.height * g.scale).roundToInt()),
                filterQuality = FilterQuality.None,
            )
            val label = TextStyle(fontSize = 9.sp)
            for (p in places) {
                val c = at(p.x, p.y)
                drawCircle(PLACE, 3.5f * density / 2f + 2f, c)
                drawText(measurer, p.short, c + Offset(5f, -14f * density), label.copy(color = PLACE))
            }
            for (t in state?.objects.orEmpty()) {
                val c = at(t.x, t.y)
                val half = 4.5f * density / 1.5f
                drawRect(THING, c - Offset(half, half), androidx.compose.ui.geometry.Size(2 * half, 2 * half))
                drawText(measurer, t.name, c + Offset(6f, 2f), label.copy(color = THING))
            }
            val path = state?.path.orEmpty()
            if (path.size >= 2) {
                drawPoints(path.map { at(it.x, it.y) }, PointMode.Polygon, PATH, strokeWidth = 2.5f * density)
            }
            state?.target?.let { cross(at(it.x, it.y), TARGET) }
            pending?.let { cross(at(it.x, it.y), PENDING) }
            state?.pose?.let { pose ->
                val c = at(pose.x, pose.y)
                val length = max((DOG_SIZE / info.resolution * g.scale).toFloat(), 9f * density)
                val tri = Path()
                listOf(0.0 to length, 2.5 to length * 0.7f, -2.5 to length * 0.7f).forEachIndexed { i, (a, l) ->
                    val px = c.x + l * cos(pose.yaw + a).toFloat()
                    val py = c.y - l * sin(pose.yaw + a).toFloat()
                    if (i == 0) tri.moveTo(px, py) else tri.lineTo(px, py)
                }
                tri.close()
                drawPath(tri, DOG)
                drawPath(tri, Color.White, style = Stroke(1.5f * density))
            }
        }
    }
}

private fun DrawScope.cross(c: Offset, color: Color) {
    val d = 7f * density
    drawLine(color, c - Offset(d, d), c + Offset(d, d), 3f * density)
    drawLine(color, c - Offset(d, -d), c + Offset(d, -d), 3f * density)
}
