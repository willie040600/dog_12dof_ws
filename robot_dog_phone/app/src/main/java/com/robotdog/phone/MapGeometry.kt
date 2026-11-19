package com.robotdog.phone

/**
 * A map drawn as large as fits, centred, in a view of viewWidth x viewHeight pixels; map metres <-> view pixels.
 * The same arithmetic as robot_dog_agent's map_view.MapView: y is up, so cell row 0 is at the bottom.
 */
class MapGeometry(val info: MapInfo, viewWidth: Float, viewHeight: Float) {
    /** Pixels per cell. */
    val scale: Float = minOf(viewWidth / info.width, viewHeight / info.height)
    val left: Float = (viewWidth - info.width * scale) / 2f
    val top: Float = (viewHeight - info.height * scale) / 2f

    fun toPixel(x: Double, y: Double): Pair<Float, Float> {
        val col = (x - info.originX) / info.resolution
        val row = (y - info.originY) / info.resolution
        return Pair(left + (col * scale).toFloat(), top + ((info.height - row) * scale).toFloat())
    }

    /** The map point under a view pixel, or null outside the picture of the map. */
    fun toWorld(px: Float, py: Float): Point? {
        val col = (px - left) / scale.toDouble()
        val row = info.height - (py - top) / scale.toDouble()
        if (col < 0.0 || col > info.width || row < 0.0 || row > info.height) return null
        return Point(info.originX + col * info.resolution, info.originY + row * info.resolution)
    }
}
