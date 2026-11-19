package com.robotdog.phone

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class MapGeometryTest {
    private val info = MapInfo(1, 0.05, -5.0, -3.0, 200, 120)  // 10 m x 6 m

    private fun assertRoundTrip(g: MapGeometry, x: Double, y: Double) {
        val (px, py) = g.toPixel(x, y)
        val back = requireNotNull(g.toWorld(px, py))
        assertEquals(x, back.x, 1e-4)
        assertEquals(y, back.y, 1e-4)
    }

    @Test fun scalesAndCentres() {
        val g = MapGeometry(info, 400f, 400f)  // wide map in a square view: margins above and below
        assertEquals(2f, g.scale, 1e-6f)
        assertEquals(0f, g.left, 1e-6f)
        assertEquals(80f, g.top, 1e-6f)
        assertEquals(Pair(0f, 320f), g.toPixel(-5.0, -3.0))  // bottom-left corner of the map
    }

    @Test fun roundTrip() = assertRoundTrip(MapGeometry(info, 400f, 300f), 1.2, 0.5)

    @Test fun yIsUp() {
        val g = MapGeometry(info, 400f, 300f)
        assertTrue(g.toPixel(0.0, 2.0).second < g.toPixel(0.0, 0.0).second)
    }

    @Test fun marginIsOutside() {
        val g = MapGeometry(info, 400f, 400f)
        assertNull(g.toWorld(200f, 40f))   // above the picture
        assertNull(g.toWorld(200f, 360f))  // below it
        assertNotNull(g.toWorld(200f, 200f))
    }

    @Test fun followsANewMapSize() {
        val bigger = MapGeometry(info.copy(version = 2, width = 300, originX = -10.0), 400f, 400f)
        assertNotEquals(MapGeometry(info, 400f, 400f).scale, bigger.scale)
        assertRoundTrip(bigger, -7.5, 0.5)
    }
}
