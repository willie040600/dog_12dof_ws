package com.robotdog.phone

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** The samples in test resources are written by robot_dog_agent's test_phone_server.py from the real server. */
fun sample(name: String): String =
    requireNotNull(ModelTest::class.java.classLoader!!.getResource("api/$name")) { name }.readText()

class ModelTest {
    @Test fun parsesHello() {
        val hello = parseHello(sample("hello.json"))
        assertEquals("0a1b2c3d", hello.boot)
        assertEquals(8, hello.places.size)
        assertEquals(Place("kitchen", "廚房（流理台前，面向流理台）", "廚房", 2.5, -0.6), hello.places[5])
        assertTrue(hello.canSurvey)
        assertTrue(hello.camera)
    }

    @Test fun parsesState() {
        val state = parseState(sample("state.json"))
        assertEquals("busy", state.state)
        assertEquals("處理中…", state.status)
        assertEquals(Pose(0.12, -0.3, 0.0), state.pose)
        assertTrue(state.poseText.startsWith("位置 (0.12, -0.30)"))
        assertEquals(1, state.pathVersion)
        assertEquals(listOf(Point(0.12, -0.3), Point(0.6, 0.1), Point(1.2, 0.5)), state.path)
        assertEquals(Point(1.2, 0.5), state.target)
        assertEquals(listOf(Thing("沙發", 1.0, 2.0)), state.objects)
        assertEquals(MapInfo(1, 0.05, -5.0, -3.0, 200, 120), state.map)
    }

    @Test fun parsesNullPoseAndMap() {
        val state = parseState(
            """{"state": "starting", "status": "等 Nav2 啟動…", "pose": null, "pose_text": "位置未知",
               "path_version": 0, "path": null, "target": null, "objects": [], "map": null}"""
        )
        assertNull(state.pose)
        assertNull(state.map)
        assertNull(state.path)
        assertNull(state.target)
        assertEquals(emptyList<Thing>(), state.objects)
    }

    @Test fun parsesEvents() {
        val events = parseEvents(sample("events.json"))
        assertEquals("0a1b2c3d", events.boot)
        assertEquals(6, events.next)
        assertEquals(6, events.events.size)
        assertEquals(Event(4, "user", "去廚房"), events.events[3])
    }
}
