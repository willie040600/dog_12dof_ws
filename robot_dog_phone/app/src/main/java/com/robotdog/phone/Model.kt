package com.robotdog.phone

import org.json.JSONArray
import org.json.JSONObject

/** What the server on the PC sends (spec 2026-11-19, 4). */
data class Place(val id: String, val name: String, val short: String, val x: Double, val y: Double)
data class Hello(val boot: String, val places: List<Place>, val canSurvey: Boolean, val camera: Boolean)
data class Pose(val x: Double, val y: Double, val yaw: Double)
data class Point(val x: Double, val y: Double)
data class Thing(val name: String, val x: Double, val y: Double)
data class MapInfo(
    val version: Int, val resolution: Double, val originX: Double, val originY: Double, val width: Int, val height: Int,
)

/** path == null: the same path as the version the app asked with. */
data class RobotState(
    val state: String, val status: String, val pose: Pose?, val poseText: String, val pathVersion: Int,
    val path: List<Point>?, val target: Point?, val objects: List<Thing>, val map: MapInfo?,
)

data class Event(val id: Int, val kind: String, val text: String)
data class Events(val boot: String, val events: List<Event>, val next: Int)

private fun JSONObject.orNull(name: String): Any? = if (isNull(name)) null else get(name)

private fun <T> JSONArray.map(item: (Int) -> T): List<T> = List(length()) { item(it) }

private fun point(a: JSONArray) = Point(a.getDouble(0), a.getDouble(1))

fun parseHello(text: String): Hello {
    val o = JSONObject(text)
    val places = o.getJSONArray("places").map {
        val p = o.getJSONArray("places").getJSONObject(it)
        Place(p.getString("id"), p.getString("name"), p.getString("short"), p.getDouble("x"), p.getDouble("y"))
    }
    return Hello(o.getString("boot"), places, o.getBoolean("can_survey"), o.getBoolean("camera"))
}

fun parseState(text: String): RobotState {
    val o = JSONObject(text)
    val pose = (o.orNull("pose") as JSONArray?)?.let { Pose(it.getDouble(0), it.getDouble(1), it.getDouble(2)) }
    val path = (o.orNull("path") as JSONArray?)?.let { a -> a.map { point(a.getJSONArray(it)) } }
    val target = (o.orNull("target") as JSONArray?)?.let(::point)
    val things = o.getJSONArray("objects")
    val objects = things.map {
        val t = things.getJSONObject(it)
        Thing(t.getString("name"), t.getDouble("x"), t.getDouble("y"))
    }
    val map = (o.orNull("map") as JSONObject?)?.let {
        val origin = it.getJSONArray("origin")
        MapInfo(
            it.getInt("version"), it.getDouble("resolution"), origin.getDouble(0), origin.getDouble(1),
            it.getInt("width"), it.getInt("height"),
        )
    }
    return RobotState(
        o.getString("state"), o.getString("status"), pose, o.getString("pose_text"), o.getInt("path_version"),
        path, target, objects, map,
    )
}

fun parseEvents(text: String): Events {
    val o = JSONObject(text)
    val list = o.getJSONArray("events")
    val events = list.map {
        val e = list.getJSONObject(it)
        Event(e.getInt("id"), e.getString("kind"), e.getString("text"))
    }
    return Events(o.getString("boot"), events, o.getInt("next"))
}
