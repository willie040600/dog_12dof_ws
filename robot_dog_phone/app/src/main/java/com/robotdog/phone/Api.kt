package com.robotdog.phone

import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONException
import org.json.JSONObject
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL

/** Why a request failed, in the words the app shows. */
sealed class ApiError(message: String) : Exception(message) {
    class Unreachable : ApiError("連不上電腦")
    class NotOurServer : ApiError("這個位址不是機器狗的伺服器")
    class BadCode : ApiError("配對碼錯誤")
    class Locked : ApiError("錯太多次，請一分鐘後再試")
    class Http(val code: Int) : ApiError("伺服器回應錯誤（HTTP $code）")
}

/** The server on the PC (spec 2026-11-19, 4); address is host:port. Every call runs on Dispatchers.IO. */
class Api(val address: String, private val code: String) {
    private fun open(path: String, readTimeout: Int, withCode: Boolean = true): HttpURLConnection {
        val connection = URL("http://$address$path").openConnection() as HttpURLConnection
        connection.connectTimeout = CONNECT_TIMEOUT
        connection.readTimeout = readTimeout
        connection.useCaches = false
        if (withCode) connection.setRequestProperty("Authorization", "Bearer $code")
        return connection
    }

    private fun check(connection: HttpURLConnection) {
        when (val status = connection.responseCode) {
            200 -> return
            401 -> throw ApiError.BadCode()
            429 -> throw ApiError.Locked()
            else -> throw ApiError.Http(status)
        }
    }

    private suspend fun request(
        path: String, body: String? = null, readTimeout: Int = READ_TIMEOUT, withCode: Boolean = true,
    ): ByteArray = withContext(Dispatchers.IO) {
        val connection = open(path, readTimeout, withCode)
        try {
            if (body != null) {
                connection.requestMethod = "POST"
                connection.doOutput = true
                connection.setRequestProperty("Content-Type", "application/json; charset=utf-8")
                connection.outputStream.use { it.write(body.toByteArray()) }
            }
            check(connection)
            connection.inputStream.use { it.readBytes() }
        } catch (e: IOException) {
            Log.w(TAG, "$path: $e")
            throw ApiError.Unreachable()
        } finally {
            connection.disconnect()
        }
    }

    private suspend fun text(path: String, body: String? = null, readTimeout: Int = READ_TIMEOUT) =
        request(path, body, readTimeout).toString(Charsets.UTF_8)

    private suspend fun ok(path: String, body: String = "{}") = JSONObject(text(path, body)).getBoolean("ok")

    /** Is this the robot dog's server? Needs no code. */
    suspend fun ping() {
        val answer = try {
            JSONObject(request("/api/ping", withCode = false).toString(Charsets.UTF_8)).optString("app")
        } catch (e: ApiError.Http) {
            throw ApiError.NotOurServer()
        } catch (e: JSONException) {
            throw ApiError.NotOurServer()
        }
        if (answer != "robot_dog_phone") throw ApiError.NotOurServer()
    }

    suspend fun hello(): Hello = parseHello(text("/api/hello"))
    suspend fun state(pathVersion: Int): RobotState = parseState(text("/api/state?path=$pathVersion"))
    suspend fun events(after: Int): Events = parseEvents(text("/api/events?after=$after", readTimeout = EVENTS_TIMEOUT))
    suspend fun command(text: String): Boolean = ok("/api/command", JSONObject().put("text", text).toString())
    suspend fun goTo(x: Double, y: Double): Boolean = ok("/api/go_to", JSONObject().put("x", x).put("y", y).toString())
    suspend fun survey(): Boolean = ok("/api/survey")
    suspend fun stop() { text("/api/stop", "{}") }
    suspend fun mapPng(): ByteArray = request("/api/map.png")

    /** The camera stream; the caller closes it (which also ends the connection). */
    suspend fun camera(): MjpegReader = withContext(Dispatchers.IO) {
        val connection = open("/api/camera.mjpg", CAMERA_TIMEOUT)
        try {
            check(connection)
            MjpegReader(connection.inputStream)
        } catch (e: IOException) {
            connection.disconnect()
            throw ApiError.Unreachable()
        } catch (e: ApiError) {
            connection.disconnect()
            throw e
        }
    }

    companion object {
        const val TAG = "robotdog"
        const val CONNECT_TIMEOUT = 3000  // ms
        const val READ_TIMEOUT = 5000     // ms
        const val EVENTS_TIMEOUT = 30000  // ms: the server holds a request for up to 25 s
        const val CAMERA_TIMEOUT = 10000  // ms without a picture before the stream counts as broken
    }
}
