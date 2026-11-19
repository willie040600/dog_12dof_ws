package com.robotdog.phone

import com.sun.net.httpserver.HttpServer
import kotlinx.coroutines.runBlocking
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import java.net.InetSocketAddress
import java.net.ServerSocket

class ApiTest {
    private val server = HttpServer.create(InetSocketAddress("127.0.0.1", 0), 0).apply { start() }
    private val address = "127.0.0.1:${server.address.port}"
    private val seen = mutableListOf<String?>()

    @After fun stop() = server.stop(0)

    private fun answer(path: String, status: Int, body: String, type: String = "application/json") {
        server.createContext(path) { exchange ->
            seen += exchange.requestHeaders.getFirst("Authorization")
            val bytes = body.toByteArray()
            exchange.responseHeaders.add("Content-Type", type)
            exchange.sendResponseHeaders(status, bytes.size.toLong())
            exchange.responseBody.use { it.write(bytes) }
        }
    }

    private inline fun <reified E : ApiError> assertFails(block: () -> Unit) {
        try {
            block()
            fail("no error")
        } catch (e: ApiError) {
            assertTrue("got $e", e is E)
        }
    }

    @Test fun sendsTheCode() = runBlocking {
        answer("/api/hello", 200, sample("hello.json"))
        assertEquals(8, Api(address, "123456").hello().places.size)
        assertEquals(listOf("Bearer 123456"), seen)
    }

    @Test fun mapsErrors() = runBlocking {
        answer("/api/hello", 401, """{"error": "配對碼錯誤"}""")
        answer("/api/state", 429, """{"error": "x"}""")
        answer("/api/events", 500, "oops", "text/plain")
        answer("/api/ping", 200, """{"app": "other", "version": 1}""")
        val api = Api(address, "000000")
        assertFails<ApiError.BadCode> { runBlocking { api.hello() } }
        assertFails<ApiError.Locked> { runBlocking { api.state(-1) } }
        assertFails<ApiError.Http> { runBlocking { api.events(0) } }
        assertFails<ApiError.NotOurServer> { runBlocking { api.ping() } }
        val free = ServerSocket(0).use { it.localPort }  // nothing listens there now
        assertFails<ApiError.Unreachable> { runBlocking { Api("127.0.0.1:$free", "1").ping() } }
    }

    @Test fun pingNeedsNoCode() = runBlocking {
        answer("/api/ping", 200, """{"app": "robot_dog_phone", "version": 1}""")
        Api(address, "123456").ping()
        assertEquals(listOf<String?>(null), seen)
    }

    @Test fun commandReturnsOk() = runBlocking {
        answer("/api/command", 200, """{"ok": false}""")
        answer("/api/go_to", 200, """{"ok": true}""")
        val api = Api(address, "1")
        assertFalse(api.command("去廚房"))
        assertTrue(api.goTo(1.2, 0.5))
    }

    @Test fun readsTheStateAndTheMap() = runBlocking {
        answer("/api/state", 200, sample("state.json"))
        answer("/api/map.png", 200, "PNG!", "image/png")
        val api = Api(address, "1")
        assertEquals(1, api.state(-1).pathVersion)
        assertArrayEquals("PNG!".toByteArray(), api.mapPng())
    }
}
