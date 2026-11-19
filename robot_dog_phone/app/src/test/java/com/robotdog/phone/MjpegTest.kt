package com.robotdog.phone

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertNull
import org.junit.Test
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream

class MjpegTest {
    private fun part(data: ByteArray): ByteArray =
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${data.size}\r\n\r\n".toByteArray() +
            data + "\r\n".toByteArray()

    private val parts = listOf(
        byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 1, 2, 0xFF.toByte(), 0xD9.toByte()),
        byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 0x0D, 0x0A, 0x2D, 0x2D, 0xFF.toByte(), 0xD9.toByte()),  // \r\n-- inside
        ByteArray(5000) { (it % 251).toByte() },
    )

    @Test fun splitsParts() {
        val stream = ByteArrayOutputStream().apply { parts.forEach { write(part(it)) } }.toByteArray()
        val reader = MjpegReader(ByteArrayInputStream(stream))
        parts.forEach { assertArrayEquals(it, reader.next()) }
        assertNull(reader.next())
    }

    @Test fun emptyPartIsAnEmptyPicture() {  // the server's keep-alive while it has no fresh picture
        val reader = MjpegReader(ByteArrayInputStream(part(ByteArray(0)) + part(parts[0])))
        assertArrayEquals(ByteArray(0), reader.next())
        assertArrayEquals(parts[0], reader.next())
    }

    @Test fun truncatedStreamEnds() {
        val whole = part(parts[2])
        val reader = MjpegReader(ByteArrayInputStream(whole.copyOf(whole.size / 2)))
        assertNull(reader.next())
    }
}
