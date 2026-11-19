package com.robotdog.phone

import java.io.BufferedInputStream
import java.io.ByteArrayOutputStream
import java.io.InputStream

/** Reads the JPEG pictures of the server's multipart/x-mixed-replace stream, one per next(), by Content-Length. */
class MjpegReader(input: InputStream) : AutoCloseable {
    private val input = BufferedInputStream(input)

    /** The next picture, or null when the stream ended (or broke off in the middle of a picture). */
    fun next(): ByteArray? {
        var length = -1
        var inHeaders = false
        while (true) {
            val line = readLine() ?: return null
            if (!inHeaders) {
                if (line.startsWith("--")) inHeaders = true
                continue
            }
            if (line.isEmpty()) {
                if (length < 0) { inHeaders = false; continue }
                break
            }
            val colon = line.indexOf(':')
            if (colon > 0 && line.substring(0, colon).trim().equals("Content-Length", ignoreCase = true)) {
                length = line.substring(colon + 1).trim().toIntOrNull() ?: -1
            }
        }
        val data = ByteArray(length)
        var read = 0
        while (read < length) {
            val n = input.read(data, read, length - read)
            if (n < 0) return null
            read += n
        }
        return data
    }

    /** One header line without its line end, or null at the end of the stream. */
    private fun readLine(): String? {
        val bytes = ByteArrayOutputStream()
        while (true) {
            val b = input.read()
            if (b < 0) return if (bytes.size() == 0) null else bytes.toString(Charsets.US_ASCII.name())
            if (b == '\n'.code) break
            if (b != '\r'.code) bytes.write(b)
        }
        return bytes.toString(Charsets.US_ASCII.name())
    }

    override fun close() = input.close()
}
