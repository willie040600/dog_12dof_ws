package com.robotdog.phone

/** The conversation as shown: server events with the window's prefixes; a new boot of the server starts it over. */
class ChatLog {
    private var boot: String? = null
    private val shown = mutableListOf<String>()

    val lines: List<String> get() = shown.toList()

    /** Adds the events and returns the number to ask after next. */
    fun apply(events: Events): Int {
        if (events.boot != boot) {
            boot = events.boot
            shown.clear()
        }
        for (e in events.events) {
            if (e.kind != "state") shown += prefix(e.kind) + e.text
        }
        return events.next
    }

    companion object {
        fun prefix(kind: String) = when (kind) {
            "user" -> "你："
            "reply" -> "狗："
            "error" -> "[錯誤] "
            else -> ""
        }
    }
}
