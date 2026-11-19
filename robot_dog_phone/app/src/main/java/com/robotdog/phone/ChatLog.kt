package com.robotdog.phone

/** The conversation as shown: server events with the window's prefixes; a new boot of the server starts it over. */
class ChatLog {
    private var boot: String? = null
    private val shown = mutableListOf<String>()

    val lines: List<String> get() = shown.toList()

    /**
     * Adds the events of a request for those after askedAfter, and returns the number to ask after next. A new boot
     * of the server numbers its events from 1 again: an answer to an old number is dropped, and 0 is returned so the
     * next request starts from the beginning.
     */
    fun apply(events: Events, askedAfter: Int = 0): Int {
        if (events.boot != boot) {
            boot = events.boot
            shown.clear()
            if (askedAfter != 0) return 0
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
