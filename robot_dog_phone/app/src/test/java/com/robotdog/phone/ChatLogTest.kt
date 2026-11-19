package com.robotdog.phone

import org.junit.Assert.assertEquals
import org.junit.Test

class ChatLogTest {
    private fun events(boot: String, vararg items: Pair<Int, Pair<String, String>>) =
        Events(boot, items.map { (id, e) -> Event(id, e.first, e.second) }, items.maxOfOrNull { it.first } ?: 0)

    @Test fun prefixesAndSkipsState() {
        val log = ChatLog()
        log.apply(
            events(
                "a", 1 to ("user" to "去廚房"), 2 to ("state" to "busy"), 3 to ("log" to "  [工具] x"),
                4 to ("reply" to "到了"), 5 to ("error" to "壞了"),
            )
        )
        assertEquals(listOf("你：去廚房", "  [工具] x", "狗：到了", "[錯誤] 壞了"), log.lines)
    }

    @Test fun continuesWithinABoot() {
        val log = ChatLog()
        assertEquals(2, log.apply(events("a", 1 to ("log" to "1"), 2 to ("log" to "2"))))
        assertEquals(3, log.apply(events("a", 3 to ("log" to "3"))))
        assertEquals(listOf("1", "2", "3"), log.lines)
    }

    @Test fun bootChangeClearsTheLog() {
        val log = ChatLog()
        log.apply(events("a", 1 to ("log" to "1"), 2 to ("log" to "2"), 3 to ("log" to "3")))
        // the restarted server answered "after 3" with its own numbers: drop it, ask again from the start
        assertEquals(0, log.apply(events("b", 3 to ("log" to "new 3")), askedAfter = 3))
        assertEquals(emptyList<String>(), log.lines)
        assertEquals(3, log.apply(events("b", 1 to ("log" to "new 1"), 2 to ("log" to "new 2"), 3 to ("log" to "new 3"))))
        assertEquals(listOf("new 1", "new 2", "new 3"), log.lines)
    }

    @Test fun emptyAnswerKeepsTheNumber() {
        val log = ChatLog()
        log.apply(events("a", 1 to ("log" to "1")))
        assertEquals(1, log.apply(Events("a", emptyList(), 1)))
        assertEquals(listOf("1"), log.lines)
    }
}
