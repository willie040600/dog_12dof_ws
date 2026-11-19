package com.robotdog.phone

import android.content.Context

/** The PC's address and the pairing code, kept on the phone. */
class Settings(context: Context) {
    private val prefs = context.getSharedPreferences("robot_dog_phone", Context.MODE_PRIVATE)

    var address: String
        get() = prefs.getString("address", DEFAULT_ADDRESS) ?: DEFAULT_ADDRESS
        set(value) = prefs.edit().putString("address", value).apply()

    var code: String
        get() = prefs.getString("code", "") ?: ""
        set(value) = prefs.edit().putString("code", value).apply()

    companion object {
        const val DEFAULT_ADDRESS = "192.168.0.154:8765"
    }
}
