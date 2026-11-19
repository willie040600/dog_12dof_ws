package com.robotdog.phone

import android.app.Application
import android.graphics.BitmapFactory
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

enum class Connection { Connected, Retrying, NeedsSetup }

/** Everything the screens show, kept up to date from the server while the app is in the foreground. */
class RobotViewModel(application: Application) : AndroidViewModel(application) {
    val settings = Settings(application)

    var connection by mutableStateOf(if (settings.code.isEmpty()) Connection.NeedsSetup else Connection.Retrying)
        private set
    var setupMessage by mutableStateOf<String?>(null)
        private set
    var hello by mutableStateOf<Hello?>(null)
        private set
    var state by mutableStateOf<RobotState?>(null)
        private set
    var mapImage by mutableStateOf<ImageBitmap?>(null)
        private set
    var mapInfo by mutableStateOf<MapInfo?>(null)
        private set
    var frame by mutableStateOf<ImageBitmap?>(null)
        private set
    var chat by mutableStateOf<List<String>>(emptyList())
        private set
    var pendingTarget by mutableStateOf<Point?>(null)
        private set

    private var api: Api? = null
    private var loops: Job? = null
    private val chatLog = ChatLog()
    private var pathVersion = -1
    private var path: List<Point> = emptyList()
    private var camera: MjpegReader? = null

    val idle get() = connection == Connection.Connected && state?.state == "idle"
    val busy get() = connection == Connection.Connected && state?.state == "busy"

    /** Try the address and code; on success keep them and start. Returns why not, or null. */
    suspend fun connect(address: String, code: String): ApiError? {
        val candidate = Api(address.trim(), code.trim())
        try {
            candidate.ping()
            candidate.hello()
        } catch (e: ApiError) {
            return e
        }
        settings.address = candidate.address
        settings.code = code.trim()
        setupMessage = null
        pause()
        connection = Connection.Retrying
        resume()
        return null
    }

    fun openSetup() {
        pause()
        connection = Connection.NeedsSetup
    }

    /** Start the three loops: state every 0.2 s, events by long poll, the camera. */
    fun resume() {
        // not while the settings page is up: a wrong code would be sent again and again (and lock the phone out)
        if (loops != null || connection == Connection.NeedsSetup) return
        val api = Api(settings.address, settings.code).also { this.api = it }
        loops = viewModelScope.launch {
            launch { stateLoop(api) }
            launch { eventLoop(api) }
            launch { cameraLoop(api) }
        }
    }

    fun pause() {
        loops?.cancel()
        loops = null
        camera?.close()
        camera = null
    }

    private fun lost(e: ApiError) {
        if (e is ApiError.BadCode) {  // e.g. the server made a new code: ask for it
            setupMessage = e.message
            pause()
            connection = Connection.NeedsSetup
        } else {
            connection = Connection.Retrying
        }
    }

    private suspend fun stateLoop(api: Api) {
        var mapVersion = -1
        var boot: String? = null
        while (viewModelScope.isActive) {
            try {
                if (hello == null || boot == null) {
                    val h = api.hello()
                    if (h.boot != boot) { mapVersion = -1; pathVersion = -1 }  // a restarted server counts afresh
                    boot = h.boot
                    hello = h
                }
                val s = api.state(pathVersion)
                s.path?.let { path = it }
                pathVersion = s.pathVersion
                val info = s.map
                if (info != null && info.version != mapVersion) {
                    val png = api.mapPng()
                    mapImage = withContext(Dispatchers.Default) {
                        BitmapFactory.decodeByteArray(png, 0, png.size)?.asImageBitmap()
                    }
                    mapVersion = info.version
                }
                mapInfo = info
                if (info == null) mapImage = null
                state = s.copy(path = path)
                if (s.state != "idle") pendingTarget = null
                connection = Connection.Connected
                delay(STATE_PERIOD)
            } catch (e: ApiError) {
                lost(e)
                boot = null
                delay(RETRY_PERIOD)
            }
        }
    }

    private suspend fun eventLoop(api: Api) {
        var after = 0
        while (viewModelScope.isActive) {
            try {
                val asked = after
                after = chatLog.apply(api.events(asked), asked)
                chat = chatLog.lines
            } catch (e: ApiError) {
                delay(RETRY_PERIOD)
            }
        }
    }

    private suspend fun cameraLoop(api: Api) {
        while (viewModelScope.isActive) {
            if (hello?.camera != true) {
                frame = null
                delay(RETRY_PERIOD)
                continue
            }
            try {
                val reader = api.camera().also { camera = it }
                withContext(Dispatchers.IO) {
                    reader.use {
                        while (isActive) {
                            val jpeg = it.next() ?: break
                            frame = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.size)?.asImageBitmap() ?: continue
                        }
                    }
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {  // ApiError, or the stream broke off (IOException): try again
            }
            frame = null
            delay(RETRY_PERIOD)
        }
    }

    private fun act(block: suspend (Api) -> Unit) {
        val api = api ?: return
        viewModelScope.launch {
            try {
                block(api)
            } catch (e: ApiError) {
                lost(e)
            }
        }
    }

    fun send(text: String, onSent: () -> Unit) = act { if (it.command(text)) onSent() }
    fun goToPlace(place: Place) = act { it.command("去" + place.short) }
    fun survey() = act { it.survey() }
    fun stop() = act { it.stop() }

    fun tapMap(point: Point) {
        if (idle) pendingTarget = point
    }

    fun confirmTarget() {
        val target = pendingTarget ?: return
        pendingTarget = null
        act { it.goTo(Math.round(target.x * 100) / 100.0, Math.round(target.y * 100) / 100.0) }
    }

    fun cancelTarget() {
        pendingTarget = null
    }

    override fun onCleared() = pause()

    companion object {
        const val STATE_PERIOD = 200L  // ms
        const val RETRY_PERIOD = 2000L // ms
    }
}
