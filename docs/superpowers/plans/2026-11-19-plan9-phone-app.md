# 計畫 9：手機遙控 app（Android 前端 + 電腦端伺服器）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增 `ros2 run robot_dog_agent phone`（不開視窗的 HTTP 伺服器）和 Android app `robot_dog_phone/`，讓使用者在家裡的 Wi-Fi 上用手機做到電腦視窗能做的所有事。

**Architecture:** 電腦端把現有的 `Session`、CameraFeed、MapFeed、ObjectMemory 接到 Python 內建的 `ThreadingHTTPServer`：純 Python 的 `phone_state.py`（事件紀錄、路線與目標、狀態 JSON）和 `phone_code.py`（配對碼）可以單獨測試，`phone_server.py` 只做路由和串流，`phone.py` 負責組裝。手機端是 Kotlin + Jetpack Compose 的單一 Activity；解析、座標換算、MJPEG 切張和對話紀錄是純 Kotlin，用 JVM 單元測試；畫面由 `RobotViewModel` 驅動。

**Tech Stack:** Python 3.12、`http.server`、numpy、OpenCV（`cv2`，已在用）、rclpy（Jazzy）、pytest；Kotlin、Jetpack Compose、kotlinx-coroutines、Android Gradle Plugin、JUnit；Android 模擬器 `Medium_Phone_API_37.0`。

**Spec:** `docs/superpowers/specs/2026-11-19-phone-app-design.md`

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`AGENT` = `~/dog_12dof_ws/robot_dog_agent`，`PKG` = `AGENT/robot_dog_agent`，`PHONE` = `~/dog_12dof_ws/robot_dog_phone`，`WS9` = `~/dog_12dof_ws/.superpowers/sdd/2026-11-19-plan9-phone-app`（拋棄式腳本、下載、log、截圖）。
- Python 測試：`cd $AGENT && python3 -m pytest test -q`，不 source ROS、不需要網路；原有 219 個必須仍然通過。伺服器測試只綁 `127.0.0.1`、port 0（隨機）。
- Android：`cd $PHONE && JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64 ./gradlew test`（JVM 單元測試）和 `./gradlew assembleDebug`。SDK 在 `/home/willie/Android/Sdk`（`local.properties`，不提交）。
- **不改** `gui.py`、`session.py`、`map_widget.py`，也不改 `robot_dog_description`、`robot_dog_navigation`、Nav2 參數、地圖檔。`chat`、`gui`、`agent_eval` 的行為不變。
- `phone_state.py`、`phone_code.py`、`phone_server.py` 不可匯入 rclpy、PyQt5、requests。`phone.py` 在模組層不可匯入 rclpy、PyQt5（rclpy 和 feeds 只在非 `--no-robot` 分支匯入），也不可 import `gui`。
- Python 端不安裝新套件。Android app 只用 AndroidX、Compose、kotlinx-coroutines；網路用 `HttpURLConnection`、JSON 用 `org.json`；**只有測試**加 `testImplementation("org.json:json:<最新穩定版>")`。
- 照 spec 的值：port 8765；配對碼 6 位數字、存 `~/.config/robot_dog/phone_code`（權限 600）；配對碼錯 → 等 1 秒再回 401；同一 IP 60 秒內錯 10 次 → 之後 60 秒全部 429；事件最多保留 300 筆；長輪詢最多 25 秒；路線點最多 300 個；目標 ✕ 指令結束後保留 2.0 秒；相機 JPEG 品質 70、每 0.1 秒一張、畫面超過 3.0 秒沒更新就不送；app 每 0.2 秒拿 `/api/state`、連不上每 2 秒重試；app 預設位址 `192.168.0.154:8765`，模擬器用 `10.0.2.2:8765`；minSdk 26；鎖直向。
- 畫面文字和電腦視窗相同：前綴 `你：`、`狗：`、`[錯誤] `；狀態文字來自伺服器（`Session.status_text()`、`pose_text`）。連不上時顯示 `連不上電腦，重試中…`。
- 第一次建置時 Gradle 會從 `services.gradle.org`、`dl.google.com`、Maven Central 下載工具和函式庫（spec 第 5.4 節已說明，使用者已同意）。其他東西不下載、不安裝；**不執行 `ufw` 或任何 sudo**（防火牆由使用者自己開，spec 第 7 節）。
- API key：`. ~/dog_12dof_ws/.superpowers/sdd/2026-11-10-plan5-agent-nav-chat/bin/key.sh`（從 `~/.bashrc` 載入，不印出）。每次 commit 前跑 `grep -rlF "$GEMINI_API_KEY" $AGENT $PHONE docs README.md`，必須沒有輸出。配對碼可以印在終端機，但不提交。
- Isaac Sim：用 `~/dog_12dof_ws/.superpowers/sdd/2026-11-11-plan6-agent-gui/bin/stack.sh`（開相機）。呼叫它的 Bash 指令裡不能出現它的比對字樣。同一時間只跑一個 Isaac Sim；啟動前確認使用者沒有開著。
- `.vscode/settings.json` 和 `robot_dog_navigation/rviz/nav.rviz` 是使用者的修改，不帶進任何 commit（`git commit -- <本計畫的路徑>`）。
- 程式註解英文；畫面文字、文件用繁體中文。commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。不 push。

## Review Focus

1. **伺服器重開時 app 還開著**：事件編號從 1 重來。期待 app 看到 `boot` 變了就清掉對話、從頭拿，不會漏訊息也不會卡住（Task 5 `ChatLogTest.bootChangeClearsTheLog`）。
2. **兩支手機同時按、或連點兩下**：`Session` 的待命檢查沒有鎖。期待只有一個指令執行，另一個 `ok=false`（Task 3 `test_two_commands_at_once_only_one_runs`）。
3. **猜配對碼**：錯 10 次之後，就算打對也要 429 到一分鐘後（Task 2 `test_locked_even_with_the_right_code`）。
4. **手機在串流或長輪詢途中斷線**（Wi-Fi 斷、app 退到背景）：期待伺服器那條執行緒結束、不丟例外到終端機、其他手機照常（Task 3 `test_client_hangs_up_mid_stream`）。
5. **地圖晚到或換大小**（剛啟動還沒有 `/map`、建圖模式地圖擴張）：期待 `map` 為 `null` 時 app 顯示等待文字，版本變了重抓 PNG、換算用新大小（Task 3 `test_map_version_follows_the_feed`；Task 5 `MapGeometryTest.followsANewMapSize`）。

---

### Task 1：狀態與事件（`phone_state.py`）

**Files:**
- Create: `PKG/phone_state.py`
- Test: `AGENT/test/test_phone_state.py`

**Interfaces:**
- Consumes: `map_view.grid_to_gray(grid) -> bytes`（已上下翻轉）；`session.pose_text(pose)`；`Session` 的 `state`、`status_text()`、`pose()`。
- Produces:
  - `MAX_EVENTS = 300`；`MAX_PATH_POINTS = 300`；`TARGET_HOLD = 2.0`。
  - `class EventLog(max_events=MAX_EVENTS)`：`add(kind, text) -> int`（編號從 1 起，執行緒安全）；`after(n, timeout) -> (list[dict], next)`，每筆 `{"id", "kind", "text"}`，有比 `n` 新的就立刻回，否則等到 `timeout` 回 `([], n)`；`n` 比保留的最舊還舊時從最舊的開始；`close()` 叫醒所有等待者並讓之後的 `after` 立刻回。
  - `class MapTracker(clock=time.monotonic)`：`on_state(state)`（`busy` 記下開始時間並清掉上一個指令的目標；離開 `busy` 時目標開始倒數 2.0 秒）；`set_target(x, y)`；`snapshot(latest_path) -> (path_version: int, path: list[[x, y]], target: [x, y] | None)`，`latest_path` 是 `MapFeed.latest_path()` 的結果（`([(x, y)...], 到達時間)` 或 `None`）；只有 `busy` 且到達時間 ≥ 開始時間的路線才算，否則 `[]`；路線內容變了 `path_version` 加一；超過 300 點就平均抽樣（保留第一點和最後一點）。
  - `state_json(session, tracker, latest_path, map_info, objects, path_version_seen) -> dict`：照 spec 第 4 節的欄位；`path` 只在 `path_version != path_version_seen` 時給，否則 `null`；`map_info` 是 `{"version","resolution","origin","width","height"}` 或 `None`；`objects` 是 `[{"name","x","y"}]`。
  - `grid_png(grid) -> bytes`：`grid_to_gray` 的結果用 `cv2.imencode('.png', ...)` 壓成灰階 PNG。
  - `read_objects(objects, last) -> list`：`objects()` 遇到 `RuntimeError` 重試一次，再失敗回 `last`。

- [ ] **Step 1：寫失敗的測試**

```python
def test_event_log_numbers_and_after():          # add 3 → after(0, 0) gives ids [1,2,3], next 3; after(3, 0) → ([], 3)
def test_event_log_wakes_a_waiter():             # a thread waits after(0, 5); add() from another thread; returns in < 0.5 s
def test_event_log_keeps_the_newest_300():       # 305 adds → after(0, 0) starts at id 6, 300 items
def test_event_log_close_wakes_waiters():        # waiter on after(0, 25) returns at once after close()
def test_path_only_for_the_running_command():    # path arrived before busy → []; arrived after busy → the points; idle → []
def test_path_version_changes_with_the_path():   # same path twice → same version; new points → version + 1
def test_long_path_is_sampled():                 # 1000 points → 300, first and last kept
def test_target_held_two_seconds():              # set_target, on_state('idle') at t=10 → target at t=11.9, None at t=12.1
def test_next_command_clears_the_target():       # target held, on_state('busy') → None at once
def test_state_json_fields():                    # keys: state status pose pose_text path_version path target objects map
def test_state_json_path_only_when_version_differs():
def test_state_json_no_pose_no_map():            # pose None → "pose": null, "pose_text": "位置未知", "map": null
def test_grid_png_round_trip():                  # 3x2 grid → cv2.imdecode gives 2 rows x 3 cols, top row = grid's last row
def test_read_objects_retries_then_keeps_last():
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: event log, path and target tracking for the phone server`

### Task 2：配對碼（`phone_code.py`）

**Files:**
- Create: `PKG/phone_code.py`
- Test: `AGENT/test/test_phone_code.py`

**Interfaces:**
- Produces:
  - `CODE_PATH = os.path.expanduser('~/.config/robot_dog/phone_code')`；`WRONG_WAIT = 1.0`；`MAX_WRONG = 10`；`WINDOW = 60.0`；`LOCK = 60.0`。
  - `load_or_create(path=CODE_PATH, new=False) -> str`：檔案存在且是 6 位數字就沿用；否則（或 `new=True`）用 `secrets.randbelow(10**6)` 產生 `'%06d'`，寫檔（目錄 `os.makedirs(..., exist_ok=True)`，檔案 `0o600`，原子寫入）。
  - `class Guard(code, clock=time.monotonic, sleep=time.sleep)`：`check(authorization: str | None, ip: str) -> int`，回 200、401 或 429。`authorization` 是 `Authorization` header 的值，必須是 `Bearer <code>`；比對用 `hmac.compare_digest`。錯一次 `sleep(WRONG_WAIT)` 再回 401；同一 IP 在 `WINDOW` 秒內錯到 `MAX_WRONG` 次，之後 `LOCK` 秒內一律 429（不 sleep，打對也一樣）。執行緒安全。

- [ ] **Step 1：寫失敗的測試**

```python
def test_creates_a_six_digit_code_with_mode_600(tmp_path):
def test_reuses_the_code(tmp_path):                 # two calls → same code
def test_new_code_replaces_it(tmp_path):
def test_bad_file_is_replaced(tmp_path):            # file contains 'abc' → new 6-digit code
def test_right_code_passes():                       # 'Bearer 123456' → 200, no sleep
def test_missing_or_wrong_code_is_401_after_a_wait():  # None, 'Bearer 000000', '123456' (no Bearer) → 401, sleep(1.0) each
def test_ten_wrong_in_a_minute_locks_the_ip():      # 10 wrong at t=0..9 → 11th call (right code) → 429; other IP → 200
def test_locked_even_with_the_right_code():         # locked at t=9 → 429 at t=68, 200 at t=70
def test_wrong_ones_older_than_a_minute_do_not_count():  # 9 wrong at t=0, 1 wrong at t=61 → not locked
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: pairing code for the phone`

### Task 3：HTTP 伺服器（`phone_server.py`）

**Files:**
- Create: `PKG/phone_server.py`
- Test: `AGENT/test/test_phone_server.py`
- Create（測試產生、兩邊共用的樣本）：`PHONE/app/src/test/resources/api/hello.json`、`state.json`、`events.json`

**Interfaces:**
- Consumes: Task 1 全部；Task 2 `Guard`；`vision.encode_jpeg(rgb, width, height, quality)`；`places.PLACES`；Session 的 `submit`、`go_to`、`survey`、`stop`、`state`、`status_text`、`pose`。
- Produces:
  - `Sources = namedtuple('Sources', 'latest_frame latest_map latest_path objects can_survey')`：`latest_frame() -> (w, h, rgb, 到達時間) | None` 或 `None`（沒有相機）；`latest_map() -> (grid, resolution, origin, version) | None`；`latest_path() -> (points, 到達時間) | None` 或 `None`；`objects() -> [(name, x, y)]` 或 `None`；`can_survey: bool`。
  - `class PhoneServer(session, guard, sources, host='0.0.0.0', port=8765, clock=time.monotonic)`：建構時產生 `boot = secrets.token_hex(4)`；`on_event(kind, text)` 給 `session.on_event` 用（寫進 `EventLog`，`state` 事件也交給 `MapTracker.on_state`）；`start()` 在 daemon 執行緒 `serve_forever`；`port` 屬性（實際綁到的）；`close()` 關事件紀錄、停伺服器、斷開串流。
  - 路由照 spec 第 4 節。`/api/ping` 不驗證，回 `{"app": "robot_dog_phone", "version": 1}`；其他先過 `Guard.check`（IP 用 `client_address[0]`），401／429 回 `{"error": ...}`。`/api/hello` 的 places 每筆 `{"id","name","short","x","y"}`，`short` = `name.split('（')[0]`；`camera` = `latest_frame is not None`。`POST` body 是 JSON；格式錯回 400。`/api/command`、`/api/go_to`、`/api/survey` 在**同一把鎖**裡呼叫 Session；`go_to` 成功後才 `tracker.set_target`；`can_survey` 為 False 時 `/api/survey` 直接 `ok=false`。`/api/map.png` 沒有地圖回 404。`/api/camera.mjpg`：沒有相機回 404；否則 `multipart/x-mixed-replace; boundary=frame`，每張帶 `Content-Type: image/jpeg` 和 `Content-Length`；寫入失敗（對方斷線）就結束迴圈。回應一律 `Cache-Control: no-store`；JSON 用 `ensure_ascii=False`、UTF-8。不輸出每個請求的存取紀錄（覆寫 `log_message`）。

- [ ] **Step 1：寫失敗的測試**（`FakeSession`：記錄呼叫、`state` 可設、`submit` 等方法照真的 Session 檢查待命；`urllib.request` 打 `http://127.0.0.1:<port>`）

```python
def test_ping_needs_no_code():
def test_everything_else_needs_the_code():          # /api/hello without header → 401; with → 200
def test_hello():                                   # 8 places, short '廚房', can_survey, camera, boot is 8 hex chars
def test_state_and_path_versions():                 # ?path=<current version> → "path": null; ?path=-1 → list
def test_events_long_poll():                        # after=0 returns history; after=next waits; on_event from a thread → returns
def test_command_go_to_survey_stop_reach_the_session():
def test_busy_gives_ok_false():
def test_survey_without_eyes_is_refused():          # can_survey False → ok false, session.survey never called
def test_two_commands_at_once_only_one_runs():      # FakeSession.submit sleeps 0.2 s between its check and set; 2 threads → one ok
def test_go_to_sets_the_target():                   # then /api/state target == [x, y]
def test_bad_json_is_400():
def test_map_png_and_404():
def test_map_version_follows_the_feed():            # latest_map version 1 → 2 with a bigger grid → state map.version 2, width changes
def test_camera_stream_sends_jpegs():               # first part's bytes start with FF D8, Content-Length matches
def test_no_camera_is_404():
def test_client_hangs_up_mid_stream():              # open camera.mjpg, read one part, close socket; server keeps serving /api/ping
def test_nothing_contains_the_key():                # FakeSession built with key 'AIzaFAKE-KEY'; every response body lacks it
def test_android_samples_are_current():             # hello/state/events from a fixed FakeSession == files in PHONE/app/src/test/resources/api/
                                                    # (UPDATE_SAMPLES=1 writes them instead)
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`UPDATE_SAMPLES=1 python3 -m pytest test/test_phone_server.py -q` 寫出樣本，再 `python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**（含三個樣本檔）：`robot_dog_agent: HTTP server for the phone app`

### Task 4：`phone` 程式（`phone.py`）

**Files:**
- Create: `PKG/phone.py`
- Modify: `AGENT/setup.py`（console script `phone = robot_dog_agent.phone:main`）
- Test: `AGENT/test/test_phone.py`

**Interfaces:**
- Consumes: Task 2 `load_or_create`、`Guard`；Task 3 `PhoneServer`、`Sources`；`chat.usable_key`、`chat.build_agent`、`chat.DEFAULT_MODEL`、`chat.VISION_MODEL`、`chat.NO_KEY`、`chat.BAD_KEY`；`Session`；`FakeRobot`；`map_view.load_saved_map`；（非 `--no-robot`）`CameraFeed`、`MapFeed`、`Nav2Robot`、`ObjectMemory`、`GeminiClient`、`Eyes`。
- Produces:
  - `lan_address() -> str`：UDP socket `connect(('192.0.2.1', 9))` 的 `getsockname()[0]`（不會真的送封包）；失敗回 `'0.0.0.0'`。
  - `create(args, api_key) -> (server, session, feeds)`：組裝內容和 `gui.create` 相同（`--no-robot`：FakeRobot、`load_saved_map()` 版本 1、沒有相機、`can_survey=False`），但把 `session.on_event = server.on_event`，並在 `session.start(...)` 之前接好。
  - `main(argv=None) -> int`：參數 `--port`（預設 8765）、`--model`、`--vision-model`、`--no-robot`、`--new-code`；沒有 key 時印 `chat.NO_KEY`／`chat.BAD_KEY` 回 2；啟動後印 `手機連線：http://<lan_address>:<port>　配對碼 <code>`；Ctrl+C（SIGINT handler 設一個 Event）→ `session.shutdown()`、`server.close()`；不乾淨時 `os._exit(1)`；否則關 feeds、回 0。rclpy 用 `SignalHandlerOptions.NO` 初始化（和 `gui.create` 相同）。

- [ ] **Step 1：寫失敗的測試**

```python
def test_lan_address_is_an_ip():
def test_no_key_exits_2(capsys, monkeypatch):
def test_no_robot_server_works(tmp_path, monkeypatch):   # create(--no-robot, port 0, code file in tmp) → /api/hello ok,
                                                          # camera False, can_survey False, /api/map.png 200; shutdown clean
def test_module_does_not_import_ros_or_qt():              # importing phone leaves rclpy and PyQt5 out of sys.modules
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過；`colcon build --symlink-install --packages-select robot_dog_agent` 成功；`ros2 run robot_dog_agent phone --no-robot --port 8766` 印出連線行，`curl -s 127.0.0.1:8766/api/ping` 正確，Ctrl+C 結束（以 `timeout -s INT` 在前景跑）**
- [ ] **Step 5：Commit**：`robot_dog_agent: ros2 run robot_dog_agent phone`

### Task 5：Android 專案與純 Kotlin 部分

**Files:**
- Create: `PHONE/COLCON_IGNORE`、`PHONE/.gitignore`（`build/`、`.gradle/`、`local.properties`、`.idea/`、`*.iml`、`.kotlin/`）、`PHONE/settings.gradle.kts`、`PHONE/build.gradle.kts`、`PHONE/gradle.properties`、`PHONE/gradle/libs.versions.toml`、Gradle wrapper（`gradlew`、`gradle/wrapper/*`）、`PHONE/app/build.gradle.kts`、`PHONE/app/src/main/AndroidManifest.xml`、`PHONE/app/src/main/res/xml/network_security_config.xml`
- Create: `PHONE/app/src/main/java/com/robotdog/phone/Model.kt`、`MapGeometry.kt`、`Mjpeg.kt`、`ChatLog.kt`
- Test: `PHONE/app/src/test/java/com/robotdog/phone/ModelTest.kt`、`MapGeometryTest.kt`、`MjpegTest.kt`、`ChatLogTest.kt`

**版本（實作者照這個規則決定，寫進 ledger）：** AGP 取 `https://dl.google.com/android/maven2/com/android/tools/build/gradle/maven-metadata.xml` 上最新的穩定版；Gradle 用該版 AGP 要求的版本（下載 `gradle-<版本>-bin.zip` 到 `WS9`，執行一次 `gradle wrapper --gradle-version <版本>` 產生 wrapper）；Kotlin 和 Compose BOM 取和這版 AGP 相容的最新穩定版。compileSdk / targetSdk 37；若 AGP 不支援 37，加 `android.suppressUnsupportedCompileSdk=37`。JDK 用 `/usr/lib/jvm/java-21-openjdk-amd64`。

**Interfaces:**
- Produces（package `com.robotdog.phone`）：
  - `Model.kt`：`data class Place(id, name, short, x: Double, y: Double)`；`Hello(boot: String, places: List<Place>, canSurvey: Boolean, camera: Boolean)`；`Pose(x, y, yaw)`；`Point(x, y)`；`Thing(name, x, y)`；`MapInfo(version: Int, resolution, originX, originY, width: Int, height: Int)`；`RobotState(state, status, pose: Pose?, poseText, pathVersion: Int, path: List<Point>?, target: Point?, objects: List<Thing>, map: MapInfo?)`；`Event(id: Int, kind, text)`；`Events(boot, events: List<Event>, next: Int)`；`parseHello(String)`、`parseState(String)`、`parseEvents(String)`（`org.json`）。
  - `MapGeometry(info: MapInfo, viewWidth: Float, viewHeight: Float)`：等比縮放置中，`scale`（每格幾像素）、`left`、`top`；`toPixel(x, y): Pair<Float, Float>`；`toWorld(px, py): Point?`（地圖影像外回 `null`）。換算和 `map_view.MapView` 相同（y 朝上：格的第 0 列在畫面最下方）。
  - `Mjpeg.kt`：`class MjpegReader(input: InputStream)`，`next(): ByteArray?`：讀到 `--frame` 邊界後的標頭、照 `Content-Length` 讀那張 JPEG；串流結束回 `null`。
  - `ChatLog.kt`：`class ChatLog`，`lines: List<String>`（已加前綴的顯示文字；`state` 事件不顯示）；`apply(events: Events): Int`（回傳下次要用的 `after`）：`boot` 和目前不同（含第一次）就先清空；`prefix(kind)` 照 Global Constraints 的前綴。

- [ ] **Step 1：建專案骨架**（照上面的版本規則；Manifest：`INTERNET` 權限、`networkSecurityConfig` 允許明文、單一 `MainActivity`、`screenOrientation="portrait"`、`windowSoftInputMode="adjustResize"`；`MainActivity` 先只顯示一行字）；`./gradlew assembleDebug` 成功
- [ ] **Step 2：寫失敗的測試**

```kotlin
// ModelTest：讀 test resources 的 api/hello.json、state.json、events.json（Task 3 產生）
@Test fun parsesHello()            // 8 places, places[5].short == "廚房", boot not empty
@Test fun parsesState()            // state, status, pose, path, target, objects, map 全部對得上樣本
@Test fun parsesNullPoseAndMap()   // {"pose": null, "map": null, "path": null, ...} → pose == null, map == null, path == null
@Test fun parsesEvents()
// MapGeometryTest：MapInfo(1, 0.05, -5.0, -3.0, 200, 120)
@Test fun roundTrip()              // toWorld(toPixel(1.2, 0.5)) ≈ (1.2, 0.5)
@Test fun yIsUp()                  // toPixel(x, 2.0).second < toPixel(x, 0.0).second
@Test fun marginIsOutside()        // a tall view: a point above the image → null
@Test fun followsANewMapSize()     // same view, MapInfo width 300 → different scale; round trip still holds
// MjpegTest
@Test fun splitsParts()            // a hand-built stream of 3 parts → 3 byte arrays equal to the parts, then null
@Test fun truncatedStreamEnds()    // stream cut in the middle of a part → null, no exception
// ChatLogTest
@Test fun prefixesAndSkipsState()  // user/reply/error/log/state → "你：…", "狗：…", "[錯誤] …", "…", (none)
@Test fun continuesWithinABoot()   // apply(boot a, ids 1-2) → next 2; apply(boot a, id 3) → 3 lines
@Test fun bootChangeClearsTheLog() // apply(boot a, 3 events) then apply(boot b, ids 1-2) → only the 2 new lines
```

- [ ] **Step 3：跑 `./gradlew test` 確認失敗** → **Step 4：實作** → **Step 5：`./gradlew test` 全部通過**
- [ ] **Step 6：Commit**（`git add robot_dog_phone` 前先 `git status` 確認沒有 `build/`、`local.properties`）：`robot_dog_phone: Android project, models, map geometry, MJPEG, chat log`

### Task 6：連線（`Api.kt`、`RobotViewModel.kt`）

**Files:**
- Create: `PHONE/app/src/main/java/com/robotdog/phone/Api.kt`、`RobotViewModel.kt`、`Settings.kt`
- Test: `PHONE/app/src/test/java/com/robotdog/phone/ApiTest.kt`

**Interfaces:**
- Consumes: Task 5 全部。
- Produces:
  - `sealed class ApiError : Exception`：`Unreachable`、`NotOurServer`、`BadCode`（401）、`Locked`（429）、`Http(code: Int)`。
  - `class Api(address: String, code: String)`（`address` 是 `host:port`）：`ping()`、`hello(): Hello`、`state(pathVersion: Int): RobotState`、`events(after: Int): Events`（讀取逾時 30 秒）、`command(text): Boolean`、`goTo(x, y): Boolean`、`survey(): Boolean`、`stop()`、`mapPng(): ByteArray`、`camera(): MjpegReader`（呼叫者負責關）。全部是 `suspend`、在 `Dispatchers.IO` 執行；連線逾時 3 秒；header `Authorization: Bearer <code>`。
  - `Settings(context)`：`address`（預設 `192.168.0.154:8765`）、`code`，存 SharedPreferences。
  - `RobotViewModel`：Compose 狀態 `connection`（`Connected` / `Retrying` / `NeedsSetup`）、`hello`、`state`、`mapImage: ImageBitmap?`、`frame: ImageBitmap?`、`chat: List<String>`、`pendingTarget: Point?`；方法 `connect(address, code): ApiError?`（先 `ping` 再 `hello`，成功才存設定）、`send(text)`、`goToPlace(place)`（送 `去<short>`，和電腦視窗相同）、`tapMap(point)`（待命時設 `pendingTarget`）、`confirmTarget()`、`cancelTarget()`、`survey()`、`stop()`、`resume()`／`pause()`（前景才跑三個迴圈：每 0.2 秒 state、長輪詢 events、相機；任何一個連不上就 `Retrying`、每 2 秒重試；地圖版本變了重抓 PNG）。路線：`state.path == null` 表示沒變，沿用上一份。

- [ ] **Step 1：寫失敗的測試**（JVM 上用 `com.sun.net.httpserver.HttpServer` 當假伺服器）

```kotlin
@Test fun sendsTheCode()              // fake server sees "Bearer 123456"
@Test fun mapsErrors()                // 401 → BadCode, 429 → Locked, 500 → Http(500), nothing listening → Unreachable,
                                      // ping answers {"app": "other"} → NotOurServer
@Test fun commandReturnsOk()          // {"ok": false} → false
```

- [ ] **Step 2：確認失敗** → **Step 3：實作** → **Step 4：`./gradlew test` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_phone: API client, settings and view model`

### Task 7：畫面，並在模擬器上對 `--no-robot` 伺服器跑通

**Files:**
- Create: `PHONE/app/src/main/java/com/robotdog/phone/MainScreen.kt`、`MapCanvas.kt`、`SettingsScreen.kt`
- Modify: `PHONE/app/src/main/java/com/robotdog/phone/MainActivity.kt`
- Create: `WS9/bin/emu.sh`（`start`：`emulator -avd Medium_Phone_API_37.0 -no-snapshot-save -no-audio` 背景啟動並等 `sys.boot_completed`；`stop`；`shot <檔名>`：`adb exec-out screencap -p`；`tap <testTag>`：`uiautomator dump` 找 resource-id 再 `input tap`）

**Interfaces:**
- Consumes: Task 6 `RobotViewModel`、`Settings`；Task 5 `MapGeometry`。
- Produces：版面照 spec 第 5.1 節的草圖；設定頁照 5.2 節（三種錯誤分開講：`連不上電腦`、`這個位址不是機器狗的伺服器`、`配對碼錯誤`，429 為 `錯太多次，請一分鐘後再試`）。
  - 地圖畫法和 `map_widget.py` 相同（顏色 `#1565c0` 狗、`#2e7d32` 路線、`#d32f2f` 目標、`#6d4c41` 地點、`#ef6c00` 物件；待確認的目標是灰色 ✕）；沒有地圖時顯示 `等地圖…（nav.launch.py 開了嗎？）`；沒有相機時顯示 `沒有相機畫面`。
  - 大畫面預設地圖，右下角小窗是另一個，點小窗互換。
  - `hello.canSurvey` 為 false 時不顯示巡邏按鈕（和電腦視窗相同）；`hello.camera` 為 false 時小窗顯示 `沒有相機畫面`。
  - `MainActivity`：`FLAG_KEEP_SCREEN_ON`；`onStart` → `resume()`、`onStop` → `pause()`；根 Modifier 設 `testTagsAsResourceId = true`。testTag：`address`、`code`、`connect`、`map`、`camera`、`place_<id>`、`survey`、`input`、`send`、`stop`、`go_here`、`cancel_target`、`settings`、`status`。

- [ ] **Step 1：實作畫面**；`./gradlew test assembleDebug` 成功
- [ ] **Step 2：模擬器跑通**：前景 `ros2 run robot_dog_agent phone --no-robot`（另一個終端或背景程序，記下配對碼）；`emu.sh start`；`adb install -r app/build/outputs/apk/debug/app-debug.apk`；在設定頁輸入 `10.0.2.2:8765` 和配對碼 → 主畫面出現地圖和 FakeRobot 的狗；點地圖空地 → 灰色 ✕ 和「去這裡」→ 按「去」→ 狗到那裡、對話出現 `[地圖] 去 (…)` 和工具紀錄；打錯配對碼 → `配對碼錯誤`；Ctrl+C 伺服器 → 狀態列 `連不上電腦，重試中…` → 重開伺服器 → 自動接上、對話清空重來。每一步截圖到 `WS9/shots/`，自己看過畫面沒有跑版、字沒有被切掉、鍵盤打開時輸入列在鍵盤上方。
- [ ] **Step 3：Commit**：`robot_dog_phone: main screen, map, settings`

### Task 8：模擬器實測、README、spec 結果

**Files:**
- Create: `WS9/bin/live_check.sh`（用 `emu.sh` 操作 app、用 `/odom_gt` 判定；Gemini 請求數從伺服器終端機輸出的工具紀錄和 `[地圖]` 行判斷）
- Create: `PHONE/README.md`
- Modify: `AGENT/README.md`（`phone` 一節：啟動、配對碼、`--new-code`、防火牆那一行 `sudo ufw allow from 192.168.0.0/24 to any port 8765 proto tcp` 由使用者自己執行、和 `gui` 二選一）、根目錄 `README.md`（一行介紹手機 app）
- Modify: `docs/superpowers/specs/2026-11-19-phone-app-design.md`（加「結果」一節：測試數、6 項實測的數字、和設計不同的地方）

- [ ] **Step 1：實測 spec 第 8 節的 6 項**：確認使用者沒有開 Isaac Sim → `stack.sh`（開相機、定位模式）→ `phone` → 模擬器 app 連 `10.0.2.2:8765` → 逐項跑、記下數字到 ledger。截兩張圖：導航途中的地圖大畫面、相機大畫面。結束後 `stack.sh stop`、`emu.sh stop`。
- [ ] **Step 2：寫 README 和 spec 結果**；`PHONE/README.md` 寫怎麼用 Android Studio 開專案、怎麼 `adb install` 到自己的手機、怎麼在設定頁填位址和配對碼。
- [ ] **Step 3：`python3 -m pytest test -q` 通過；`./gradlew test assembleDebug` 通過；key 外洩檢查；Commit**：`robot_dog_phone: verified live on the emulator, README`
