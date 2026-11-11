# 計畫 6：機器狗對話視窗（含相機畫面）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `ros2 run robot_dog_agent gui` 開一個 PyQt5 視窗：相機畫面、對話、八個地點按鈕、停止按鈕、狀態列；對話流程沿用現有的 Agent。

**Architecture:** 純 Python 的 `Session` 在背景執行緒跑 `Agent.ask` 並往外送事件；視窗只在 Qt 主執行緒更新，用計時器拉最新的相機影像和位置。停止靠機器人介面新增的 `cancel()`（一個 `threading.Event`）。相機是獨立的 rclpy 節點，只留最新一張影像。

**Tech Stack:** Python 3.12、PyQt5 5.15（系統已裝）、numpy、rclpy（Jazzy）、pytest（Qt 用 offscreen 平台）

**Spec:** `docs/superpowers/specs/2026-11-11-agent-gui-design.md`（對話流程本身見 `2026-11-10-agent-nav-chat-design.md`）

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`AGENT` = `~/dog_12dof_ws/robot_dog_agent`，`PKG` = `AGENT/robot_dog_agent`，`WS6` = `~/dog_12dof_ws/.superpowers/sdd/2026-11-11-plan6-agent-gui`（拋棄式腳本和 log）。
- 測試：`cd $AGENT && python3 -m pytest test -q`，不 source ROS、不需要網路。原有 68 個測試必須仍然通過。視窗測試在匯入 PyQt5 之前設 `QT_QPA_PLATFORM=offscreen`。
- `session.py`、`frames.py` 不可匯入 rclpy、requests、PyQt5。`gui.py` 在模組層不可匯入 rclpy（`camera_feed`、`nav2_robot` 在 `main` 裡、非 `--no-robot` 時才匯入）。
- 不安裝新套件。不改 `robot_dog_description`、`robot_dog_navigation`、Nav2 參數、地圖。終端機版 `chat` 和 `agent_eval` 的行為不變。
- 門檻（照 spec）：影像計時器 100 ms、狀態計時器 500 ms、影像過期 3.0 s、`shutdown` 等指令執行緒最多 10 s。
- API key：`. ~/dog_12dof_ws/.superpowers/sdd/2026-11-10-plan5-agent-nav-chat/bin/key.sh` 會從使用者的 `~/.bashrc` 載入 `GEMINI_API_KEY`（使用者已同意這個做法）。key 不可被印出、不可進 repo；commit 前跑 `grep -rlF "$GEMINI_API_KEY" $AGENT docs README.md`，必須沒有輸出。
- 模擬器：從計畫 5 的工作區複製 `stack.sh` 到 `WS6/bin/`，把模擬器那一行的 `--no-camera` 拿掉（這次要相機）。**呼叫 `stack.sh` 的那個 Bash 指令裡不能出現它的比對字樣**（launch 檔名、container 名稱等），否則 `stop` 會把自己的 shell 殺掉。同一時間只跑一個 Isaac Sim；啟動前確認使用者沒有開著。
- `.vscode/settings.json` 和 `robot_dog_navigation/rviz/nav.rviz` 是使用者的修改，不帶進任何 commit（`git commit -- <本計畫的路徑>`）。
- 程式註解英文；印給使用者看的文字、文件用繁體中文。commit 結尾 `Co-Authored-By: Claude Fable 5.2 <noreply@anthropic.com>`。

## Review Focus

1. **還在等 Nav2 時就關視窗**：建立 `Nav2Robot` 的執行緒卡在等待、TF 執行緒不是 daemon。期待行程照樣結束（Task 2 `test_shutdown_reports_a_job_that_will_not_end`；Task 4 `main` 在 `shutdown()` 回 `False` 時用 `os._exit`）。
2. **視窗關掉之後才到的事件**：指令執行緒晚一步結束。期待不當掉（Task 4 `test_events_after_close_are_ignored`）。
3. **停止按在不對的時候**：待命時、連按兩次、還沒就緒時。期待沒有多餘的 `cancel()`，下一個指令不受影響（Task 2 `test_stop_only_cancels_while_busy`、`test_a_stale_cancel_does_not_hit_the_next_command`）。
4. **影像內容不對**：格式不支援、資料太短、解析度中途改變。期待顯示提示文字或新的尺寸，不當掉（Task 3 `test_bad_frames_give_none`；Task 4 `test_a_frame_that_cannot_be_shown_is_skipped`）。
5. **啟動失敗後使用者照樣打字**：期待輸入一直鎖著、對話區有原因（Task 2 `test_start_failure`；Task 4 `test_widgets_follow_the_state`）。

---

### Task 1：機器人介面的取消、不等待的位置、進度回呼

**Files:**
- Modify: `PKG/robot.py`、`PKG/nav2_robot.py`、`PKG/chat.py`
- Test: `AGENT/test/test_tools.py`、`AGENT/test/test_chat.py`

**Interfaces:**
- Produces（機器人介面新增，`FakeRobot` 和 `Nav2Robot` 都實作）：
  - `cancel()`：可從任何執行緒呼叫。導航中 → 該次 `go_to` 回 `NavResult('canceled', 'canceled by the user', t)`；沒有導航 → 下一次 `go_to` 不送目標、直接回同樣的結果（`seconds` 0.0）。一次 `cancel()` 只生效一次。
  - `clear_cancel()`：忘掉還沒用到的取消。
  - `pose(wait=...)`：`Nav2Robot.pose(wait=0)` 只查一次 TF，沒有就立刻丟 `RuntimeError`；`FakeRobot.pose(wait=None)` 忽略參數。
  - `close()`：`FakeRobot.close()` 設 `self.closed = True`。
- Produces（`chat.py`）：`build_agent(robot, api_key, model, say=print, on_call=None, on_progress=None)`：給了 `on_progress` 就把距離原樣交給它，不再印「剩 … m」；沒給時行為不變。
- `Nav2Robot`：`__init__` 只在 `not rclpy.ok()` 時呼叫 `rclpy.init`；取消用 `threading.Event`，在 `_wait` 送出目標之前和等待迴圈裡各檢查一次（迴圈裡看到就 `cancelTask()` 並照 Ctrl+C 的方式等任務結束）。`KeyboardInterrupt` 的路徑不變。

- [ ] **Step 1：寫失敗的測試**

```python
# test_tools.py
def test_fake_robot_cancel_stops_the_next_navigation_once():
    robot = FakeRobot()
    robot.cancel()
    assert robot.go_to(1.0, 0.0, 0.0) == NavResult('canceled', 'canceled by the user', 0.0)
    assert robot.goals == [] and robot.pose() == (0.0, 0.0, 0.0)
    assert robot.go_to(1.0, 0.0, 0.0).status == 'reached'            # one cancel, one navigation

def test_fake_robot_clear_cancel():
    robot = FakeRobot()
    robot.cancel(); robot.clear_cancel()
    assert robot.go_to(1.0, 0.0, 0.0).status == 'reached'

def test_fake_robot_pose_takes_a_wait_and_close_is_recorded():
    robot = FakeRobot((1.0, 2.0, 0.5))
    assert robot.pose(wait=0) == (1.0, 2.0, 0.5) and robot.closed is False
    robot.close()
    assert robot.closed is True

def test_a_canceled_navigation_is_reported_by_the_toolbox():
    robot = FakeRobot(); robot.cancel()
    r = box(robot).call('go_to_place', {'place': 'kitchen'})
    assert r['status'] == 'canceled' and (r['x'], r['y']) == (0.0, 0.0)

# test_chat.py
def test_build_agent_hands_progress_to_a_callback():
    said, seen = [], []
    agent = chat.build_agent(FakeRobot(), 'k', 'm', say=said.append, on_progress=seen.append)
    agent.toolbox.on_progress(1.234); agent.toolbox.on_progress(0.0)
    assert seen == [1.234, 0.0] and said == []
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作三個檔案** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: cancel from another thread, pose without waiting, progress callback`

### Task 2：Session（`session.py`）

**Files:**
- Create: `PKG/session.py`
- Test: `AGENT/test/test_session.py`

**Interfaces:**
- Consumes: 機器人介面（Task 1）；`Agent.ask(text) -> str`；`GeminiError`（只用類別名稱比對，不匯入 `gemini`：`type(e).__name__ == 'GeminiError'`）；`places.area_of`。
- Produces:
  - `class Session(run=None)`：`run(fn) -> job`，`job` 有 `join(timeout)` 和 `is_alive()`；預設開 daemon 執行緒。
  - 屬性：`on_event`（預設什麼都不做，`(kind: str, text: str)`）、`state`（`'starting'`、`'idle'`、`'busy'`、`'failed'`）、`remaining`（`float | None`）、`robot`（就緒前是 `None`）。
  - `start(make_robot, make_agent)`：背景執行 `robot = make_robot()`，再 `agent = make_agent(robot, say=<送 log 事件>, on_call=<清掉 remaining>, on_progress=<設 remaining>)`。成功 → `idle` 並送 `('log', READY)`；`RuntimeError` → `('error', NOT_LOCALIZED % e)`、`failed`；其他例外 → `('error', INTERNAL % 類別名稱)`、`failed`。
  - `submit(text) -> bool`、`stop()`、`shutdown(timeout=10.0) -> bool`（`False` 表示背景工作在期限內沒有結束）、`pose() -> (x, y, yaw) | None`（`robot.pose(wait=0)`，`RuntimeError` 或還沒有機器人時回 `None`）、`status_text() -> str`。
  - `pose_text(pose) -> str`：`'位置 (2.48, -0.61) 朝向 -90°　客餐廚'`；`None` → `'位置未知'`。
  - 常數：`READY = '可以開始了：打字、或按下面的地點按鈕。'`、`NOT_LOCALIZED = 'Nav2 啟動了，但機器狗還沒有定位（%s）。請確認 nav.launch.py 正常執行。'`、`INTERNAL = '程式內部錯誤（%s），這一句沒有完成。'`、`AREA_ZH = {'living, dining and kitchen area': '客餐廚', 'master bedroom': '主臥室', 'second bedroom': '次臥室', 'bathroom': '浴室'}`。
  - `status_text()`：`starting` → `'等 Nav2 啟動…'`；`idle` → `'待命'`；`busy` 且 `remaining is None` → `'處理中…'`；`busy` 且有距離 → `'導航中，剩 1.2 m'`；`failed` → `'啟動失敗'`。
- 事件：每次狀態改變送 `('state', 新狀態)`。一個指令的順序是 `state busy → user → （log …）→ reply 或 error → state idle`。`submit` 一開始呼叫 `robot.clear_cancel()`。`ask` 丟 `GeminiError` → `('error', str(e))`；其他例外 → `('error', INTERNAL % 類別名稱)`；`KeyboardInterrupt` 視同其他例外。

- [ ] **Step 1：寫失敗的測試** `test_session.py`。輔助：`sync = lambda fn: (fn(), DoneJob())[1]`（當場執行）；`FakeAgent(answers)`（同 `test_chat.py` 的寫法）；`started(answers=(), robot=None)` 回傳 `(session, events, robot, agent)`，已用 `sync` 完成 `start`、`events` 是收集到的 `(kind, text)`，並在回傳前清空。

```python
def test_start_makes_the_session_idle():          # events == [('state', 'idle'), ('log', READY)]; session.robot is the robot
def test_start_failure():                         # make_robot raises RuntimeError('no transform') -> state 'failed',
                                                  # events hold ('error', NOT_LOCALIZED % 'no transform'); submit('去廚房') is False
def test_start_failure_of_another_kind():         # ValueError -> ('error', INTERNAL % 'ValueError'), state 'failed'
def test_submit_runs_one_command():
    # answers ['到了']: submit('去廚房') is True; events == [('state','busy'), ('user','去廚房'), ('reply','到了'), ('state','idle')]
def test_blank_and_busy_submits_are_refused():    # '' and '  ' -> False, nothing asked; with a run that only stores fn:
                                                  # first submit True, second False while state == 'busy'
def test_not_ready_submits_are_refused():         # Session(sync) before start: submit('去廚房') is False
def test_gemini_error_and_other_errors_end_the_command():
    # GeminiError('被限流') -> ('error', '被限流'); ValueError('x') -> ('error', INTERNAL % 'ValueError'); state back to 'idle' both times
def test_tool_lines_and_notices_are_log_events():  # the say given to make_agent, called with '  [工具] …' -> ('log', '  [工具] …')
def test_progress_sets_remaining_and_a_finished_tool_clears_it():
    # on_progress(1.23) -> remaining == 1.23, status_text() == '導航中，剩 1.2 m' while busy; on_call(call, result) -> remaining None
def test_status_text_for_every_state():
def test_stop_only_cancels_while_busy():          # idle: stop() -> robot.cancels == 0; busy (stored fn): stop() twice -> 2 calls, no error
def test_a_stale_cancel_does_not_hit_the_next_command():
    # robot.cancel() while idle, then submit: the agent's go_to (FakeAgent that calls robot.go_to) is 'reached'
def test_pose_and_pose_text():
    # FakeRobot((2.48, -0.61, -pi/2)): session.pose() == that; pose_text(...) == '位置 (2.48, -0.61) 朝向 -90°　客餐廚'
    # pose_error set -> session.pose() is None; pose_text(None) == '位置未知'; before start -> None
def test_shutdown_cancels_and_closes_the_robot():  # idle session: shutdown() is True, robot.closed, robot.cancels == 1
def test_shutdown_reports_a_job_that_will_not_end():
    # run returns a job whose is_alive() stays True: shutdown(timeout=0.05) is False and robot.close was not called
def test_shutdown_before_the_robot_exists():       # Session(sync) never started: shutdown() is True, no error
```

（測試用的 `FakeRobot` 子類別多記一個 `cancels` 計數。）

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `session.py`** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: Session, one command at a time off the UI thread`

### Task 3：影像轉換與相機訂閱（`frames.py`、`camera_feed.py`）

**Files:**
- Create: `PKG/frames.py`、`PKG/camera_feed.py`
- Modify: `AGENT/package.xml`（`exec_depend`：`sensor_msgs`、`python3-pyqt5`）
- Test: `AGENT/test/test_frames.py`

**Interfaces:**
- Produces（`frames.py`）：`to_rgb(encoding: str, width: int, height: int, step: int, data: bytes) -> bytes | None`：回傳長度 `width × height × 3` 的 RGB 位元組（每列沒有填充）。支援 `rgb8`、`bgr8`、`rgba8`、`bgra8`、`mono8`；`step` 可以大於 `width × 通道數`；格式不支援、`width`/`height` ≤ 0、`step` 太小、資料長度小於 `step × height` 時回 `None`。
- Produces（`camera_feed.py`，匯入 rclpy）：`class CameraFeed(topic='/camera/image_raw')`：需要 rclpy 已初始化；建立節點 `robot_dog_agent_camera`（`use_sim_time` 為真），用 `qos_profile_sensor_data` 訂閱，自己的 `SingleThreadedExecutor` 在一條 daemon 執行緒裡 spin。`latest() -> (width, height, rgb_bytes, time.monotonic() 的收到時間) | None`（有鎖；`to_rgb` 回 `None` 的影像不存）。`close()`：停 executor、銷毀節點。

- [ ] **Step 1：寫失敗的測試** `test_frames.py`

```python
RED, GREEN = bytes([255, 0, 0]), bytes([0, 255, 0])

def test_rgb8_passes_through():        # 2x1: to_rgb('rgb8', 2, 1, 6, RED + GREEN) == RED + GREEN
def test_bgr8_is_swapped():            # to_rgb('bgr8', 1, 1, 3, bytes([0, 0, 255])) == RED
def test_alpha_is_dropped():           # rgba8 [255,0,0,9] -> RED; bgra8 [0,0,255,9] -> RED
def test_mono8_becomes_grey():         # to_rgb('mono8', 2, 1, 2, bytes([0, 200])) == bytes([0,0,0, 200,200,200])
def test_row_padding_is_removed():     # rgb8 2x2 with step 8 (2 padding bytes per row) -> the 12 pixel bytes only
def test_bad_frames_give_none():
    for args in [('yuv422', 1, 1, 2, b'ab'), ('rgb8', 2, 1, 6, RED), ('rgb8', 0, 1, 0, b''), ('rgb8', 2, 1, 5, RED + GREEN),
                 ('rgb8', -1, 1, 3, RED)]:
        assert to_rgb(*args) is None, args
def test_the_result_is_bytes_of_the_right_length():   # 640x480 rgb8 zeros -> bytes, len 921600
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `frames.py`（numpy reshape/切片）和 `camera_feed.py`** → **Step 4：全部測試通過；`source /opt/ros/jazzy/setup.bash` 後 `python3 -c "import robot_dog_agent.camera_feed"` 成功（在 `AGENT` 目錄下）**
- [ ] **Step 5：Commit**：`robot_dog_agent: camera frames as RGB bytes, CameraFeed keeps the latest one`

### Task 4：視窗（`gui.py`）

**Files:**
- Create: `PKG/gui.py`
- Modify: `AGENT/setup.py`（`gui = robot_dog_agent.gui:main`）
- Test: `AGENT/test/test_gui.py`

**Interfaces:**
- Consumes: `Session`（Task 2）的 `submit`、`stop`、`shutdown`、`state`、`status_text()`、`pose()`、`on_event`；`session.pose_text`；`places.PLACES`；`chat.usable_key`、`chat.NO_KEY`、`chat.BAD_KEY`、`chat.DEFAULT_MODEL`、`chat.build_agent`；`CameraFeed.latest()`（Task 3）。
- Produces:
  - `class MainWindow(QMainWindow)`，`MainWindow(session, latest_frame=None, no_camera_text=NO_CAMERA, clock=time.monotonic)`：`latest_frame()` 回 `CameraFeed.latest()` 的格式或 `None`；`None` 的 `latest_frame` 表示沒有相機。
  - 元件屬性（測試用）：`image_label`（QLabel）、`log`（QPlainTextEdit，唯讀）、`input`（QLineEdit）、`send_button`、`stop_button`、`place_buttons`（list，順序同 `PLACES`）、`status_left`、`status_right`（狀態列裡的 QLabel）。
  - signal `events = pyqtSignal(str, str)`，連到 `on_event(kind, text)`；`main` 把 `session.on_event` 設成 `window.events.emit`。
  - `refresh_frame()`、`refresh_status()`：兩個計時器呼叫的函式，測試直接呼叫。
  - `create(args, api_key) -> (window, session, feed_or_None)`：組起全部物件並 `session.start(...)`；`main` 和實機檢查的驅動腳本都用它。
  - `main(argv=None) -> int`：`--model`（預設 `chat.DEFAULT_MODEL`）、`--no-robot`。
  - 常數：`NO_CAMERA = '沒有相機畫面（Isaac Sim 是不是加了 --no-camera？）'`、`NO_ROBOT_CAMERA = '--no-robot：沒有相機'`、`FRAME_PERIOD_MS = 100`、`STATUS_PERIOD_MS = 500`、`STALE_AFTER = 3.0`。

行為（spec 第 2 節的表格是依據）：
- `on_event`：`user` → `你：<text>`；`reply` → `狗：<text>`；`error` → `[錯誤] <text>`；`log` → 原樣；`state` → 套用鎖定規則並更新狀態列。視窗關閉後收到的事件直接丟掉。
- 鎖定規則：`idle` → 輸入、送出、地點按鈕可用，停止不可用；`busy` → 相反；`starting`、`failed` → 全部不可用。`idle` 時把游標放回輸入框。
- 送出：`input` 的 Enter 和 `send_button` 都呼叫同一個函式：`session.submit(文字)` 回 `True` 才清空輸入框。地點按鈕：文字是 `place.name.split('（')[0]`，按下 `session.submit('去' + 文字)`。
- `refresh_frame`：`latest_frame()` 有值且 `clock() - 收到時間 <= STALE_AFTER` → 包成 `QImage(Format_RGB888)`（先 `.copy()`，位元組的生命週期才不成問題）、依 `image_label` 目前的大小保持比例縮放後顯示；長度和 `width × height × 3` 不符的影像跳過；否則顯示 `no_camera_text`。同一張影像（收到時間沒變）不重畫。
- `refresh_status`：`status_left` = `pose_text(session.pose())`，`status_right` = `session.status_text()`。
- `closeEvent`：停計時器，標記已關閉，呼叫 `session.shutdown()` 並把結果存在 `self.clean_shutdown`。
- `main`：key 檢查和 `chat.main` 相同（訊息印在終端機，回 2，不建立 `QApplication`）。`--no-robot`：`make_robot=FakeRobot`、沒有 `CameraFeed`、`no_camera_text=NO_ROBOT_CAMERA`。否則先 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`，建立 `CameraFeed`，`make_robot=Nav2Robot`。`SIGINT` 的處理函式呼叫 `window.close()`（計時器讓 Python 有機會執行它）。`app.exec_()` 結束後：`feed.close()`；`window.clean_shutdown` 為 `False` 時 `os._exit(1)`（還卡在等 Nav2 的執行緒收不掉），否則回 0。

- [ ] **Step 1：寫失敗的測試** `test_gui.py`（檔案開頭先 `os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')` 再匯入 PyQt5；模組層級只建立一個 `QApplication`。`FakeSession`：記下 `submitted`、`stops`、`shutdowns`，`state` 可設定，`submit` 回傳可設定的 `accept`，`pose()` 回傳可設定的值，`status_text()` 回 `'待命'`。`window(session=None, **kw)` 建立 `MainWindow` 並先送一次 `('state', session.state)`。）

```python
def test_place_buttons_send_go_commands():
    # texts == ['起點', '客廳西側', '主臥室', '次臥室', '餐桌旁', '廚房', '玄關', '浴室門口']; clicking each in turn ->
    # session.submitted == ['去起點', '去客廳西側', …, '去浴室門口']
def test_enter_and_the_send_button_submit_and_clear():   # '去廚房' + returnPressed, '你在哪' + click: both submitted, input empty after each
def test_a_refused_submit_keeps_the_text():             # session.accept = False: the input still holds the text
def test_widgets_follow_the_state():
    # idle: input/send/all place buttons enabled, stop disabled; busy: the reverse; starting and failed: everything disabled
def test_events_become_lines_in_the_log():
    # user '去廚房', log '  [工具] x', reply '到了', error '被限流' -> log text == '你：去廚房\n  [工具] x\n狗：到了\n[錯誤] 被限流'
def test_stop_button_calls_stop():                       # busy: click -> session.stops == 1
def test_a_fresh_frame_is_shown():                       # latest_frame -> (2, 1, RED + GREEN, 100.0), clock -> 101.0:
                                                         # image_label.pixmap() is not null and image_label.text() == ''
def test_a_stale_or_missing_frame_shows_the_hint():      # clock -> 104.0 (older than 3 s) -> text == NO_CAMERA, no pixmap;
                                                         # latest_frame returning None -> the same; latest_frame=None -> the same
def test_a_frame_that_cannot_be_shown_is_skipped():      # (2, 1, RED, 100.0): no exception, the hint is shown
def test_status_bar():                                   # session.pose() -> (2.48, -0.61, -pi/2): status_left.text() ==
                                                         # '位置 (2.48, -0.61) 朝向 -90°　客餐廚', status_right.text() == '待命'
def test_closing_shuts_the_session_down():               # window.close() -> session.shutdowns == 1, window.clean_shutdown is True
def test_events_after_close_are_ignored():               # after close(): on_event('reply', '晚到') does not raise, log unchanged
def test_main_without_a_key(monkeypatch, capsys):        # GEMINI_API_KEY unset: gui.main(['--no-robot']) == 2, output has GEMINI_API_KEY
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `gui.py`**（版面照 spec 第 2 節：左欄影像、兩列地點按鈕、停止；右欄對話和輸入列；最小視窗 1000×600；停止按鈕用紅底白字）→ **Step 4：`python3 -m pytest test -q` 全部通過；`colcon build --symlink-install --packages-select robot_dog_agent` 成功**
- [ ] **Step 5：不開模擬器看一次**：`QT_QPA_PLATFORM=offscreen` 下用一支 `WS6/bin/shot_norobot.py`（拋棄式）呼叫 `gui.create(--no-robot)`，送出「去廚房」，等回覆（最多 60 秒，期間 `app.processEvents()`），`window.grab().save(WS6/runs/gui_norobot.png)`。預期對話區有 `你：去廚房`、`[工具] go_to_place … → reached`、`狗：…`，狀態列是 `位置 (2.50, -0.60) 朝向 -90°　客餐廚` 和 `待命`。用 Read 看截圖確認版面沒有跑掉。
- [ ] **Step 6：Commit**：`robot_dog_agent: the chat window with the camera image`

### Task 5：實機檢查與文件

**Files:**
- Create: `WS6/bin/stack.sh`（複製並拿掉 `--no-camera`）、`WS6/bin/gui_live_check.py`（拋棄式）
- Modify: `AGENT/README.md`、`~/dog_12dof_ws/README.md`、spec（加「結果」一節）

- [ ] **Step 1：寫驅動腳本** `gui_live_check.py`：`QT_QPA_PLATFORM=offscreen`；`gui.create`（真的 `Nav2Robot` 和 `CameraFeed`）；另開一個節點訂閱 `/odom_gt`（自己的 executor 執行緒）；用 `app.processEvents()` 加短暫 `sleep` 的迴圈推進，依序做 spec 第 7 節的五項，每項印一行 `[GUI] <n> PASS|FAIL <量到的數字>`，最後 `[GUI] PASS k/5`。第 2 項途中存兩張 `image_label` 的截圖比對是否不同；結束前存 `WS6/runs/gui_live.png`（整個視窗，導航途中截）。第 5 項由外層的 shell 判定：腳本結束碼和結束時間，加上 `ros2 action info /navigate_to_pose` 之類的方式或 launch log 最後一個目標的狀態，確認沒有進行中的目標。
- [ ] **Step 2：跑實機檢查**：確認沒有其他 Isaac Sim；`WS6/bin/stack.sh start False`；載入 key；執行驅動腳本，輸出存 `WS6/runs/gui_live_1.log`。需要 `[GUI] PASS 5/5`。不過時依原因修正（程式錯誤走 TDD：先補一個會失敗的測試），修完整輪重跑。用 Read 看 `gui_live.png`：相機畫面是公寓內部、不是提示文字。
- [ ] **Step 3：結束碼的兩個情況**（隔離的 ROS domain、假的 key、不連網，做法同計畫 5 的 `startup_fail_check.sh`）：Nav2 沒開時啟動 `gui`（offscreen），5 秒後對它送 `SIGINT` → 15 秒內結束、沒有 traceback；同樣情況改成由腳本呼叫 `window.close()` → 15 秒內結束。
- [ ] **Step 4：`stack.sh stop`；寫文件**。`AGENT/README.md` 加「視窗版」一節：啟動指令（模擬器不能加 `--no-camera`）、畫面各部分的說明、停止按鈕的兩種情況、相機沒畫面時怎麼辦、`--no-robot`、限制（一次一個指令、地點按鈕也用額度、開相機後模擬較慢）；檔案表加四個新檔。根目錄 `README.md` 的「用中文指揮機器狗」加 `gui` 指令。spec 加「結果」一節（實測數字、和設計不同的地方）。把 `gui_live.png` 用 SendUserFile 傳給使用者。
- [ ] **Step 5：`cd $AGENT && python3 -m pytest test -q` 通過；key 外洩檢查；Commit**：`robot_dog_agent: window verified live, README`
