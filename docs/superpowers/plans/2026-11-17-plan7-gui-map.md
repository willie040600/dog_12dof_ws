# 計畫 7：視窗加上地圖（顯示位置與路徑、點地圖叫狗過去）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 對話視窗右欄上方多一張地圖：畫出地圖、八個地點、狗的位置與朝向、導航路徑；在地圖上點一下，狗就直接（不經 Gemini）走過去。

**Architecture:** 純 Python 的 `map_view.py` 負責佔據格轉灰階和座標換算；`map_feed.py` 用自己的 rclpy 節點收 `/map` 和 `/plan`；`map_widget.py`（PyQt5）自己畫圖並把點擊換算成地圖座標發出 signal；`Session.go_to` 在背景直接呼叫現有工具 `go_to_pose`，沿用同一套狀態、停止與關閉流程。

**Tech Stack:** Python 3.12、PyQt5 5.15、numpy、rclpy（Jazzy，`nav_msgs`）、pytest（Qt offscreen）

**Spec:** `docs/superpowers/specs/2026-11-17-agent-gui-map-design.md`（視窗本身見 `2026-11-11-agent-gui-design.md`）

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`AGENT` = `~/dog_12dof_ws/robot_dog_agent`，`PKG` = `AGENT/robot_dog_agent`，`WS7` = `~/dog_12dof_ws/.superpowers/sdd/2026-11-17-plan7-gui-map`（拋棄式腳本和 log）。
- 測試：`cd $AGENT && python3 -m pytest test -q`，不 source ROS、不需要網路。原有 120 個測試必須仍然通過。視窗測試在匯入 PyQt5 之前設 `QT_QPA_PLATFORM=offscreen`。
- `map_view.py`、`session.py`、`tools.py` 不可匯入 rclpy、requests、PyQt5。`map_widget.py` 不可匯入 rclpy。`gui.py` 在模組層不可匯入 rclpy（`map_feed` 和 `camera_feed` 一樣只在 `create` 的非 `--no-robot` 分支匯入）。
- 不安裝新套件；`nav_msgs` 已在 `package.xml`。不改 `robot_dog_description`、`robot_dog_navigation`、Nav2 參數、地圖檔。終端機版 `chat`、`agent_eval` 的行為不變。
- 照 spec 的值：視窗最小 1100×700；右欄地圖和對話區高度比 3:2；被拒絕或結束後目標 ✕ 保留 2.0 秒；點地圖不送任何 Gemini 請求。
- 地圖顏色（我的決定，spec 第 7 節交給實作者）：空地 245、障礙 40、未知 160（灰階）；佔據值 `< 0` 為未知、`>= 65` 為障礙（和 `nav2_params.yaml` 的 `occupied_thresh` 0.65 一致），其餘為空地。
- API key：`. ~/dog_12dof_ws/.superpowers/sdd/2026-11-10-plan5-agent-nav-chat/bin/key.sh`（使用者已同意的做法，從 `~/.bashrc` 載入，不印出）。commit 前跑 `grep -rlF "$GEMINI_API_KEY" $AGENT docs README.md`，必須沒有輸出。
- 模擬器：用 `~/dog_12dof_ws/.superpowers/sdd/2026-11-11-plan6-agent-gui/bin/stack.sh`（開相機）。呼叫它的 Bash 指令裡不能出現它的比對字樣（launch 檔名、container 名稱等）。同一時間只跑一個 Isaac Sim；啟動前確認使用者沒有開著。
- `.vscode/settings.json` 和 `robot_dog_navigation/rviz/nav.rviz` 是使用者的修改，不帶進任何 commit（`git commit -- <本計畫的路徑>`）。
- 程式註解英文；畫面上的文字、文件用繁體中文。commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。

## Review Focus

1. **上一段導航的舊路徑**：`/plan` 的最新一份可能是上一個指令留下的。期待新指令開始後、新路徑到之前不畫舊路徑（Task 4 `test_an_old_path_is_not_drawn`）。
2. **建圖模式下地圖變大或原點改變**：slam_toolbox 會擴張地圖。期待換新地圖後點擊換算和狗的位置都用新的範圍（Task 1 `test_view_follows_a_new_map_size`；Task 4 `test_a_new_map_replaces_the_old_one`）。
3. **視窗縮放後點擊**：地圖置中縮放，留白處不算地圖。期待換算依目前元件大小，留白處點了沒反應（Task 1 `test_outside_the_image_is_none`；Task 4 `test_click_after_resize`）。
4. **還沒有地圖或還沒有位置**：期待顯示提示文字、不畫狗、點了沒反應，不當掉（Task 4 `test_no_map_yet`、`test_no_pose_draws_no_dog`）。
5. **點地圖的指令執行中按停止或關視窗**：期待和文字指令一樣取消、不再回頭做事（Task 3 `test_go_to_stop_and_shutdown`）。

---

### Task 1：地圖換算（`map_view.py`）

**Files:**
- Create: `PKG/map_view.py`
- Test: `AGENT/test/test_map_view.py`

**Interfaces:**
- Consumes: `robot_dog_navigation.map_check.load_map(yaml_path) -> (grid int8 [rows, cols]，100 障礙／0 空地／-1 未知，row 0 是 y 最小；resolution；(origin_x, origin_y))`。
- Produces:
  - `FREE, OCCUPIED, UNKNOWN = 245, 40, 160`；`OCCUPIED_AT = 65`。
  - `grid_to_gray(grid) -> bytes`：`grid` 是 2D int 陣列（row 0 是 y 最小），回傳 `rows × cols` 個灰階位元組，**第一列是 y 最大**（畫面由上往下）。
  - `class MapView(resolution, origin, width, height, widget_width, widget_height)`：`width`、`height` 是格數。地圖等比縮放後置中。屬性 `scale`（每格幾像素）、`left`、`top`（地圖左上角的像素位置）。方法 `to_pixel(x, y) -> (px, py)`（float）、`to_world(px, py) -> (x, y) | None`（在地圖影像外回 `None`）。
  - `load_saved_map() -> (grid, resolution, origin)`：讀 `robot_dog_navigation` 的 `maps/apartment.yaml`。尋找方式照 `nav_goals.load_truth`：先找原始碼目錄（`robot_dog_navigation/maps/`，相對於 `robot_dog_navigation` 套件的位置），再找安裝後的 share 目錄；都沒有就丟 `FileNotFoundError`。

- [ ] **Step 1：寫失敗的測試** `test_map_view.py`

```python
import numpy as np
from robot_dog_agent.map_view import FREE, OCCUPIED, UNKNOWN, MapView, grid_to_gray, load_saved_map

def test_grid_to_gray_values_and_flip():
    grid = np.array([[0, 100, -1],       # row 0: smallest y -> last line of the image
                     [64, 65, 0]], dtype=np.int8)
    assert grid_to_gray(grid) == bytes([FREE, OCCUPIED, FREE, FREE, OCCUPIED, UNKNOWN])

VIEW = MapView(0.5, (-1.0, -1.0), 4, 2, 100, 100)   # 2 m x 1 m map in a 100 x 100 widget

def test_view_scales_and_centres():
    assert VIEW.scale == 25.0 and VIEW.left == 0.0 and VIEW.top == 25.0

def test_to_pixel_corners():
    assert VIEW.to_pixel(-1.0, -1.0) == (0.0, 75.0)     # bottom-left of the map
    assert VIEW.to_pixel(1.0, 0.0) == (100.0, 25.0)     # top-right

def test_round_trip():
    x, y = VIEW.to_world(*VIEW.to_pixel(0.3, -0.4))
    assert abs(x - 0.3) < 1e-9 and abs(y + 0.4) < 1e-9
    assert VIEW.to_world(50.0, 50.0) == (0.0, -0.5)

def test_outside_the_image_is_none():
    assert VIEW.to_world(50.0, 10.0) is None and VIEW.to_world(50.0, 90.0) is None
    assert VIEW.to_world(-1.0, 50.0) is None

def test_view_follows_a_new_map_size():
    wide = MapView(0.5, (-2.0, -1.0), 8, 2, 100, 100)   # the map grew to the left (SLAM)
    assert wide.scale == 12.5 and wide.to_world(0.0, 50.0) == (-2.0, -0.5)

def test_load_saved_map():
    grid, res, origin = load_saved_map()
    assert grid.shape == (119, 197) and res == 0.05 and origin == (-3.417, -1.809)
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `map_view.py`** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: map image and coordinate conversion for the window`

### Task 2：`/map` 和 `/plan` 的訂閱（`map_feed.py`）

**Files:**
- Create: `PKG/map_feed.py`

**Interfaces:**
- Produces: `class MapFeed()`：需要 rclpy 已初始化。節點 `robot_dog_agent_map`（`use_sim_time` 為真），自己的 `SingleThreadedExecutor` 在 daemon 執行緒 spin（寫法同 `camera_feed.CameraFeed`）。
  - 訂閱 `/map`（`nav_msgs/OccupancyGrid`）：QoS reliable、transient local、depth 1（啟動前就發布的地圖才收得到）。
  - 訂閱 `/plan`（`nav_msgs/Path`）：預設 QoS、depth 1。
  - `latest_map() -> (grid, resolution, origin, version) | None`：`grid` 是 `np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width)`；`origin` 是 `(msg.info.origin.position.x, msg.info.origin.position.y)`；`version` 從 1 開始，每收到一張新地圖加 1。
  - `latest_path() -> (points, arrived) | None`：`points` 是 `[(x, y), ...]`，`arrived` 是收到時的 `time.monotonic()`。
  - `close()`：同 `CameraFeed.close()`，rclpy 已關閉後呼叫也不丟例外。
- 有鎖；回呼只做轉換和存檔。地圖原點有旋轉的情況不處理（Nav2 的地圖沒有旋轉）。

- [ ] **Step 1：實作 `map_feed.py`**（rclpy 部分沒有純 Python 測試，同 `camera_feed.py`）
- [ ] **Step 2：在隔離的 ROS domain 試一次**：`WS7/bin/map_feed_check.py`（拋棄式）設 `ROS_DOMAIN_ID=79`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，用另一個節點以 transient local 發一張 3×2 的 `OccupancyGrid`，**之後**才建立 `MapFeed`，再發一條兩點的 `Path`。預期 3 秒內 `latest_map()` 的 `grid.shape == (2, 3)`、`version == 1`，`latest_path()[0]` 是那兩點；`close()` 後行程正常結束。
- [ ] **Step 3：`python3 -m pytest test -q` 仍全部通過；Commit**：`robot_dog_agent: MapFeed keeps the latest map and Nav2 path`

### Task 3：點地圖的指令（`Session.go_to`、`tools.tool_line`）

**Files:**
- Modify: `PKG/tools.py`、`PKG/chat.py`、`PKG/session.py`
- Test: `AGENT/test/test_tools.py`、`AGENT/test/test_session.py`

**Interfaces:**
- Produces（`tools.py`）：`tool_line(name: str, args: dict, result: dict) -> str`：`'  [工具] %s %s → %s%s'`，參數用 `json.dumps(args, ensure_ascii=False)`，有 `reason` 時加 `'（%s）'`。`chat.build_agent` 的 `show_call` 改用它（輸出不變）。
- Produces（`session.py`）：
  - `go_to(x: float, y: float) -> bool`：狀態不是 `idle` 時回 `False`。否則和 `submit` 一樣：`remaining = None`、`state busy`、`robot.clear_cancel()`，送 `('log', MAP_LINE % (x, y))`，在背景執行 `self._agent.toolbox.call('go_to_pose', {'x': x, 'y': y})`，送 `('log', tool_line('go_to_pose', args, result))`，最後 `state idle`。例外的處理和 `submit` 相同（`Closing` 也照樣結束指令、不送錯誤）。不呼叫模型，不送 `reply`。
  - `MAP_LINE = '[地圖] 去 (%.2f, %.2f)'`。
  - `on_progress` 照常更新 `remaining`（工具本來就會交給它）；工具結束時清掉。

- [ ] **Step 1：寫失敗的測試**

```python
# test_tools.py
def test_tool_line():
    assert tools.tool_line('go_to_pose', {'x': 1.2, 'y': 0.5}, {'status': 'reached'}) == \
        '  [工具] go_to_pose {"x": 1.2, "y": 0.5} → reached'
    assert tools.tool_line('go_to_place', {'place': 'kitchen'}, {'status': 'blocked', 'reason': 'on an obstacle'}) == \
        '  [工具] go_to_place {"place": "kitchen"} → blocked（on an obstacle）'

# test_session.py (helpers as in the file: started(), CountingRobot, sync, DoneJob, StuckJob)
def test_go_to_runs_the_tool_without_the_model():
    # agent = FakeAgent() with a real Toolbox(robot, places.load_body_truth()) as agent.toolbox
    # session.go_to(1.0, 0.0) is True; agent.asked == []; robot.goals[0][:2] == (1.0, 0.0)
    # events == [('state','busy'), ('log','[地圖] 去 (1.00, 0.00)'),
    #            ('log','  [工具] go_to_pose {"x": 1.0, "y": 0.0} → reached'), ('state','idle')]
def test_go_to_a_blocked_point():
    # go_to(0.4, 2.7): the tool line ends with '→ blocked（not connected to the free space …）', robot.goals == []
def test_go_to_is_refused_unless_idle():
    # before start: False; while busy (a run that only stores fn): False
def test_go_to_stop_and_shutdown():
    # busy go_to (stored fn): stop() -> robot.cancels == 1; then run the stored fn: the tool line says canceled
    # shutdown during a go_to whose tool result comes back afterwards: no error event, state idle, robot closed
def test_go_to_errors_become_error_events():
    # toolbox.call raising ValueError -> ('error', INTERNAL % 'ValueError'), state idle
```

（`FakeAgent` 加一個 `toolbox` 屬性；`test_go_to_runs_the_tool_without_the_model` 用真的 `Toolbox`，所以 `started` 要能傳入 agent。）

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: Session.go_to, a map click goes straight to the go_to_pose tool`

### Task 4：地圖元件與視窗版面（`map_widget.py`、`gui.py`）

**Files:**
- Create: `PKG/map_widget.py`
- Modify: `PKG/gui.py`
- Test: `AGENT/test/test_map_widget.py`、`AGENT/test/test_gui.py`

**Interfaces:**
- Consumes: Task 1 的 `grid_to_gray`、`MapView`、`load_saved_map`；Task 2 的 `latest_map()`、`latest_path()` 格式；Task 3 的 `Session.go_to`；`places.PLACES`。
- Produces（`map_widget.py`）：`class MapWidget(QWidget)`：
  - signal `clicked = pyqtSignal(float, float)`：地圖座標。只有 `set_clickable(True)` 時、而且點在地圖影像內，左鍵按下才發出。可點時游標是 `Qt.PointingHandCursor`，否則一般游標。
  - `set_map(grid, resolution, origin)`：換一張地圖（重建灰階影像）。
  - `set_pose(pose_or_None)`、`set_path(points)`（`[]` 表示不畫）、`set_target(xy_or_None)`。
  - 唯讀屬性（測試用）：`has_map`、`pose`、`path`、`target`、`view`（目前大小下的 `MapView`，沒有地圖時是 `None`）。
  - `NO_MAP = '等地圖…（nav.launch.py 開了嗎？）'`：沒有地圖時置中顯示。
  - `paintEvent` 畫：灰階地圖（等比縮放置中）、八個地點（圓點加 `place.name.split('（')[0]`）、路徑（綠色折線）、目標（紅色 ✕）、狗（藍色三角形，尖端指向朝向）。最小尺寸 300×180。
- Produces（`gui.py`）：
  - `MainWindow(session, latest_frame=None, no_camera_text=NO_CAMERA, clock=time.monotonic, latest_map=None, latest_path=None)`；新屬性 `map_widget`。
  - 版面：右欄 = `map_widget`（stretch 3）+ `log`（stretch 2）+ 輸入列；最小視窗 1100×700。
  - `_apply_state`：`map_widget.set_clickable(state == 'idle')`；進入 `busy` 時記下 `self._busy_since = clock()`；離開 `busy` 時記下 `self._target_until = clock() + TARGET_HOLD`。
  - `map_widget.clicked` → `session.go_to(x, y)`；回 `True` 才 `set_target((x, y))`。
  - `refresh_map()`（狀態計時器每 0.5 秒呼叫，和 `refresh_status` 一起）：`latest_map()` 的 `version` 變了才 `set_map`；`set_pose(session.pose())`；路徑只在 `state == 'busy'` 而且 `latest_path()` 的 `arrived >= self._busy_since` 時畫，否則 `[]`；`target` 在 `clock() > self._target_until` 而且不在 `busy` 時清掉。
  - `TARGET_HOLD = 2.0`。
  - `create(args, api_key) -> (window, session, feeds)`：**第三個回傳值改成 list**（`[CameraFeed, MapFeed]`，`--no-robot` 時是 `[]`）。`--no-robot` 時 `latest_map` 是固定回傳 `load_saved_map()` 加 `version` 1 的函式、`latest_path` 為 `None`。`main` 結束時逐一 `close()`。
  - 文字指令經過 Agent 時也畫路徑（同一套規則）；地點按鈕不設 `target`（spec 只對點地圖畫 ✕）。

- [ ] **Step 1：寫失敗的測試** `test_map_widget.py`（offscreen；模組層一個 `QApplication`；`GRID` 用 `load_saved_map()` 的結果）

```python
def widget(size=(600, 360)):          # MapWidget with the saved map, resized and shown offscreen
def test_no_map_yet():                # fresh MapWidget: has_map False, view None; set_clickable(True) and a click at
                                      # the centre emits nothing; grab() does not raise
def test_click_emits_map_coordinates():
    # set_clickable(True); px, py = w.view.to_pixel(2.6, 2.0); QTest.mouseClick at (round(px), round(py)):
    # one clicked signal with x, y within one pixel's worth (1 / w.view.scale * resolution) of (2.6, 2.0)
def test_not_clickable_emits_nothing():
def test_click_in_the_margin_emits_nothing():   # a widget much wider than the map: click at x = 2 px
def test_click_after_resize():                  # resize to (900, 400), then the same check as above with the new view
def test_cursor_follows_clickable():
def test_a_new_map_replaces_the_old_one():      # set_map with a 4x2 grid at origin (-1, -1): view uses the new size
def test_no_pose_draws_no_dog():                # set_pose(None): pose None; grab() does not raise
def test_everything_drawn():                    # pose, path of 3 points, target set: grab() is not null
```

`test_gui.py` 新增（`FakeSession` 加 `go_to`、`went`）：

```python
def test_a_map_click_sends_go_to_and_marks_the_target():
    # window(latest_map=<saved map, version 1>); w.refresh_map(); emit w.map_widget.clicked(2.6, 2.0):
    # session.went == [(2.6, 2.0)], w.map_widget.target == (2.6, 2.0)
def test_a_refused_go_to_marks_nothing():        # session.accept = False -> target None
def test_the_map_is_clickable_only_when_idle():  # idle True; busy, starting, failed False
def test_the_dog_follows_the_session_pose():     # session.pose_value set -> after refresh_map, map_widget.pose equals it
def test_an_old_path_is_not_drawn():
    # clock 10: state busy (busy_since 10); latest_path -> ([(0,0),(1,0)], 9.0): path [] after refresh_map;
    # latest_path -> (…, 11.0): path drawn; state idle: path []
def test_the_target_is_cleared_two_seconds_after_the_command():
    # target set, busy then idle at clock 20: still there at 21.5, gone at 22.5
def test_the_map_is_redrawn_only_for_a_new_version():   # count set_map calls (wrap) over three refreshes with
                                                        # versions 1, 1, 2 -> 2 calls
def test_create_without_a_robot_has_the_saved_map():    # replaces the old create test: feeds == [], after the start
                                                        # and refresh_map the map widget has_map
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `map_widget.py` 和 `gui.py` 的改動** → **Step 4：`python3 -m pytest test -q` 全部通過；`colcon build --symlink-install --packages-select robot_dog_agent` 成功**
- [ ] **Step 5：`--no-robot` 截圖**：沿用計畫 6 的做法寫 `WS7/bin/shot_norobot.py`（offscreen，`gui.create(--no-robot)`，視窗 1200×760），點地圖上 (2.6, 2.0)，等回到「待命」，存 `WS7/runs/map_norobot.png`。預期對話區有 `[地圖] 去 (2.60, 2.00)` 和 `go_to_pose … → reached`，沒有「狗：」那一行；用 Read 看截圖：地圖、地點名稱、狗的三角形都看得到、版面沒跑掉。這一步不用 key（點地圖不呼叫模型），用假 key 即可。
- [ ] **Step 6：Commit**：`robot_dog_agent: the map in the window, click to send the dog`

### Task 5：實機檢查與文件

**Files:**
- Create: `WS7/bin/map_live_check.py`（拋棄式）
- Modify: `AGENT/README.md`、`~/dog_12dof_ws/README.md`、spec（「結果」一節，以及第 5 節的檔案表改成 `map_widget.py`）、`AGENT/gui.png`

- [ ] **Step 1：寫驅動腳本**：offscreen 的真視窗（`gui.create`，真的 `Nav2Robot`、`CameraFeed`、`MapFeed`），另一個節點訂閱 `/odom_gt`。把 `GeminiClient.send` 包一層計數。點擊用 `map_widget.clicked.emit(x, y)`（換算已由單元測試涵蓋）。依 spec 第 6 節四項，每項印 `[MAP] <n> PASS|FAIL <數字>`，最後 `[MAP] PASS k/4`：
  1. 30 秒內 `map_widget.has_map`；`refresh_map` 後 `map_widget.pose` 和真值相差 < 0.15 m。
  2. 點 (2.6, 2.0)：導航途中 `map_widget.path` 不是空的（途中存 `WS7/runs/map_live.png`）；回到「待命」時真值在 0.25 m 內；Gemini 計數仍是 0。
  3. 點 (0.4, 2.7)：對話區最後一行含 `blocked`；狗 3 秒內移動 < 0.1 m；Nav2 沒有新目標（`/navigate_to_pose/_action/status` 沒有執行中的）。
  4. 點 (-2.5, -0.3)，路徑出現後 5 秒按停止：對話區出現 `canceled`；回到「待命」；之後 3 秒移動 < 0.1 m。
- [ ] **Step 2：跑實機檢查**：`stack.sh start False`；`gui.create` 需要 key（啟動時建立 Gemini 客戶端，但這四項都不送請求）；輸出存 `WS7/runs/map_live_1.log`，需要 `[MAP] PASS 4/4`。不過時依原因修正（程式錯誤先補會失敗的測試），修完整輪重跑。用 Read 看 `map_live.png`。
- [ ] **Step 3：`stack.sh stop`；寫文件**：`AGENT/README.md` 的「視窗版」一節：版面說明加地圖一列（看得到什麼、點了會怎樣、不用 Gemini、走不到時會拒絕、停止一樣有效）、限制（不能指定到達朝向、不能縮放）、實測結果；檔案表加 `map_view.py`、`map_feed.py`、`map_widget.py`。用 `map_live.png` 取代 `AGENT/gui.png`。根目錄 `README.md` 的視窗版說明加一句「還有地圖，點一下狗就走過去」。spec 加「結果」一節。用 SendUserFile 把截圖傳給使用者。
- [ ] **Step 4：`python3 -m pytest test -q` 通過；key 外洩檢查；Commit**：`robot_dog_agent: map verified live, README`
