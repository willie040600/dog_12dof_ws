# 計畫 8：用相機認東西（巡邏記憶、隨叫隨到、找不到就去找）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 視窗版的狗能用相機加深度認出家具、記住它們在地圖上的位置（存檔），並且「去〈東西〉旁邊」時自己算出停的位置和朝向；記憶裡沒有就去找。

**Architecture:** 模擬端在現有相機上多發布一張深度影像。純 Python 的 `vision.py`（框 + 深度 → 地圖座標）、`objects.py`（記憶與合併）、`places.stand_near`（停的位置）負責所有計算；`eyes.py` 把「拍照 → 問 Gemini → 換算 → 記住」串起來；`Toolbox` 有眼睛時多六個工具。只有視窗版接上眼睛，終端機版和 `agent_eval` 不變。

**Tech Stack:** Python 3.12、numpy、OpenCV（`cv2`，已裝）、PyQt5、rclpy（Jazzy）、Isaac Sim 6.2 OmniGraph（`ROS2CameraHelper`）、Gemini Interactions API（圖片輸入）

**Spec:** `docs/superpowers/specs/2026-11-18-object-memory-design.md`

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`AGENT` = `~/dog_12dof_ws/robot_dog_agent`，`PKG` = `AGENT/robot_dog_agent`，`ISAAC` = `~/dog_12dof_ws/robot_dog_description/isaac`，`WS8` = `~/dog_12dof_ws/.superpowers/sdd/2026-11-18-plan8-object-memory`（拋棄式腳本和 log）。
- 測試：`cd $AGENT && python3 -m pytest test -q`，不 source ROS、不需要網路。原有 154 個測試必須仍然通過。
- `vision.py`、`objects.py`、`eyes.py`、`places.py`、`tools.py`、`session.py` 不可匯入 rclpy、requests、PyQt5（`vision.py` 可以用 `cv2`、numpy）。
- 模擬端只改 `ISAAC/robot_dog_sim/ros_graph.py` 的 `camera_graph`。不改 Nav2 參數、地圖檔、`robot_dog_navigation`。
- 沒有眼睛時（`chat`、`agent_eval`、`gui --no-robot`），工具清單和 system instruction 必須和現在**逐字相同**。
- 照 spec 的值：合併距離 0.6 m（同名）；有效深度 0.1～5.0 m；停的位置在記下位置的 0.5～1.0 m 圓環上、淨空 ≥ 0.30 m；巡邏 8 個觀察點 × 4 個方向，最多 32 次認東西的請求；`find_object` 每次最多 32 次；記憶檔 `~/.local/share/robot_dog_agent/objects.json`。
- 認東西預設模型 `VISION_MODEL = 'gemini-3.5-flash-lite'`；Task 7 Step 2 比較後若改用 `gemini-robotics-er-2-preview`，改這個常數並記入 spec。spec 說比較是「第一步」；這裡放到 Task 7，因為要先有整條「拍照 → 換算」的流程才能打分數（ledger 記一行 Ruling）。
- API key：`. ~/dog_12dof_ws/.superpowers/sdd/2026-11-10-plan5-agent-nav-chat/bin/key.sh`（使用者同意的做法，不印出）。commit 前 `grep -rlF "$GEMINI_API_KEY" $AGENT docs README.md`，必須沒有輸出。
- 模擬器：`~/dog_12dof_ws/.superpowers/sdd/2026-11-11-plan6-agent-gui/bin/stack.sh`（開相機）。呼叫它的 Bash 指令裡不能出現它的比對字樣（launch 檔名、container 名稱等）。同一時間只跑一個 Isaac Sim；啟動前確認使用者沒有開著。Task 1 改了模擬腳本，之後每次都要重新啟動模擬器才生效。
- `.vscode/settings.json` 和 `robot_dog_navigation/rviz/nav.rviz` 是使用者的修改，不帶進任何 commit（`git commit -- <本計畫的路徑>`）。
- 程式註解英文；畫面上的文字、文件用繁體中文。commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。

## Review Focus

1. **Gemini 回的不是乾淨的 JSON**：多了說明文字、```json 圍欄、缺欄位、框超出 0～1000、空清單。期待照樣解析能用的、丟掉壞的，不當掉（Task 2 `test_parse_detections_tolerates_mess`）。
2. **框落在窗戶或遠處**：框內深度全是無限大、超過 5 m、或有效像素太少。期待那個框被捨棄，不會記下錯的位置（Task 2 `test_box_depth_rejects_far_and_empty`）。
3. **記憶檔壞掉或寫不進去**：JSON 毀損、資料夾不存在。期待從空的記憶開始、照常運作（Task 3 `test_a_broken_file_starts_empty`、`test_the_folder_is_created`）。
4. **巡邏或找東西途中按停止**：期待立刻停、不再走下一個觀察點，已看到的保留（Task 5 `test_survey_stops_on_cancel`、`test_find_stops_on_cancel`）。
5. **模型給的 id 不存在**（記憶剛被清空、或它猜錯）：期待 `go_to_object` 回 `error`，狗不動（Task 5 `test_go_to_an_unknown_object`）。

---

### Task 1：模擬端的深度影像

**Files:**
- Modify: `ISAAC/robot_dog_sim/ros_graph.py`（`camera_graph`）

**Interfaces:**
- Produces: `/camera/depth/image_raw`（`sensor_msgs/Image`，`32FC1`，公尺，`frame_id` `camera_optical_link`），和 `/camera/image_raw` 同一個 render product、同樣的 `frameSkipCount`，所以時間戳一致、像素對齊。

- [ ] **Step 1：改 `camera_graph(stage, cam_path, size, topic='/camera/image_raw', info_topic='/camera/camera_info', depth_topic='/camera/depth/image_raw', frame_id='camera_optical_link', frame_skip=0)`**：多一個節點 `('PubDepth', ROS + 'ROS2CameraHelper')`，`inputs:type` 設 `'depth'`，其餘值和連線照 `PubImage` 的寫法（execIn、renderProductPath、context、frameId、frameSkipCount）。
- [ ] **Step 2：實測**（確認使用者沒開 Isaac Sim；`stack.sh start False`）：
  - `ros2 topic hz /camera/depth/image_raw` 約每秒 9 張（和彩色相同）。
  - 一則深度訊息：`encoding` 是 `32FC1`，640×480，`frame_id` 是 `camera_optical_link`；畫面中央的值是有限的公尺數（狗在起點面向電視櫃，約 0.6～1.0 m）。
  - 同時收一張彩色、一張深度，時間戳相同。
  - 記下開深度前後的即時率（`/clock` 和牆上時間的比值，各量 20 秒），寫進 ledger。
  - `stack.sh stop`。
- [ ] **Step 3：`cd $ISAAC && python3 -m pytest tests -q` 仍全部通過；Commit**（只有 `ros_graph.py`）：`Isaac Sim: publish the camera's depth image as well`

### Task 2：影像與深度的換算（`vision.py`）

**Files:**
- Create: `PKG/vision.py`
- Test: `AGENT/test/test_vision.py`

**Interfaces:**
- Produces:
  - `Detection = namedtuple('Detection', 'name box')`：`box` 是 `(ymin, xmin, ymax, xmax)`，0～1000。
  - `Seen = namedtuple('Seen', 'name x y distance width')`：地圖座標（m）、距離（m）、大概寬度（m）。
  - `Snapshot = namedtuple('Snapshot', 'rgb width height depth K T_map_cam')`：`rgb` 是 RGB 位元組；`depth` 是 `float32 [height, width]`（m）；`K` 是 3×3；`T_map_cam` 是 4×4（`camera_optical_link` → `map`）。
  - `MIN_RANGE, MAX_RANGE = 0.1, 5.0`；`INNER = 0.5`（取框中間邊長一半的區域）；`MIN_PIXELS = 20`。
  - `parse_detections(text: str) -> list[Detection]`：從文字中找出第一個 JSON 陣列（容忍前後說明文字和 ```json 圍欄）；每項要有字串 `label` 和四個數字的 `box_2d`；座標夾到 0～1000；面積為 0 或欄位不對的項目丟掉；完全找不到陣列回 `[]`。
  - `detection_prompt(target: str | None = None) -> str`：要求只回傳 JSON 陣列，每項 `{"label": 繁體中文簡短名稱, "box_2d": [ymin, xmin, ymax, xmax]}`（0～1000）；沒有 `target` 時列出家具和大型物品；有 `target` 時只列符合 `target` 的，沒有就回 `[]`。用詞由實作者決定（spec 第 8 節）。
  - `box_pixels(box, width, height) -> (u0, v0, u1, v1)`（int，`u` 是欄、`v` 是列）。
  - `box_depth(depth, pixels) -> float | None`：框中間 `INNER` 比例區域內、有限且在 `MIN_RANGE`～`MAX_RANGE` 之間的值，少於 `MIN_PIXELS` 個回 `None`，否則回中位數。
  - `transform_matrix(translation, quaternion) -> np.ndarray`：`quaternion` 是 `(x, y, z, w)`。
  - `pixel_to_map(u, v, z, K, T_map_cam) -> (x, y)`：針孔模型，`z` 是沿光軸的深度。
  - `locate(detections, snapshot) -> list[Seen]`：框中心像素 + `box_depth` → 地圖座標；`width = (u1 - u0) * z / fx`；深度無效的框丟掉。
  - `depth_from_msg(encoding, width, height, step, data) -> np.ndarray | None`：`32FC1`（公尺）、`16UC1`（毫米 → 公尺）；有列填充也要處理；其他格式或長度不夠回 `None`。
  - `encode_jpeg(rgb, width, height, quality=85) -> bytes`（`cv2.imencode`，先 RGB → BGR）。

- [ ] **Step 1：寫失敗的測試**（`K = [[466, 0, 320], [0, 466, 240], [0, 0, 1]]`；`OPTICAL = transform_matrix((0.178, 0.0, 0.28), (-0.5, 0.5, -0.5, 0.5))` 是「狗在原點面向 +x、相機光軸朝 +x」）

```python
def test_transform_matrix_optical_frame():
    assert np.allclose(OPTICAL[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
def test_pixel_to_map_centre_and_right():
    assert np.allclose(pixel_to_map(320, 240, 2.0, K, OPTICAL), (2.178, 0.0))
    assert np.allclose(pixel_to_map(320 + 466, 240, 2.0, K, OPTICAL), (2.178, -2.0))   # image right = robot's right
def test_box_pixels():
    assert box_pixels((250, 500, 750, 1000), 640, 480) == (320, 120, 640, 360)
def test_box_depth_uses_the_middle():          # 2.0 in the middle quarter, 9.0 around it -> 2.0
def test_box_depth_rejects_far_and_empty():    # all inf -> None; all 6.0 -> None; only 10 valid pixels -> None
def test_locate():                             # one box centred on the image, depth 2.0 there -> Seen('沙發', 2.178, 0.0, 2.0, width)
                                               # with width == (u1 - u0) * 2.0 / 466; a box over inf depth is dropped
def test_parse_detections():
    text = '[{"label": "沙發", "box_2d": [100, 200, 300, 400]}]'
    assert parse_detections(text) == [Detection('沙發', (100, 200, 300, 400))]
def test_parse_detections_tolerates_mess():
    # '好的：```json\n[ … ]\n```' parses; a missing label, a 3-number box, a zero-area box are dropped;
    # [-5, 0, 1200, 900] becomes (0, 0, 1000, 900); 'no objects' -> []; '[]' -> []; 'not json [1,' -> []
def test_detection_prompt():                   # mentions box_2d and 0-1000 both times; contains the target name when given
def test_depth_from_msg():                     # 32FC1 2x1 floats; 16UC1 [1500, 0] -> [1.5, 0.0]; row padding; 'rgb8' -> None
def test_encode_jpeg_round_trip():             # a 4x2 red image: cv2.imdecode gives a 4x2 image whose pixels are red-ish (BGR)
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: vision maths, from a box and depth to a point on the map`

### Task 3：記憶與停的位置（`objects.py`、`places.stand_near`）

**Files:**
- Create: `PKG/objects.py`
- Modify: `PKG/places.py`
- Test: `AGENT/test/test_objects.py`、`AGENT/test/test_places.py`

**Interfaces:**
- Consumes: Task 2 的 `Seen`。
- Produces:
  - `MERGE_DISTANCE = 0.6`；`MEMORY_PATH = os.path.expanduser('~/.local/share/robot_dog_agent/objects.json')`。
  - `Item = namedtuple('Item', 'id name x y width seen last_seen')`。
  - `class ObjectMemory(path=MEMORY_PATH, clock=time.time)`：建立時讀檔（檔案不存在或壞掉就從空的開始，壞檔不刪）；`items -> list[Item]`（依 id 排序）；`add(seen) -> str`（回傳 id；同名且距離 ≤ 0.6 m 就合併：位置和寬度依 `seen` 次數加權平均、`seen + 1`、更新 `last_seen`；否則新增 `obj_<n>`，`n` 從 1 遞增、清空後也不重用）；`find(item_id) -> Item | None`；`clear()`。每次 `add`、`clear` 都寫檔（先寫暫存檔再 `os.replace`；資料夾不存在就建立）。寫檔失敗不丟例外（記憶仍在程式裡）。
  - `places.stand_near(truth, target, robot_xy) -> (x, y, yaw) | None`：在以 `target` 為中心、半徑 0.5、0.6、…、1.0 m，每圈 24 個角度的點中，留下 `check_target` 通過的，選離 `robot_xy` 最近的；`yaw` 面向 `target`。都不行回 `None`。

- [ ] **Step 1：寫失敗的測試**

```python
# test_objects.py (tmp_path for every memory; clock = a list-backed fake)
def test_add_new_and_ids():                   # two different names -> obj_1, obj_2
def test_same_name_close_merges_weighted():   # 沙發 at (0, 0) twice then at (0.3, 0) -> one item at x == 0.1, seen 3
def test_same_name_far_does_not_merge():      # 0.7 m apart -> two items
def test_different_names_do_not_merge():      # 沙發 and 茶几 0.1 m apart -> two items
def test_saved_and_loaded():                  # a new ObjectMemory on the same path has the same items and goes on with obj_3
def test_clear_is_saved_and_ids_go_on():      # clear(); a new memory on the path is empty; the next add is obj_3
def test_a_broken_file_starts_empty():        # path holds 'not json' -> items == [], the file is still there
def test_the_folder_is_created():             # path in a folder that does not exist yet -> add() creates it
def test_an_unwritable_path_does_not_raise(): # path is a directory -> add() returns an id, items has it
def test_find():                              # find('obj_1') is the item; find('obj_9') is None

# test_places.py
def test_stand_near_the_sofa():
    x, y, yaw = places.stand_near(TRUTH, (0.0, -0.75), (0.0, 0.0))     # the sofa's front, seen from the start
    d = math.hypot(x - 0.0, y + 0.75)
    assert 0.5 - 1e-9 <= d <= 1.0 + 1e-9 and places.check_target(TRUTH, x, y) is None
    assert abs(places.wrap(yaw - math.atan2(-0.75 - y, 0.0 - x))) < 1e-9
def test_stand_near_picks_the_side_nearest_the_robot():   # same target, robot at (1.5, -0.3) -> x > 0
def test_stand_near_nowhere():                            # target (20.0, 0.0) -> None
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: object memory with merging, and where to stand near a thing`

### Task 4：問 Gemini 一張圖（`GeminiClient.ask_image`）

**Files:**
- Modify: `PKG/gemini.py`
- Test: `AGENT/test/test_gemini.py`

**Interfaces:**
- Produces: `GeminiClient.ask_image(prompt: str, jpeg: bytes) -> str`：送 `{'model': self.model, 'input': [{'type': 'text', 'text': prompt}, {'type': 'image', 'data': <base64>, 'mime_type': 'image/jpeg'}]}`，不帶 `system_instruction`、`tools`、`previous_interaction_id`；不改變 `self.previous_id`；回傳所有 `model_output` 文字串起來。重試、錯誤處理和 key 的保護和 `send` 相同（把現有 `_post` 的重試迴圈抽成 `_post_request(request) -> response`，兩者共用）。

- [ ] **Step 1：寫失敗的測試**（沿用 `test_gemini.py` 的 `FakePost`、`client()`、`recorded()`）

```python
def test_ask_image_request_shape():
    # c.ask_image('找沙發', b'\xff\xd8jpeg'): one post; json == {'model': MODEL, 'input': [
    #   {'type': 'text', 'text': '找沙發'}, {'type': 'image', 'data': base64 of the bytes, 'mime_type': 'image/jpeg'}]}
    # returns the recorded text reply's text
def test_ask_image_leaves_the_conversation_alone():
    # send(text='去廚房') then ask_image(...) then send(results=…): the image request has no previous_interaction_id and
    # the last send still carries the id of the first reply
def test_ask_image_retries_and_errors_like_send():   # 429 then a reply -> the text, sleeps == [10]; error_key -> GeminiError
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過（原有 Gemini 測試不變）**
- [ ] **Step 5：Commit**：`robot_dog_agent: GeminiClient.ask_image, one picture outside the conversation`

### Task 5：眼睛、六個工具、巡邏指令（`eyes.py`、`tools.py`、`session.py`、`chat.py`）

**Files:**
- Create: `PKG/eyes.py`
- Modify: `PKG/tools.py`、`PKG/session.py`、`PKG/chat.py`
- Test: `AGENT/test/test_eyes.py`、`AGENT/test/test_tools.py`、`AGENT/test/test_session.py`

**Interfaces:**
- Consumes: Task 2 的 `Snapshot`、`parse_detections`、`detection_prompt`、`locate`、`encode_jpeg`；Task 3 的 `ObjectMemory`、`stand_near`；Task 4 的 `ask_image`（以函式傳入）。
- Produces（`eyes.py`）：`class Eyes(snap, ask_image, memory)`：`snap() -> Snapshot | None`；`look(target=None) -> list[Seen]`：沒有影像丟 `RuntimeError('no camera image yet')`；否則送圖、解析、換算，每個 `Seen` 都 `memory.add`；`ask_image` 的例外照樣往上丟。屬性 `memory`。
- Produces（`tools.py`）：
  - `declarations(with_eyes=False) -> list`、`system_instruction(with_eyes=False) -> str`。`False` 時和現在的 `DECLARATIONS`、`system_instruction()` 完全相同（`DECLARATIONS` 保留為 `declarations()` 的結果）。`True` 時多六個工具（spec 4.5），instruction 多一段：說到某件東西時先 `list_objects`；有就 `go_to_object`，沒有就 `find_object`；不要猜座標；巡邏用 `survey`。
  - `Toolbox(robot, truth, on_progress=None, eyes=None, on_note=None)`。
  - 新工具的回傳（全都含 `status`）：
    - `list_objects` → `{'status': 'ok', 'objects': [{'id', 'name', 'x', 'y', 'area'}, …]}`（座標兩位小數，`area` 用 `places.area_of`）。
    - `go_to_object(object_id)` → 不存在：`error`；`stand_near` 為 `None`：`blocked`（`'no reachable place to stand near it'`）；否則照 `_navigate` 的結果，再加 `'object': id`。
    - `find_object(name)` → `name` 必須是非空字串；觀察點（`places.PLACES` 的座標）依離狗的距離排序；每點：導航過去（`blocked`/`failed`/`timeout` 就跳過這點；`canceled` 立刻回 `canceled`）→ 四個方向各 `eyes.look(target=name)`，每看完一次原地左轉 90°（`robot.go_to` 同位置、`yaw + pi/2`）；某次看到東西就停止搜尋，取其中最近的那個在記憶中的 id，回傳 `go_to_object` 的結果並加 `'found_at'`（觀察點 id）；全部看完沒有 → `{'status': 'not_found', 'reason': 'looked from every observation point', 'looks': n}`。`look` 丟例外就算這次沒看到、繼續。最多 32 次 `look`。
    - `look_around` → 原地四個方向各 `look()`（每次後左轉 90°）；`{'status': 'ok', 'seen': [名稱…], 'remembered': len(memory.items)}`。
    - `survey` → 先 `memory.clear()`；依 `PLACES` 順序走到每個觀察點，`on_note('巡邏中 %d/%d' % (i, 8))`；走不到就記入 `skipped` 繼續；`canceled` 立刻停（回 `canceled`，附已記住的數量）；每點四個方向 `look()`，例外就計入 `failed_looks` 繼續；完成回 `{'status': 'ok', 'remembered': n, 'looks': k, 'failed_looks': f, 'skipped': [...]}`。
    - `forget_objects` → `memory.clear()`，`{'status': 'ok'}`。
    - 沒有眼睛卻被呼叫：`error`（`'no camera on this robot'`）。
  - `'not_found'` 不是 `ok`/`reached`，`Agent` 照現有規則跳過同一輪後面的呼叫。
- Produces（`session.py`）：`note` 屬性（`on_note` 設定，指令結束清掉）；`status_text()` 在 `busy` 時：有 `note` 顯示 `note`，再有距離時接 `'，剩 %.1f m'`；`survey() -> bool`：和 `go_to` 相同的狀態流程，log 一行 `'[巡邏] 開始'`，背景執行 `toolbox.call('survey', {})`，再 log `tool_line`。`make_agent` 多傳 `on_note=self._note`。
- Produces（`chat.py`）：`build_agent(robot, api_key, model, say=print, on_call=None, on_progress=None, eyes=None, on_note=None)`：把 `eyes` 交給 `Toolbox`，並用 `declarations(eyes is not None)`、`system_instruction(eyes is not None)` 建 `GeminiClient`；`VISION_MODEL = 'gemini-3.5-flash-lite'`。

- [ ] **Step 1：寫失敗的測試**（`FakeEyes(script)`：`look(target)` 依序回傳 `script` 裡的 `Seen` 清單（或丟出例外），同時真的 `memory.add`；記下每次的 `target`）

```python
# test_eyes.py
def test_look_sends_the_picture_and_remembers():   # fake snap and ask_image ('[{"label":"沙發","box_2d":[...]}]'):
                                                   # one Seen, memory has 沙發, ask_image got the jpeg and the prompt
def test_look_without_a_picture():                 # snap -> None: RuntimeError
def test_look_with_a_target_uses_it_in_the_prompt():
def test_look_lets_model_errors_through():         # ask_image raises -> the same exception, memory unchanged

# test_tools.py
def test_without_eyes_nothing_changes():
    assert tools.declarations() == tools.DECLARATIONS and tools.system_instruction() == tools.system_instruction(False)
    assert {d['name'] for d in tools.declarations()} == {'get_pose', 'go_to_place', 'go_to_pose', 'move_relative'}
def test_with_eyes_six_more_tools():               # names add list_objects, go_to_object, find_object, look_around,
                                                   # survey, forget_objects; the instruction mentions list_objects
def test_eye_tools_without_eyes_are_errors():
def test_list_objects():
def test_go_to_an_object():                        # memory has 沙發 at (0.0, -0.75): reached, robot ended 0.5-1.0 m from it
def test_go_to_an_unknown_object():                # 'obj_9' -> error, robot.goals == []
def test_go_to_an_object_with_nowhere_to_stand():  # item at (20, 0) -> blocked
def test_survey_visits_every_point_and_looks_four_ways():
    # FakeRobot, FakeEyes returning [] each time: goals visit the 8 places in table order; 32 looks; memory cleared
    # first; notes == ['巡邏中 1/8', …, '巡邏中 8/8']; result ok with looks 32
def test_survey_skips_unreachable_points():        # FakeRobot results: the first go_to failed -> skipped == ['start']
def test_survey_counts_failed_looks():             # one look raises -> failed_looks 1, still ok
def test_survey_stops_on_cancel():                 # robot.cancel() after the 2nd point: status canceled, no 3rd point
def test_find_goes_to_what_it_finds():             # nothing at the first point, 洗衣機 at the second -> reached, found_at set,
                                                   # every look had target '洗衣機'
def test_find_gives_up_after_every_point():        # always [] -> not_found, looks 32
def test_find_stops_on_cancel():
def test_find_needs_a_name():                      # '', 3, missing -> error
def test_forget_objects():

# test_session.py
def test_survey_runs_the_tool_and_shows_progress():   # status_text while busy shows the note; log has '[巡邏] 開始' and the tool line
def test_note_and_distance_together():                # note '巡邏中 2/8' and remaining 1.23 -> '巡邏中 2/8，剩 1.2 m'
def test_note_is_cleared_after_the_command():
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過**（包含原有的 `chat`、`agent_eval` 相關測試）
- [ ] **Step 5：Commit**：`robot_dog_agent: eyes and six tools to remember, find and go to things`

### Task 6：相機快照與視窗（`camera_feed.py`、`map_widget.py`、`gui.py`）

**Files:**
- Modify: `PKG/camera_feed.py`、`PKG/map_widget.py`、`PKG/gui.py`
- Test: `AGENT/test/test_map_widget.py`、`AGENT/test/test_gui.py`

**Interfaces:**
- Consumes: Task 2 的 `Snapshot`、`depth_from_msg`、`transform_matrix`；Task 3 的 `ObjectMemory`；Task 4 的 `ask_image`；Task 5 的 `Eyes`、`Session.survey`、`build_agent(eyes=…)`。
- Produces（`camera_feed.py`）：`CameraFeed` 多訂閱 `/camera/depth/image_raw`（只留最近 5 張，以時間戳為鍵）和 `/camera/camera_info`（存 `K`）；彩色也記下時間戳。`snapshot(tf_buffer) -> Snapshot | None`：取最新一張彩色，找時間戳相同（或相差 ≤ 0.05 s）的深度；查 `map` → `camera_optical_link`（用影像時間戳，查不到就用最新的）；缺任何一樣回 `None`。
- Produces（`map_widget.py`）：`set_objects(items)`（`[(name, x, y), …]`）；畫成橘色小方塊加名稱；屬性 `objects`。
- Produces（`gui.py`）：
  - `MainWindow(…, objects=None)`：`objects() -> [(name, x, y), …]`；`refresh_map` 一併 `set_objects`。
  - 「巡邏」按鈕 `survey_button`：只在有眼睛時出現（`MainWindow(…, can_survey=False)`），按下 `session.survey()`；鎖定規則和地點按鈕相同。
  - `create(args, api_key)`：非 `--no-robot` 時建立 `ObjectMemory()`、看圖用的 `GeminiClient(api_key, args.vision_model, '', [])`，`make_agent` 用 `Eyes(lambda: camera.snapshot(robot.tf_buffer), vision.ask_image, memory)` 建 agent；`objects` 是記憶裡的清單；`can_survey=True`。`--no-robot` 不變。
  - `main` 多一個 `--vision-model`（預設 `chat.VISION_MODEL`）。

- [ ] **Step 1：寫失敗的測試**

```python
# test_map_widget.py
def test_objects_are_drawn():               # set_objects([('沙發', 0.0, -0.75)]): objects == that; grab() is not null and the
                                            # pixel at the sofa's spot is orange-ish (red > 200, blue < 100)
# test_gui.py
def test_the_survey_button_calls_survey():  # can_survey=True: click -> session.surveyed == 1
def test_the_survey_button_follows_the_state():   # enabled only when idle, like the place buttons
def test_no_survey_button_without_eyes():   # can_survey=False: survey_button is None
def test_remembered_objects_on_the_map():   # objects=lambda: [('沙發', 0.0, -0.75)]: after refresh_map the map widget has it
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作** → **Step 4：`python3 -m pytest test -q` 全部通過；`colcon build --symlink-install --packages-select robot_dog_agent` 成功**
- [ ] **Step 5：`--no-robot` 看一次**：offscreen 截圖（沿用計畫 7 的 `shot_norobot.py` 寫法），確認版面沒變、沒有巡邏按鈕。
- [ ] **Step 6：Commit**：`robot_dog_agent: camera snapshots with depth, things on the map, a survey button`

### Task 7：比較模型、實機檢查、文件

**Files:**
- Create: `WS8/bin/furniture_truth.py`、`WS8/bin/compare_models.py`、`WS8/bin/objects_live_check.py`（都是拋棄式）
- Modify: `AGENT/README.md`、`~/dog_12dof_ws/README.md`、spec（「模型比較」與「結果」兩節）、`AGENT/gui.png`；視比較結果可能改 `chat.VISION_MODEL`

- [ ] **Step 1：家具真值**：`furniture_truth.py` 用 Isaac 的 Python（`pxr`）讀公寓 USD，把 `Furniture_Group`、`Kitchen_Group`、`Bath_Group` 底下的零件依名稱前綴（例如 `Sofa_*`、`DiningChair_1_*`）合併成一件，算出世界座標（公寓座標 −(3.5, 1.8)）的平面外框，存成 `WS8/runs/furniture_truth.json`。並列出 spec 第 7 節清單裡每一項對應到哪個名稱。
- [ ] **Step 2：比較模型**：模擬器開相機（`stack.sh start False`）；`compare_models.py` 用 `Nav2Robot` 和 `CameraFeed.snapshot` 在起點、客廳西側、廚房、主臥室各拍一張（存檔），同一組影像分別用 `gemini-3.5-flash-lite` 和 `gemini-robotics-er-2-preview` 跑 `detection_prompt()` + `locate`；以真值打分（落在某件家具外框 0.5 m 內的偵測數、找到的不同家具數、離外框最遠的錯誤）。用掉約 8 次請求。ER 明顯較準且沒有遇到額度錯誤才改 `VISION_MODEL`；結果寫進 spec 新的「模型比較」一節。
- [ ] **Step 3：實機檢查**：重新啟動模擬器；`objects_live_check.py` 以 offscreen 的真視窗（`gui.create`）依 spec 第 7 節執行，每項印 `[OBJ] <n> PASS|FAIL <數字>`，最後 `[OBJ] PASS k/5`：
  1. 按「巡邏」：完成；記憶中至少 8 件落在真值外框 0.5 m 內；記錄巡邏用掉的請求數（≤ 32 次認東西 + 0 次對話）。
  2. 說「去沙發旁邊」：停在沙發外框 1.2 m 內，朝向和「面向記下的位置」相差 ≤ 30°。
  3. 說「去〈清單中另一件，若有冰箱就用冰箱，否則餐桌〉旁邊」：同第 2 項的標準。
  4. 說「忘掉看過的東西」，再說「去洗衣機旁邊」：自己找到並停在洗衣機外框 1.2 m 內。
  5. 說「去鋼琴旁邊」：回報找不到（工具結果 `not_found`），而且認東西的請求不超過 32 次。
  途中存視窗截圖 `WS8/runs/objects_live.png`（地圖上要看得到物品標記）。需要 5/5；不過時依原因修正（程式錯誤先補會失敗的測試），修完整輪重跑。
- [ ] **Step 4：`stack.sh stop`；寫文件**：`AGENT/README.md` 加「認東西」一節（怎麼用：按巡邏或說「巡邏一圈」、「去〈東西〉旁邊」、「你看過哪些東西」、「忘掉看過的東西」；記憶檔位置；只有視窗版有；額度；實測結果；限制）；檔案表加 `vision.py`、`objects.py`、`eyes.py`；啟動說明註明 Isaac Sim 不能加 `--no-camera`（深度也來自相機）。根目錄 `README.md` 的視窗版說明加一句。spec 加「結果」一節。用 `objects_live.png` 取代 `AGENT/gui.png`，並用 SendUserFile 傳給使用者。
- [ ] **Step 5：`python3 -m pytest test -q` 通過；`cd $ISAAC && python3 -m pytest tests -q` 通過；key 外洩檢查；Commit**：`robot_dog_agent: object memory verified live, README`
