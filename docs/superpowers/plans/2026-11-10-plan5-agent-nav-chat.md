# 計畫 5：用自然語言指揮機器狗（Gemini agent）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增套件 `robot_dog_agent`：在終端機用中文下指令，Gemini 透過四個工具把目標送給 Nav2；定位模式下 `agent_eval` 連續兩次 6/6。

**Architecture:** 純 Python 的三層（`places` 幾何與目標檢查、`tools` 工具宣告與執行、`agent` 對話迴圈）只依賴兩個小介面：機器人（`pose`、`go_to`）和 LLM（`send`、`reset`）。`gemini.py` 用 `requests` 實作 LLM 介面（Interactions API），`nav2_robot.py` 用 `BasicNavigator` 實作機器人介面。測試換成 `FakeRobot` 和照劇本回應的假 LLM。

**Tech Stack:** Python 3.12、`requests`、ROS 2 Jazzy（rclpy、nav2_simple_commander、tf2_ros）、Gemini Interactions API（REST）、pytest

**Spec:** `docs/superpowers/specs/2026-11-10-agent-nav-chat-design.md`

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`AGENT` = `~/dog_12dof_ws/robot_dog_agent`，`NAV` = `~/dog_12dof_ws/robot_dog_navigation`，`WS5` = `~/dog_12dof_ws/.superpowers/sdd/2026-11-10-plan5-agent-nav-chat`（拋棄式腳本和 log）。
- 純 Python 測試：`cd $AGENT && python3 -m pytest test -q`，不 source ROS、不需要網路。`places.py`、`robot.py`、`tools.py`、`agent.py`、`eval_cases.py` 不可匯入 rclpy 或 requests。
- ROS 端：`source /opt/ros/jazzy/setup.bash && cd ~/dog_12dof_ws && colcon build --symlink-install --packages-select robot_dog_agent && source install/setup.bash`。
- **API key**：使用者把 `export GEMINI_API_KEY=...` 放在 `~/.config/robot_dog/gemini.env`（權限 600），並在 `~/.bashrc` 加 `source ~/.config/robot_dog/gemini.env`。本計畫需要 key 的指令前面都先 `. ~/.config/robot_dog/gemini.env`。程式只讀環境變數 `GEMINI_API_KEY`。key 不可被印出、不可進 repo；每次 commit 測試資料前跑 `grep -rF "$GEMINI_API_KEY" $AGENT && echo LEAK`，必須沒有輸出。
- Gemini：`POST https://generativelanguage.googleapis.com/v1beta/interactions`，標頭 `x-goog-api-key`，預設模型 `gemini-3.6-flash`。不裝 SDK。
- 不改 `robot_dog_description`、Nav2 參數、地圖。`robot_dog_navigation` 只被匯入，不修改。
- 門檻（照 spec）：`MIN_CLEARANCE = 0.30` m；導航逾時 120 s（sim time）；一個指令最多 8 輪；`429`／`5xx`／連線錯誤等 10、20、40 s 最多重試 3 次；HTTP 逾時 60 s；到達判定 0.25 m / 0.3 rad；跌倒 = 真值機身高度 < 0.12 m。
- Isaac Sim 同一時間只跑一個；啟動前先確認使用者沒有開著。背景程序以 PID 停止，不用 `pkill -f`。
- `.vscode/settings.json` 和 `robot_dog_navigation/rviz/nav.rviz` 是使用者的修改，不帶進任何 commit（一律 `git commit -- <本計畫的路徑>`）。
- 程式註解和工具回傳的字串用英文；印給使用者看的訊息、文件用繁體中文。commit 結尾 `Co-Authored-By: Claude <noreply@anthropic.com>`。

## Review Focus

1. **模型給的參數不乾淨**：數字寫成字串、少了必要參數、多了不認識的參數、NaN。期待工具回 `error` 和原因，程式不當掉，狗不動（Task 2 `test_bad_arguments_return_error`）。
2. **一句話做到一半出錯**：等 Gemini 回應時按 Ctrl+C、重試用完、或超過 8 輪，伺服器上的對話會停在沒有結果的工具呼叫。期待下一句話照常能用（Task 3 `test_conversation_is_reset_after_an_exception`、`test_too_many_rounds`）。
3. **模型的回應形狀不如預期**：同時有文字和工具呼叫，或兩者都沒有。期待前者照常執行呼叫，後者回固定的訊息而不是空白（Task 3 `test_text_with_calls`、`test_empty_reply`）。
4. **使用者輸入空白行**：期待不送出請求、不花額度（Task 5 `test_blank_lines_are_not_sent`）。
5. **還沒定位好就下指令**：TF 還沒有 map→base_footprint。期待工具回 `error`，不是 traceback（Task 2 `test_robot_without_a_pose`）。

---

### Task 1：套件骨架與地點幾何（`places.py`）

**Files:**
- Create: `AGENT/package.xml`、`setup.py`、`setup.cfg`、`resource/robot_dog_agent`、`robot_dog_agent/__init__.py`（照 `NAV` 的寫法；`exec_depend`：rclpy、geometry_msgs、tf2_ros、nav2_simple_commander、robot_dog_navigation、python3-requests、python3-numpy、python3-scipy；`console_scripts` 先留空 list）
- Create: `AGENT/robot_dog_agent/places.py`
- Create: `AGENT/test/conftest.py`（把 `NAV` 的 checkout 路徑插進 `sys.path`，測試才能匯入 `robot_dog_navigation`）
- Test: `AGENT/test/test_places.py`

**Interfaces:**
- Consumes: `robot_dog_navigation.nav_goals.load_truth(name)`、`clearance(truth, x, y)`、`reachable(truth, x, y)`；`robot_dog_navigation.map_check.to_cell(res, origin, x, y)`；truth = `(grid, res, origin)`，`grid == 100` 是障礙。
- Produces:
  - `Place = namedtuple('Place', 'id name x y yaw')`（`yaw` 弧度，`name` 中文）
  - `PLACES: tuple[Place, ...]`：spec 第 4 節的八列，順序相同
  - `MIN_CLEARANCE = 0.30`
  - `load_body_truth()`：`load_truth('apartment_body')`
  - `find_place(place_id: str) -> Place | None`
  - `relative_target(pose, forward=0.0, left=0.0, turn_left_deg=0.0) -> (x, y, yaw)`：`pose = (x, y, yaw)`；回傳的 yaw 正規化到 (−π, π]
  - `check_target(truth, x, y) -> str | None`：依序回 `'outside the map'`、`'on an obstacle'`、`'not connected to the free space the robot is in (inside furniture or a closed-off area)'`、`'only %.2f m from an obstacle, needs 0.30 m'`，都通過回 `None`
  - `area_of(x, y) -> str`：`'bathroom'`（公寓座標 x ≥ 7.8 且 y ≤ 2.1）、`'living, dining and kitchen area'`（y < 3.0）、`'master bedroom'`（x < 5.0）、`'second bedroom'`；公寓座標 = 世界座標 + (3.5, 1.8)
  - `nearest_place(x, y) -> (Place, float)`

- [ ] **Step 1：寫失敗的測試** `test_places.py`

```python
import itertools, math
from robot_dog_agent import places
from robot_dog_navigation.nav_goals import clearance, reachable

TRUTH = places.load_body_truth()

def test_place_table():
    assert [p.id for p in places.PLACES] == ['start', 'living_room', 'master_bedroom', 'second_bedroom',
                                             'dining_table', 'kitchen', 'entrance', 'bathroom_door']
    k = places.find_place('kitchen')
    assert (k.x, k.y) == (2.5, -0.6) and abs(k.yaw + math.pi / 2) < 1e-9 and places.find_place('fridge') is None

def test_places_are_clear_reachable_and_apart():
    for p in places.PLACES:
        assert clearance(TRUTH, p.x, p.y) >= 0.45 and reachable(TRUTH, p.x, p.y), p.id
        assert places.check_target(TRUTH, p.x, p.y) is None, p.id
    assert min(math.hypot(a.x - b.x, a.y - b.y) for a, b in itertools.combinations(places.PLACES, 2)) >= 0.7

def test_relative_target():
    x, y, yaw = places.relative_target((1.0, 2.0, math.pi / 2), forward=1.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9
    x, y, yaw = places.relative_target((0.0, 0.0, 0.0), left=0.5, turn_left_deg=-90)
    assert abs(x) < 1e-9 and abs(y - 0.5) < 1e-9 and abs(yaw + math.pi / 2) < 1e-9
    assert abs(places.relative_target((0, 0, math.radians(170)), turn_left_deg=30)[2] - math.radians(-160)) < 1e-9

def test_check_target_reasons():
    assert places.check_target(TRUTH, 0.0, 0.0) is None
    assert places.check_target(TRUTH, 20.0, 0.0) == 'outside the map'
    assert places.check_target(TRUTH, -3.45, 0.0) == 'on an obstacle'           # the west wall
    assert places.check_target(TRUTH, 0.4, 2.7).startswith('not connected')     # the middle of the bed
    assert places.check_target(TRUTH, 0.0, -0.55) == 'only 0.25 m from an obstacle, needs 0.30 m'   # by the sofa

def test_area_and_nearest():
    assert places.area_of(0.0, 0.0) == 'living, dining and kitchen area'
    assert places.area_of(-1.5, 3.0) == 'master bedroom' and places.area_of(3.0, 2.2) == 'second bedroom'
    assert places.area_of(5.0, -1.0) == 'bathroom'
    place, dist = places.nearest_place(2.4, -0.6)
    assert place.id == 'kitchen' and abs(dist - 0.1) < 1e-9
```

- [ ] **Step 2：跑測試確認失敗**：`cd $AGENT && python3 -m pytest test/test_places.py -q` → `ModuleNotFoundError: robot_dog_agent.places`
- [ ] **Step 3：實作 `places.py`**。`check_target` 用 `to_cell` 判斷範圍和障礙格，再用 `reachable`、`clearance`。測試裡的四個點已對真值地圖確認過（西牆是障礙格、床中央不連通、沙發前淨空 0.25 m）。
- [ ] **Step 4：跑測試確認通過**，再 `colcon build --symlink-install --packages-select robot_dog_agent` 成功。
- [ ] **Step 5：Commit**（`AGENT` 全部新檔）：`robot_dog_agent: package skeleton, place table and target checks`

### Task 2：機器人介面與工具（`robot.py`、`tools.py`）

**Files:**
- Create: `AGENT/robot_dog_agent/robot.py`、`AGENT/robot_dog_agent/tools.py`
- Test: `AGENT/test/test_tools.py`

**Interfaces:**
- Consumes: Task 1 的 `places`。
- Produces（`robot.py`，純 Python）：
  - `NavResult = namedtuple('NavResult', 'status reason seconds')`，`status` 是 `'reached'`、`'failed'`、`'timeout'`、`'canceled'` 之一
  - 機器人介面：`pose() -> (x, y, yaw)`（拿不到時丟 `RuntimeError`）；`go_to(x, y, yaw, on_progress=None) -> NavResult`
  - `class FakeRobot(pose=(0.0, 0.0, 0.0), results=())`：`go_to` 把 `(x, y, yaw)` 記進 `self.goals`；`results` 還有就依序取用，否則回 `NavResult('reached', '', 1.0)`；只有 `reached` 才把位置改成目標；`pose_error: str | None` 設了以後 `pose()` 丟 `RuntimeError`
- Produces（`tools.py`）：
  - `DECLARATIONS: list[dict]`：四個 `{"type": "function", "name", "description", "parameters"}`，見下表；`go_to_place` 的 `place` 是 `enum`，值由 `PLACES` 產生
  - `system_instruction() -> str`：下方的文字，地點那一段由 `PLACES` 產生
  - `class Toolbox(robot, truth, on_progress=None)`，`call(name: str, args: dict) -> dict`：永遠不丟例外（`KeyboardInterrupt` 除外）；回傳 spec 5.1 的 dict，`x`、`y` 四捨五入到 2 位、`yaw_deg` 到整數、`seconds` 到 1 位

| 工具 | 參數（JSON Schema） | description |
|---|---|---|
| `get_pose` | 無 | Where the robot is now: position, heading, area and nearest named place. Does not move the robot. |
| `go_to_place` | `place`: string enum，required | Walk to a named place and face the way listed for it. |
| `go_to_pose` | `x`、`y`: number，required；`yaw_deg`: number | Walk to map coordinates in metres. Without yaw_deg the robot ends facing its direction of travel. |
| `move_relative` | `forward_m`、`left_m`、`turn_left_deg`: number | Move relative to the current pose: forward_m (negative = back), left_m (negative = right), turn_left_deg (negative = right turn). |

`system_instruction()` 的文字（地點列由程式產生，格式 `- id: name, (x, y)`）：

```
You control a quadruped robot dog in a simulated apartment through the tools. The user types commands in Chinese.

Coordinates: map frame, metres. The dog started at (0, 0) facing +x; +y is to its left when it faces +x.
Angles are degrees, counter-clockwise (a left turn) positive.

Places (id: name, x, y):
- start: 起點（客廳中央，沙發和電視櫃之間）, (0.0, 0.0)
...

Rules:
- For a named place always call go_to_place with its id. Never guess coordinates for a place or an object that is
  not in the list; say you do not know where it is.
- For forward / back / left / right / turn commands call move_relative.
- For several steps in one command, call the tools in the order the user gave.
- When a tool returns status "blocked", tell the user the reason. Do not try a nearby point unless the user asks.
- When a tool returns any status other than "ok" or "reached", stop and tell the user what happened.
- Do not call a navigation tool for a question that needs no movement. Use get_pose to answer where the dog is.
- Reply in Traditional Chinese (Taiwan), in one or two short sentences.
```

行為細節：
- 數字參數必須是 `int` 或 `float`（不含 `bool`）且有限；否則 `error`，`reason` 指出是哪個參數。不認識的參數忽略。
- `go_to_pose` 沒給 `yaw_deg`：朝向 = 目前位置指向目標的方位；距離 < 0.1 m 時用目前朝向。
- `move_relative`：三個都是 0 回 `error`（`'nothing to do'`）；`forward_m`、`left_m` 超過 ±5 回 `error`。
- 目標沒通過 `check_target` 回 `blocked`，`reason` 是 `check_target` 的字串，`go_to` 不被呼叫。
- `NavResult.status` 直接當 `status`；`x`、`y`、`yaw_deg` 是結束後的 `robot.pose()`。
- 三個導航工具一開始都先讀 `robot.pose()`（`blocked` 的位置欄位和相對移動都要用）。它丟 `RuntimeError` 時回 `error`，`reason` 是例外訊息，不含位置欄位，`go_to` 不被呼叫。
- `get_pose`：`{"status": "ok", "x", "y", "yaw_deg", "area", "nearest_place": id, "distance_to_place"}`。

- [ ] **Step 1：寫失敗的測試** `test_tools.py`（`TRUTH = places.load_body_truth()`；`box(robot) = Toolbox(robot, TRUTH)`）

```python
def test_declarations_match_the_place_table():
    by_name = {d['name']: d for d in tools.DECLARATIONS}
    assert set(by_name) == {'get_pose', 'go_to_place', 'go_to_pose', 'move_relative'}
    assert all(d['type'] == 'function' and d['description'] for d in tools.DECLARATIONS)
    assert by_name['go_to_place']['parameters']['properties']['place']['enum'] == [p.id for p in places.PLACES]
    assert by_name['go_to_pose']['parameters']['required'] == ['x', 'y']
    text = tools.system_instruction()
    assert all(p.id in text and p.name in text for p in places.PLACES) and 'Traditional Chinese' in text

def test_get_pose():
    r = box(FakeRobot((2.4, -0.6, math.pi))).call('get_pose', {})
    assert r == {'status': 'ok', 'x': 2.4, 'y': -0.6, 'yaw_deg': 180, 'area': 'living, dining and kitchen area',
                 'nearest_place': 'kitchen', 'distance_to_place': 0.1}

def test_go_to_place_uses_the_table():
    robot = FakeRobot()
    r = box(robot).call('go_to_place', {'place': 'master_bedroom'})
    assert robot.goals == [(-1.5, 3.0, math.pi / 2)]
    assert r == {'status': 'reached', 'x': -1.5, 'y': 3.0, 'yaw_deg': 90, 'seconds': 1.0}

def test_go_to_pose_faces_the_direction_of_travel():
    robot = FakeRobot()
    box(robot).call('go_to_pose', {'x': 0.0, 'y': 0.3})
    box(robot).call('go_to_pose', {'x': 0.05, 'y': 0.3})           # closer than 0.1 m: keep the heading
    box(robot).call('go_to_pose', {'x': -0.5, 'y': 0.3, 'yaw_deg': 0})
    assert [round(g[2], 6) for g in robot.goals] == [round(math.pi / 2, 6), round(math.pi / 2, 6), 0.0]

def test_move_relative_starts_from_the_current_pose():
    robot = FakeRobot((0.0, 0.0, math.pi / 2))
    box(robot).call('move_relative', {'forward_m': 0.3, 'turn_left_deg': 90})
    x, y, yaw = robot.goals[0]
    assert abs(x) < 1e-9 and abs(y - 0.3) < 1e-9 and abs(abs(yaw) - math.pi) < 1e-9

def test_blocked_target_never_reaches_the_robot():
    robot = FakeRobot()
    r = box(robot).call('go_to_pose', {'x': 0.4, 'y': 2.7})        # the middle of the bed
    assert r['status'] == 'blocked' and r['reason'].startswith('not connected') and (r['x'], r['y']) == (0.0, 0.0)
    assert robot.goals == []

def test_navigation_failure_is_reported_with_the_final_pose():
    robot = FakeRobot(results=[NavResult('timeout', 'no arrival within 120 s', 120.0)])
    r = box(robot).call('go_to_place', {'place': 'kitchen'})
    assert r == {'status': 'timeout', 'reason': 'no arrival within 120 s', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0,
                 'seconds': 120.0}

def test_bad_arguments_return_error():
    robot = FakeRobot()
    for name, args in [('go_to_place', {}), ('go_to_place', {'place': 'fridge'}), ('go_to_pose', {'x': '1', 'y': 0}),
                       ('go_to_pose', {'x': 1.0}), ('go_to_pose', {'x': float('nan'), 'y': 0.0}),
                       ('go_to_pose', {'x': True, 'y': 0.0}), ('move_relative', {}),
                       ('move_relative', {'forward_m': 6.0}), ('fly', {}), ('go_to_pose', None)]:
        r = box(robot).call(name, args)
        assert r['status'] == 'error' and r['reason'], (name, args)
    assert robot.goals == []
    assert box(robot).call('move_relative', {'forward_m': 0.2, 'speed': 9})['status'] == 'reached'   # extras ignored

def test_robot_without_a_pose():
    robot = FakeRobot()
    robot.pose_error = 'no transform from map to base_footprint yet'
    for name, args in [('get_pose', {}), ('move_relative', {'forward_m': 0.2}), ('go_to_place', {'place': 'kitchen'})]:
        assert box(robot).call(name, args) == {'status': 'error', 'reason': 'no transform from map to base_footprint yet'}
    assert robot.goals == []
```

- [ ] **Step 2：跑測試確認失敗**（`ModuleNotFoundError`）
- [ ] **Step 3：實作 `robot.py` 和 `tools.py`**
- [ ] **Step 4：跑 `python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: robot interface, FakeRobot and the four tools`

### Task 3：對話迴圈（`agent.py`）

**Files:**
- Create: `AGENT/robot_dog_agent/agent.py`
- Test: `AGENT/test/test_agent.py`

**Interfaces:**
- Consumes: `Toolbox.call(name, args) -> dict`。
- Produces:
  - `Call = namedtuple('Call', 'id name args')`、`Reply = namedtuple('Reply', 'text calls')`（`text` 沒有時是 `''`，`calls` 是 list）
  - LLM 介面：`send(text=None, results=None) -> Reply`（`results` 是 `[(Call, dict), ...]`，和 `text` 二擇一）；`reset()`（忘掉對話）
  - `class Agent(llm, toolbox, on_call=None)`，`MAX_ROUNDS = 8`，`ask(text: str) -> str`；`on_call(call, result)` 在每個工具呼叫有結果後被呼叫（包含 `skipped`）
  - `TOO_MANY_ROUNDS = '這個指令用了太多步驟，我先停下來。'`、`EMPTY_REPLY = '（模型沒有回覆，請再說一次。）'`

行為：
- 一輪 = 一次 `llm.send`。`Reply.calls` 是空的：回 `text`，是空字串就回 `EMPTY_REPLY`。
- 有 calls：依序 `toolbox.call`；某個結果的 `status` 不是 `'ok'` 或 `'reached'` 之後，其餘的不執行，結果是 `{'status': 'skipped', 'reason': 'an earlier step did not succeed'}`；全部結果用 `send(results=...)` 送回。同一個 Reply 裡的 `text` 忽略。
- 第 8 次 `send` 的 Reply 還有 calls：不執行，`llm.reset()`，回 `TOO_MANY_ROUNDS`。
- `ask` 裡任何例外（含 `KeyboardInterrupt`）：先 `llm.reset()` 再原樣丟出。

- [ ] **Step 1：寫失敗的測試** `test_agent.py`。測試用的 `ScriptedLLM(replies)`：`send` 把 `(text, results)` 記進 `self.sent` 並依序回 `replies`（項目是例外就丟出），`reset` 計數。`RecordingToolbox(results_by_name)`：記下呼叫並回對應的 dict。

```python
KITCHEN, START = Call('c1', 'go_to_place', {'place': 'kitchen'}), Call('c2', 'go_to_place', {'place': 'start'})
REACHED, FAILED = {'status': 'reached'}, {'status': 'failed', 'reason': 'Nav2 status FAILED'}

def test_text_only():                       # one send, no tool call, reply returned as is
def test_one_call_then_text():              # sent == [('去廚房', None), (None, [(KITCHEN, REACHED)])]; returns the text
def test_two_calls_in_one_reply_run_in_order():   # toolbox.calls == [kitchen, start]; both results in one send
def test_later_calls_are_skipped_after_a_failure():
    # kitchen -> FAILED: start is not executed, its result is {'status': 'skipped', 'reason': 'an earlier step did not succeed'}
def test_get_pose_ok_does_not_skip_the_next_call():   # {'status': 'ok', ...} then a navigation call: both run
def test_text_with_calls():                 # Reply('我現在過去', [KITCHEN]): the call runs, final reply is the later text
def test_empty_reply():                     # Reply('', []) -> agent.EMPTY_REPLY
def test_too_many_rounds():
    # 8 replies that each hold a call: ask returns TOO_MANY_ROUNDS, len(llm.sent) == 8, toolbox ran 7 calls, llm.resets == 1
def test_on_call_sees_every_result():       # including the skipped one
def test_conversation_is_reset_after_an_exception():
    # replies [Reply('', [KITCHEN]), RuntimeError('rate limited'), Reply('好', [])]: first ask raises RuntimeError and
    # llm.resets == 1; the second ask returns '好'. Same with KeyboardInterrupt raised by the toolbox.
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `agent.py`** → **Step 4：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 5：Commit**：`robot_dog_agent: the tool-call loop`

### Task 4：Gemini 客戶端（`gemini.py`）

這個 Task 的 Step 1 不依賴 Task 1～3 以外的東西，但需要使用者已經設好 key。還沒設好就停下來問使用者，不要用假的範例檔往下做。

**Files:**
- Create: `WS5/bin/probe_gemini.py`（拋棄式，不 commit）
- Create: `AGENT/test/data/{text,call,after_result,two_calls,error_previous_id,error_key}.json`
- Create: `AGENT/robot_dog_agent/gemini.py`
- Modify: spec 第 2 節（實測和文件不同的地方）
- Test: `AGENT/test/test_gemini.py`

**Interfaces:**
- Consumes: `Call`、`Reply`（Task 3）；`tools.DECLARATIONS`、`tools.system_instruction()`（Task 2）。
- Produces:
  - `class GeminiError(Exception)`：訊息是給使用者看的中文加上伺服器的原文
  - `build_request(model, system_instruction, tools, text, results, previous_id) -> dict`（純函式）
  - `parse_reply(body: dict) -> (interaction_id: str, Reply)`（純函式）
  - `class GeminiClient(api_key, model, system_instruction, tools, post=requests.post, sleep=time.sleep, say=print)`：實作 LLM 介面 `send`、`reset`；`URL`、`RETRY_WAITS = (10, 20, 40)`、`TIMEOUT = 60`

- [ ] **Step 1：探測真的 API，錄下範例檔**。`probe_gemini.py` 用 `requests` 直接送六個請求，把 `{"request": 請求 JSON, "http_status": 狀態碼, "retry_after": 標頭值或 null, "body": 回應 JSON}` 寫進 `AGENT/test/data/`：

| 檔名 | 請求 |
|---|---|
| `text.json` | `input`: `"用一句話打招呼"`，帶 system instruction 和四個工具 |
| `call.json` | `input`: `"去廚房"` |
| `after_result.json` | `previous_interaction_id` = `call.json` 的 id，`input` = 一個 `function_result`，`result` 的文字是 `{"status": "reached", "x": 2.5, "y": -0.6, "yaw_deg": -90, "seconds": 20.0}` |
| `two_calls.json` | `input`: `"先去廚房，再回起點"`（看是一次給兩個呼叫還是一次一個） |
| `error_previous_id.json` | `previous_interaction_id`: `"does-not-exist"`，`input`: `"哈囉"` |
| `error_key.json` | 標頭的 key 換成 `"invalid-key"`，`input`: `"哈囉"` |

  執行：`. ~/.config/robot_dog/gemini.env && cd $AGENT && python3 $WS5/bin/probe_gemini.py`。前四個必須是 200，`call.json` 必須含一個 `go_to_place`、`place` = `kitchen` 的工具呼叫。請求被 `400` 拒絕時，照錯誤訊息和官方文件（`https://aistudio.google.com/docs/interactions`、`.../docs/function-calling`）修正欄位名稱，直到通過；把和 spec 第 2 節不同的地方改進 spec。`call.json` 沒有呼叫工具時，先改 `system_instruction()` 或工具說明再錄。跑 key 外洩檢查（Global Constraints）。
- [ ] **Step 2：寫失敗的測試** `test_gemini.py`（`load(name)` 讀 `test/data/<name>.json`；`FakePost(responses)` 記下每次的 `url`、`headers`、`json`、`timeout`，依序回有 `status_code`、`headers`、`json()`、`text` 的物件，項目是例外就丟出）

```python
def test_build_request_matches_what_the_api_accepted():
    for name in ('text', 'call', 'after_result'):          # rebuild each recorded request from its parts
        rec = load(name)['request']
        ...  # build_request(...) == rec

def test_parse_recorded_replies():
    iid, reply = gemini.parse_reply(load('text')['body'])
    assert iid and reply.text.strip() and reply.calls == []
    iid, reply = gemini.parse_reply(load('call')['body'])
    assert [(c.name, c.args) for c in reply.calls] == [('go_to_place', {'place': 'kitchen'})] and reply.calls[0].id
    assert gemini.parse_reply(load('after_result')['body'])[1].text.strip()
    names = [c.name for c in gemini.parse_reply(load('two_calls')['body'])[1].calls]
    assert names and set(names) == {'go_to_place'}        # one call per reply or both at once: either is handled

def test_key_goes_in_the_header_only():
    # one send: post.calls[0].url == GeminiClient.URL, headers['x-goog-api-key'] == 'k', 'k' not in the url, timeout == 60

def test_conversation_is_chained_and_reset():
    # second send carries previous_interaction_id == id of the first reply; after reset() the next one carries none

def test_retries_after_429_and_5xx():
    # responses [429 with Retry-After '7', 503, recorded text reply]: sleeps == [7, 20], say() called twice with '被限流' / '伺服器'
def test_gives_up_after_three_retries():     # four 429s -> GeminiError, sleeps == [10, 20, 40]
def test_connection_errors_are_retried():    # requests.ConnectionError then a reply -> the reply
def test_bad_key_is_not_retried_and_not_leaked():
    # recorded error_key response: GeminiError, one post call, the client's api_key string is not in str(error)
def test_expired_conversation_starts_over():
    # first send ok; second gets the recorded error_previous_id response, then a reply: the third post has no
    # previous_interaction_id, say() got '之前的對話已過期，重新開始', send returns the reply
def test_non_json_error_body():              # 502 with an HTML body three times then success -> the reply, no crash
```

- [ ] **Step 3：跑測試確認失敗** → **Step 4：實作 `gemini.py`**。「對話已過期」的判斷用 `error_previous_id.json` 錄到的狀態碼和錯誤字樣，且只在這次請求帶了 `previous_interaction_id` 時才成立；每次 `send` 最多重開一次。`400`／`401`／`403`／`404` 不重試。
- [ ] **Step 5：`python3 -m pytest test -q` 全部通過**
- [ ] **Step 6：Commit**（`gemini.py`、測試、六個範例檔、spec）：`robot_dog_agent: Gemini Interactions API client, tested against recorded replies`

### Task 5：Nav2 機器人與終端機對話（`nav2_robot.py`、`chat.py`）

**Files:**
- Create: `AGENT/robot_dog_agent/nav2_robot.py`、`AGENT/robot_dog_agent/chat.py`
- Modify: `AGENT/setup.py`（`console_scripts`: `chat = robot_dog_agent.chat:main`）
- Create: `WS5/bin/stack.sh`（從計畫 4 的 `bin/stack.sh` 複製，拋棄式）
- Test: `AGENT/test/test_chat.py`

**Interfaces:**
- Consumes: 前四個 Task 的全部；`robot_dog_navigation.nav_eval` 的 `make_pose(nav, x, y, yaw)`、`yaw_of(q)`、`sim_seconds(nav)`。
- Produces（`nav2_robot.py`，匯入 rclpy）：
  - `class Nav2Robot(slam=False, timeout=120.0)`：`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`；`BasicNavigator` 設 `use_sim_time`；TF 用獨立節點加 `TransformListener(..., spin_thread=True)`（`nav_eval.main` 的寫法）；`waitUntilNav2Active(localizer='robot_localization' if slam else 'amcl')`
  - 屬性 `node`（`BasicNavigator`）、`tf_buffer`
  - `pose()`：最新的 map→base_footprint；等最多 5 s（牆上時間）還沒有就丟 `RuntimeError('no transform from map to base_footprint yet')`
  - `go_to(x, y, yaw, on_progress=None) -> NavResult`：目標被拒絕回 `('failed', 'Nav2 rejected the goal', 0.0)`；迴圈裡每 2 s（牆上時間）用 `getFeedback().distance_remaining` 呼叫 `on_progress(distance)`；超過 `timeout`（sim time）取消並回 `'timeout'`；`KeyboardInterrupt` 時 `cancelTask()`、等任務結束，回 `('canceled', 'canceled by the user', t)`；Nav2 結果 SUCCEEDED → `'reached'`，其他 → `('failed', 'Nav2 status <NAME>', t)`
  - `close()`：照 `nav_eval.main` 的 `finally` 順序收尾
- Produces（`chat.py`）：
  - `build_agent(robot, api_key, model, say=print, on_call=None) -> Agent`：組起 `Toolbox`（`on_progress` 印 `'  剩 %.1f m'`）、`GeminiClient`、`Agent`
  - `run_chat(agent, read=input, say=print) -> None`：讀輸入的迴圈
  - `main() -> int`：`--slam`、`--model`（預設 `gemini-3.6-flash`）、`--no-robot`

`run_chat` 的行為：提示字元 `'你> '`；空白行不送；`exit`、`quit`、`EOFError`、提示下的 `KeyboardInterrupt` 結束迴圈；`agent.ask` 丟 `GeminiError` 印 `'[錯誤] <訊息>'` 後繼續；`ask` 途中的 `KeyboardInterrupt` 印 `'（已取消）'` 後繼續；回覆印成 `'狗> <回覆>'`。
`main`：沒有 `GEMINI_API_KEY` 時印出申請網址 `https://aistudio.google.com/` 和設定方法（Global Constraints 的兩行），回傳 2，不初始化 ROS。啟動後印地點清單（id 和中文名稱）和 `'Ctrl+C 可以中斷導航；輸入 exit 結束。'`。`on_call` 印 `'  [工具] <name> <args> → <status>'`。`--no-robot` 用 `FakeRobot()`，不匯入 `nav2_robot`。

- [ ] **Step 1：寫失敗的測試** `test_chat.py`（`FakeAgent(answers)`：記下 `asked`，依序回字串或丟出例外）

```python
def test_blank_lines_are_not_sent():         # reads ['', '   ', '去廚房', 'exit'] -> agent.asked == ['去廚房']
def test_reply_is_printed():                 # say() got '狗> 到了'
def test_exit_words_and_end_of_input():      # 'exit', 'quit', EOFError and KeyboardInterrupt from read() each end the loop
def test_errors_do_not_end_the_chat():       # GeminiError('被限流') then '好': said '[錯誤] 被限流' and '狗> 好'
def test_ctrl_c_during_a_command():          # ask raises KeyboardInterrupt: said '（已取消）', next line is still read
def test_main_without_a_key(monkeypatch, capsys):
    # GEMINI_API_KEY unset, argv ['chat', '--no-robot']: main() == 2, output mentions GEMINI_API_KEY and aistudio.google.com
def test_build_agent_with_the_fake_robot():
    # build_agent(FakeRobot(), 'k', 'gemini-3.6-flash') returns an Agent whose toolbox answers get_pose with status 'ok'
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作 `chat.py` 和 `nav2_robot.py`** → **Step 4：`python3 -m pytest test -q` 全部通過，`colcon build` 成功**
- [ ] **Step 5：不開模擬器試對話**：`. ~/.config/robot_dog/gemini.env && source install/setup.bash && printf '去廚房\n你現在在哪裡\n往前走一公尺\nexit\n' | ros2 run robot_dog_agent chat --no-robot`。預期三句各有 `[工具]` 行和中文回覆；第一句呼叫 `go_to_place {'place': 'kitchen'}`，第三句呼叫 `move_relative`（從廚房面向流理台往前一公尺會被擋，回覆要說明原因）。
- [ ] **Step 6：開模擬器試一次**：確認沒有其他 Isaac Sim 在跑；`$WS5/bin/stack.sh start False`（AMCL）；`printf '去客廳西側\nexit\n' | ros2 run robot_dog_agent chat`。預期印出剩餘距離、`→ reached`、中文回覆，`/odom_gt` 的位置在 (−2.5, −0.3) 的 0.25 m 內。再手動測一次 Ctrl+C：送出 `去主臥室` 後 5 秒對 `chat` 程序送 `SIGINT`，預期印出 `canceled`、狗停下（`/cmd_vel` 歸零）、程序還活著。結束後 `stack.sh stop`。
- [ ] **Step 7：Commit**：`robot_dog_agent: Nav2 robot and the terminal chat`

### Task 6：實機檢查（`eval_cases.py`、`agent_eval.py`）

**Files:**
- Create: `AGENT/robot_dog_agent/eval_cases.py`（純 Python）、`AGENT/robot_dog_agent/agent_eval.py`
- Modify: `AGENT/setup.py`（`agent_eval = robot_dog_agent.agent_eval:main`）
- Test: `AGENT/test/test_eval_cases.py`

**Interfaces:**
- Consumes: `build_agent`、`Nav2Robot`（Task 5）；`places.find_place`；`robot_dog_navigation.nav_eval.check_localization(nav, truth, buf)`、`Truth`。
- Produces（`eval_cases.py`）：
  - `Observation = namedtuple('Observation', 'before after nav_ends goals_sent reply fell')`：`before`、`after` 和 `nav_ends` 的項目都是真值 `(x, y, yaw)`；`nav_ends` 是每次 `go_to` 結束時的真值；`goals_sent` 是 `go_to` 被呼叫的次數
  - `CASES: tuple[(sentence: str, judge), ...]`，六列照 spec 第 7 節的順序和句子；`judge(obs) -> (ok: bool, why: str)`；`obs.fell` 為真時一律不通過
- Produces（`agent_eval.py`）：`main() -> int`：`--model`；用一個包住 `Nav2Robot` 的計數代理記錄 `goals_sent` 和 `nav_ends`；訂閱 `/odom_gt` 記錄最新真值和這一句期間的最低機身高度；先 `check_localization`；每句印 `[AGENT] <句子> PASS|FAIL <why> | 回覆：<reply>`，最後 `[AGENT] PASS k/6`；全過回傳 0。某一句丟 `GeminiError` 算該句失敗，繼續下一句。

- [ ] **Step 1：寫失敗的測試** `test_eval_cases.py`：每個 judge 一個通過的觀測和各個門檻剛好超過的觀測。

```python
MB, K, S = (places.find_place(i) for i in ('master_bedroom', 'kitchen', 'start'))
def obs(**kw): return Observation(**{**dict(before=(0, 0, 0), after=(0, 0, 0), nav_ends=[], goals_sent=0, reply='好', fell=False), **kw})

def test_sentences():
    assert [s for s, _ in CASES] == ['去主臥室', '先去廚房，再回起點', '往前走一公尺', '向左轉 90 度', '去座標 (0.4, 2.7)', '你現在在哪裡']
def test_case_1_master_bedroom():            # after within 0.25 m / 0.3 rad passes; 0.3 m off fails; 0.4 rad off fails
def test_case_2_kitchen_then_start():        # nav_ends[0] near kitchen and after near start passes; nav_ends empty fails;
                                             # first end 0.5 m from the kitchen fails
def test_case_3_one_metre_forward():         # before (0,0,pi/2) after (0.1,1.1,..) passes; 0.7 m fails; 0.3 m sideways fails
def test_case_4_turn_left():                 # yaw 0 -> 1.5 passes; 0 -> -1.57 fails; 0 -> 1.0 fails; moved 0.3 m fails
def test_case_5_blocked_coordinates():       # goals_sent 0 and not moved passes; goals_sent 1 fails
def test_case_6_where_am_i():                # goals_sent 0, reply '我在起點' passes; empty reply fails
def test_a_fall_fails_every_case():
```

- [ ] **Step 2：跑測試確認失敗** → **Step 3：實作兩個檔案** → **Step 4：`python3 -m pytest test -q` 全部通過，`colcon build` 成功**
- [ ] **Step 5：跑驗收**。每一輪都重啟：`$WS5/bin/stack.sh start False`，然後 `. ~/.config/robot_dog/gemini.env && ros2 run robot_dog_agent agent_eval 2>&1 | tee $WS5/runs/agent_eval_<n>.log`。需要連續兩輪 `[AGENT] PASS 6/6`。不過的時候依原因處理，一次只改一項，改完從第一輪重算：
  - 第 4 句狗沒有轉：在 `Nav2Robot.go_to` 裡，目標離目前位置 < 0.1 m 時改用 `BasicNavigator.spin(spin_dist=<帶正負號的角度差>, time_allowance=30)`，其餘流程相同；更新 spec 5.1 的那一句。
  - 模型選錯工具或參數：改 `system_instruction()` 或工具的 description，並用 `--no-robot` 先確認。還是不穩再把預設模型換成 `gemini-3.8-flash`，更新 spec 第 2 節。
  - 被限流到重試用完：兩輪之間等 60 s 再跑；記下實際遇到的上限，寫進 README。
  - 導航本身失敗（Nav2 FAILED、逾時、跌倒）：不是這個計畫的範圍，記下 log 位置回報使用者，不改 Nav2 參數。
- [ ] **Step 6：`stack.sh stop`，Commit**：`robot_dog_agent: agent_eval, six sentences judged with ground truth`

### Task 7：文件

**Files:**
- Create: `AGENT/README.md`
- Modify: `~/dog_12dof_ws/README.md`（加一節指向 `robot_dog_agent/README.md`，並把套件加進 colcon build 的指令）
- Modify: spec（加「結果」一節：兩輪 `agent_eval` 的結果、實測的請求次數、和設計不同的地方）

- [ ] **Step 1：寫 `AGENT/README.md`**（繁體中文），內容依序：這是什麼（兩句）；申請 key 和設定（Global Constraints 的做法；提醒 key 不要進 repo、免費額度的內容會被 Google 用來改進產品）；啟動順序（Isaac Sim → `nav.launch.py` → `ros2 run robot_dog_agent chat`）；能說的話（Task 6 的六句加兩三個變化）；地點表；Ctrl+C 和 `exit`；`--no-robot`、`--model`、`--slam`；`agent_eval` 的用法和實測結果；已知限制（不認得家具、浴室裡不放地點、免費額度的速率上限、即時率約 0.5）。
- [ ] **Step 2：改根目錄 README 和 spec**
- [ ] **Step 3：照 README 的指令從頭走一次**（`--no-robot` 那一段實際執行；需要模擬器的那一段對照 Task 5、6 已跑過的指令逐字核對）
- [ ] **Step 4：`cd $AGENT && python3 -m pytest test -q`、`cd $NAV && python3 -m pytest test -q` 都通過；Commit**：`robot_dog_agent: README, results in the spec`
