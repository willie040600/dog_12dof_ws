# 計畫 1：環境確認 + Isaac Sim 模組重構 + RL 步態訓練（M0 + M1）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 確認這台機器上 ROS 2 ↔ Isaac Sim 能互通，把 Isaac Sim 腳本拆成可重用的模組，並訓練、評估、匯出一個通過 `gait_eval` 的 12 自由度步態策略（`policy.pt` + `policy.json`）。

**Architecture:** Isaac Sim 端的程式放在 `robot_dog_description/isaac/robot_dog_sim/`：純 Python 模組（`util`、`robot_spec`、`policy_meta`）可以用系統 python3 測試；碰到 `omni`/`pxr` 的模組只在 `SimulationApp` 建立後才 import。Isaac Lab 端沿用舊 8 自由度狗的做法：`isaaclab/scripts/*.py` 把 `isaaclab/` 加進 `sys.path`，import `robot_dog_lab.tasks` 註冊任務，再呼叫 Isaac Lab 的 CLI 進入點，在 `~/IsaacLab` 的 uv 環境執行。

**Tech Stack:** Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.2.0（pip 版，在 `~/IsaacLab/.venv`）、Isaac Lab 3.0.0、rsl_rl 5.5.1、Python 3.12、pytest 7.4（系統 python3）

**Spec:** `docs/superpowers/specs/2026-11-01-robot-dog-isaac-nav2-design.md`（本計畫實作第 4 節、第 5.1 節的模組拆分與 `policy_meta.py`，以及第 7 節的 M0、M1）

## Global Constraints

- 工作區：`~/dog_12dof_ws`（git repo，分支 `main`）。下文 `DESC` = `~/dog_12dof_ws/robot_dog_description`。
- Isaac 相關指令一律在 `~/IsaacLab` 下執行，前置：`export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1`，執行用 `uv run --extra isaacsim python <script>`。這台機器沒有 `~/isaacsim/python.sh`。
- 需要 ROS 2 的 Isaac Sim 腳本，啟動前要先 `source /opt/ros/jazzy/setup.bash`。
- Isaac Lab 3.0 的 launcher 參數（`add_launcher_args`）**沒有 `--headless`**：預設就不開畫面，要看畫面才加 `--viz kit`。`--headless` 只用在自己建立 `SimulationApp` 的腳本（`robot_dog_isaac.py`、`rclpy_probe.py`）。
- Isaac Sim／Isaac Lab 的 API 以已安裝的原始碼為準：`~/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/`、`~/IsaacLab/source/`，不憑記憶。
- 純 Python 模組（`robot_dog_sim/util.py`、`robot_spec.py`、`policy_meta.py`，`robot_dog_lab/gait_criteria.py`、`checkpoints.py`）不得 import `omni`、`pxr`、`isaacsim`、`isaaclab`、`torch`，用系統 `python3 -m pytest` 測試。
- 其餘 `robot_dog_sim` 模組只能在 `SimulationApp(...)` 建立之後 import。`robot_dog_sim/__init__.py` 保持空白。
- 數值（spec 第 4.2 節）：sim dt 0.005 s、decimation 4、action scale 0.25；站姿 hip 0.0、thigh 0.8、calf −1.5 rad；DC 馬達 effort 25 N·m、saturation 25 N·m、速度 30 rad/s、kp 20、kd 0.5；指令範圍 vx ±0.8、vy ±0.5、wz ±1.0。
- 關節一律依名稱對應，不假設順序。
- 同一時間只跑一個 Isaac 程序（GPU 12 GB）。
- 不修改 `~/robotdog_ws`。Gazebo 相關檔案不動。
- 程式註解用英文（與現有程式一致）；README 與文件用繁體中文。
- commit 訊息最後一行：`Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`

## Review Focus

1. **關節順序**：Isaac Lab articulation 的關節順序可能不是 ROS 的 FL/FR/RL/RR 順序。`policy.json` 的 `joint_names` 必須取自動作項（`joint_pos`）的關節清單；若動作項沒有剛好涵蓋 12 個關節各一次，匯出要中止並印出清單（Task 7，Step 6）。
2. **最新 checkpoint 要依數字排序**：`model_1000.pt` 比 `model_999.pt` 新，不能用字串排序（Task 7，`test_latest_checkpoint_is_numeric`）。
3. **沒 source ROS 就啟動 Isaac Sim 腳本**：應立刻以 exit code 2 結束，並提示 `source /opt/ros/jazzy/setup.bash`，而不是跑到一半 ROS 橋接才失敗（Task 1，`test_require_ros_jazzy_exits_when_unsourced`）。
4. **重複匯出到已存在的資料夾**：沒加 `--force` 時要拒絕，不能默默覆蓋已提交的策略（Task 7，`test_prepare_out_dir_refuses_existing`）。
5. **`policy.json` 寫到一半失敗**：有無法序列化的值（例如 torch tensor）時，不能留下半個 JSON 檔（Task 4，`test_save_leaves_no_file_on_unserializable_value`）。

---

### Task 1：`robot_dog_sim` 套件骨架、ROS 前置檢查、rclpy 共存探針

**Files:**
- Create: `DESC/isaac/robot_dog_sim/__init__.py`（空白）
- Create: `DESC/isaac/robot_dog_sim/util.py`
- Create: `DESC/isaac/tools/rclpy_probe.py`
- Create: `DESC/isaac/tests/conftest.py`
- Test: `DESC/isaac/tests/test_util.py`

**Interfaces:**
- Produces（`robot_dog_sim/util.py`，純 Python）：
  - `log(msg: str) -> None`：印出 `'[robot_dog] ' + msg`，flush。
  - `try_step(label: str, fn: Callable, *args) -> bool`：執行 `fn(*args)`，成功印 `'%-34s ok' % label` 並回傳 True；有例外印 `'%-34s FAILED: <例外>'` 並回傳 False（與現有 `robot_dog_isaac.py` 的 `step()` 相同）。
  - `require_ros_jazzy() -> None`：環境變數 `ROS_DISTRO` 不是 `'jazzy'` 時，印出含 `source /opt/ros/jazzy/setup.bash` 的訊息，並 `raise SystemExit(2)`。
- `tests/conftest.py`：把 `DESC/isaac` 加到 `sys.path[0]`，讓測試能 `import robot_dog_sim.util`。

- [ ] **Step 1：寫失敗的測試** `DESC/isaac/tests/test_util.py`

```python
import pytest
from robot_dog_sim.util import require_ros_jazzy, try_step

def test_require_ros_jazzy_exits_when_unsourced(monkeypatch, capsys):
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    with pytest.raises(SystemExit) as e:
        require_ros_jazzy()
    assert e.value.code == 2
    assert "source /opt/ros/jazzy/setup.bash" in capsys.readouterr().out

def test_require_ros_jazzy_rejects_other_distro(monkeypatch):
    monkeypatch.setenv("ROS_DISTRO", "humble")
    with pytest.raises(SystemExit):
        require_ros_jazzy()

def test_require_ros_jazzy_ok(monkeypatch):
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    require_ros_jazzy()  # no exception

def test_try_step_reports_ok_and_failed(capsys):
    assert try_step("good", lambda: None) is True
    assert try_step("bad", lambda: 1 / 0) is False
    out = capsys.readouterr().out
    assert "good" in out and "ok" in out
    assert "bad" in out and "FAILED" in out
```

- [ ] **Step 2：確認測試失敗**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests/test_util.py -q`
預期：FAIL，`ModuleNotFoundError: No module named 'robot_dog_sim'`

- [ ] **Step 3：實作 `robot_dog_sim/__init__.py`（空白）、`robot_dog_sim/util.py`、`tests/conftest.py`**

- [ ] **Step 4：確認測試通過**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests/test_util.py -q`
預期：`4 passed`

- [ ] **Step 5：實作 `DESC/isaac/tools/rclpy_probe.py`**

用途：確認同一個 Isaac Sim 程序裡，rclpy 能收到 OmniGraph ROS 2 橋接發佈的 `/clock`（spec 第 10 節的 rclpy 共存風險）。
- 參數：`--frames`（int，預設 300）。
- 流程：
  1. `sys.path` 加入 `DESC/isaac`，先 `require_ros_jazzy()`。
  2. `SimulationApp({'headless': True})`。
  3. 啟用 `isaacsim.core.nodes`、`isaacsim.ros2.bridge`、`isaacsim.ros2.nodes`，作法同現有 `robot_dog_isaac.py` 第 64–67 行。
  4. 開新 stage，建 OmniGraph `/ROS/clock`：`OnPlaybackTick` → `ROS2PublishClock`（topic `/clock`），時間戳接 `IsaacReadSimulationTime`，context 接 `ROS2Context`。節點型別字串與現有 `robot_dog_isaac.py` 第 58–61、202 行相同。
  5. `import rclpy`，`rclpy.init()`，建立節點並訂閱 `rosgraph_msgs/msg/Clock` 的 `/clock`。
  6. `omni.timeline.get_timeline_interface().play()`，迴圈 `--frames` 次：`simulation_app.update()`、`rclpy.spin_once(node, timeout_sec=0.0)`。
  7. 印出 `[rclpy_probe] received N /clock messages, last stamp S s`。N ≥ 10 且 S > 0 時 exit 0，否則 exit 1。最後 `rclpy.shutdown()`、`simulation_app.close()`。

- [ ] **Step 6：執行探針**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/tools/rclpy_probe.py
echo "exit=$?"
```
預期：`received N /clock messages`（N ≥ 10）且 `exit=0`。
**若失敗**（import 錯誤、程序崩潰、N = 0）：把完整輸出存成 `DESC/isaac/tools/rclpy_probe.log` 一起 commit，並在回報中標示「rclpy 共存失敗，計畫 2 改用 spec 第 10 節的 OmniGraph `ROS2SubscribeTwist` 備案」。這個結果不擋 Task 2 之後的工作。

- [ ] **Step 7：另外確認沒 source 時會被擋下**

執行（新開、沒 source ROS 的 shell）：`cd ~/IsaacLab && env -u ROS_DISTRO uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/tools/rclpy_probe.py; echo "exit=$?"`
預期：印出 source 提示，`exit=2`，而且 Isaac Sim 沒有啟動（幾秒內結束）。

- [ ] **Step 8：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaac/robot_dog_sim robot_dog_description/isaac/tools robot_dog_description/isaac/tests
git commit -m "Add robot_dog_sim util, ROS check and rclpy coexistence probe"
```

---

### Task 2：讓現有 `robot_dog_isaac.py` 在這台機器跑通（ros2_control 模式，M0）

**Files:**
- Modify: `DESC/isaac/robot_dog_isaac.py`（docstring、參數、啟動前檢查；必要時修 API 錯誤）
- Modify: `DESC/README.md`（「Isaac Sim 6.2（ROS 2 控制）」一節的啟動指令）

**Interfaces:**
- Consumes：`robot_dog_sim.util.require_ros_jazzy`、`log`、`try_step`（Task 1）。
- Produces：`robot_dog_isaac.py` 新參數 `--max-frames N`（int，預設 0 = 跑到視窗關閉）。跑滿 N 次 `simulation_app.update()` 後正常結束，exit 0。Task 3 與計畫 2 的自動化驗證都會用到。

- [ ] **Step 1：安裝缺少的 ROS 套件（需要使用者執行 sudo）**

```bash
sudo apt install -y ros-jazzy-joint-state-topic-hardware-interface
```
預期：`dpkg -l ros-jazzy-joint-state-topic-hardware-interface` 顯示 `ii`。

- [ ] **Step 2：建置 `robot_dog_description`**

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_dog_description
for m in none gazebo isaac; do xacro robot_dog_description/urdf/robot_dog.urdf.xacro sim_mode:=$m > /dev/null && echo "xacro $m ok"; done
```
預期：`Summary: 1 package finished`，以及三行 `xacro ... ok`。

- [ ] **Step 3：修改 `robot_dog_isaac.py`**
  - docstring 的執行方式改成 Global Constraints 的 `uv run` 指令。
  - 在 `from isaacsim import SimulationApp` 之前：`sys.path.insert(0, HERE)`，再 `from robot_dog_sim.util import require_ros_jazzy, log, try_step` 並呼叫 `require_ros_jazzy()`。刪掉檔內的 `log()`、`step()`，改用 util 版本。
  - 加上 `--max-frames`。主迴圈計數，到達後跳出（`simulation_app.close()` 由現有 `finally` 處理）。

- [ ] **Step 4：headless 執行並修到感測器全部 ok**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/robot_dog_isaac.py --headless --max-frames 600 2>&1 | tee ~/dog_12dof_ws/build/isaac_run.log | grep '\[robot_dog\]'
```
預期四行都是 `ok`：`joint states / commands / clock`、`IMU -> /imu`、`camera -> /camera/image_raw`、`LiDAR`。LiDAR 不能走 `/scan/points` 備案（log 裡不能出現 `2D lidar config unavailable`）。程式結束時 exit 0。
出現 `FAILED` 時，到已安裝原始碼找正確的名稱再修，例如：
`grep -rl "ROS2RtxLidarHelper" ~/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.nodes`、
`ls ~/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.sensors.experimental.rtx`、
`grep -rn "Example_Rotary_2D" ~/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/exts --include=*.json -l`。
每修一個問題就重跑這個 Step。

- [ ] **Step 5：記錄話題清單（Task 3 要比對）**

終端 A：同 Step 4 的指令，但不加 `--max-frames`。
終端 B：
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | sort > ~/dog_12dof_ws/build/topics_before.txt; cat ~/dog_12dof_ws/build/topics_before.txt
ros2 topic hz /scan --window 20      # 約 10 Hz
ros2 topic echo --once /imu --field header.frame_id      # imu_link
ros2 topic echo --once /camera/camera_info --field width # 640
```
預期：清單中有 `/clock`、`/isaac_joint_states`、`/isaac_joint_commands`、`/imu`、`/scan`、`/camera/image_raw`、`/camera/camera_info`；`/scan` 約 10 Hz。

- [ ] **Step 6：ros2_control 站立確認**

終端 A 保持執行。終端 B：
```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 launch robot_dog_description isaac.launch.py
```
終端 C：
```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 run robot_dog_description stand_up.py --ros-args -p use_sim_time:=true
sleep 10; ros2 topic echo --once /joint_states
```
預期：`ros2 control list_controllers` 中 `joint_group_position_controller` 為 active。10 秒後 `/joint_states` 的每個 `*_thigh_joint` 在 0.8 ± 0.15、`*_calf_joint` 在 −1.5 ± 0.15、`*_hip_joint` 在 0 ± 0.1。

- [ ] **Step 7：更新 `DESC/README.md` 的 Isaac Sim 啟動指令**

把 `~/isaacsim/python.sh` 改成 Global Constraints 的 `uv run` 指令，並補上 `--max-frames`、需要 `joint_state_topic_hardware_interface` 的說明。

- [ ] **Step 8：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaac/robot_dog_isaac.py robot_dog_description/README.md
git commit -m "Make robot_dog_isaac.py run on Isaac Sim 6.2 pip install"
```

---

### Task 3：把 `robot_dog_isaac.py` 拆成 `robot_dog_sim` 模組（行為不變）

**Files:**
- Create: `DESC/isaac/robot_dog_sim/robot_spec.py`、`kit.py`、`world.py`、`robot.py`、`sensors.py`、`ros_graph.py`
- Modify: `DESC/isaac/robot_dog_isaac.py`（只剩參數解析、啟動、組裝）
- Modify: `DESC/README.md`（檔案結構那一段加上 `isaac/robot_dog_sim/`）

**Interfaces:**（除了 `robot_spec` 是純 Python，其餘都在 `SimulationApp` 之後 import。所有函式都明確接收 `stage`，不用模組全域變數）
- `robot_spec.py`：`STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}`、`EFFORT = 25.0`、`VELOCITY = 30.0`、`SPAWN_Z = 0.32`、`IMU_XYZ = (0.0, 0.0, 0.048)`、`LASER_XYZ = (0.08, 0.0, 0.085)`、`CAMERA_XYZ = (0.178, 0.0, 0.0575)`、`CAMERA_HFOV = 1.204`、`CAMERA_SIZE = (640, 480)`、`DEFAULT_URDF`（`DESC/urdf/robot_dog_rl.urdf` 的絕對路徑，相對於本檔計算）。
- `kit.py`：`EXTENSIONS: tuple[str, ...]`（現有第 65–66 行那 6 個）；`enable_extensions(names=EXTENSIONS) -> None`；`new_stage(app) -> Usd.Stage`（new_stage + `app.update()` + 回傳 stage）。
- `world.py`：
  - `setup_stage(stage, physics_hz: int) -> None`：Z 向上、公尺、`/World` 為預設 prim、`/World/physicsScene`（重力 9.81、`TimeStepsPerSecond`）。
  - `ground_material(stage, static_friction=1.0, dynamic_friction=1.0, restitution=0.0) -> UsdShade.Material`：路徑 `/World/ground_friction`。
  - `static_box(stage, path, center, size, color, material) -> Usd.Prim`
  - `add_lights(stage) -> None`
  - `demo_world(stage, physics_hz: int) -> None`：重現現有 `setup_world()`，包括地面和 `/World/box_obstacle`。
- `robot.py`：
  - `convert_urdf(urdf_path: str, usd_dir: str) -> str`
  - `load_robot(stage, app, usd_path: str, spawn_z: float) -> Usd.Prim`
  - `robot_parts(root: Usd.Prim) -> tuple[Usd.Prim, Usd.Prim, dict[str, Usd.Prim]]`：回傳 (articulation root, base_link, 關節名稱→prim)
  - `configure_drives(joints, kp: float, kd: float, max_effort: float, max_velocity: float, stand: dict[str, float]) -> None`
- `sensors.py`：
  - `create_imu(stage, base_path: str, xyz) -> str`
  - `create_camera(stage, base_path: str, xyz, hfov: float, size: tuple[int, int]) -> str`
  - `create_lidar(stage, base_path: str, xyz) -> tuple[str, str, str]`：回傳 (prim 路徑, `'laser_scan'` 或 `'point_cloud'`, topic)，保留現有 2D→3D 備案邏輯。
- `ros_graph.py`：節點型別常數（`ROS`、`TICK`、`SIM_TIME`、`CONTEXT`、`RENDER_PRODUCT`）；`graph(stage, path, nodes, values, connections) -> None`；`clock_graph(stage, topic='/clock')`；`joint_state_graph(stage, art_path, topic)`；`joint_command_graph(stage, art_path, topic)`；`imu_graph(stage, imu_path, topic='/imu', frame_id='imu_link')`；`camera_graph(stage, cam_path, size, topic='/camera/image_raw', info_topic='/camera/camera_info', frame_id='camera_optical_link')`；`lidar_graph(stage, lidar_path, kind, topic, frame_id='laser_link')`。
  原本的 `joint_graph` 拆成 `clock_graph` + `joint_state_graph` + `joint_command_graph` 三張圖，各自有 Tick/Context。計畫 2 的導航模式只用前兩張。
- `robot_dog_isaac.py`：指令列參數與 log 輸出和 Task 2 完成時相同，`main()` 依序呼叫上面的函式。

- [ ] **Step 1：建立上述模組，把 `robot_dog_isaac.py` 的程式碼依職責搬過去**（只搬移和加參數，不改邏輯）

- [ ] **Step 2：靜態檢查**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m py_compile robot_dog_isaac.py robot_dog_sim/*.py && python3 -c "import sys; sys.path.insert(0,'.'); import robot_dog_sim.robot_spec as s; print(s.DEFAULT_URDF)" && python3 -m pytest tests -q`
預期：沒有錯誤；印出的 URDF 路徑存在；`4 passed`

- [ ] **Step 3：重跑 Task 2 Step 4（headless 600 frames）**
預期：四行 `ok`，exit 0。

- [ ] **Step 4：重跑 Task 2 Step 5，比對話題清單**

執行：`ros2 topic list | sort > ~/dog_12dof_ws/build/topics_after.txt; diff ~/dog_12dof_ws/build/topics_before.txt ~/dog_12dof_ws/build/topics_after.txt && echo same`
預期：`same`

- [ ] **Step 5：重跑 Task 2 Step 6（ros2_control 站立）**，預期相同。

- [ ] **Step 6：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaac robot_dog_description/README.md
git commit -m "Split robot_dog_isaac.py into robot_dog_sim modules"
```

---

### Task 4：`policy_meta.py`：policy.json 的讀寫與檢查

**Files:**
- Create: `DESC/isaac/robot_dog_sim/policy_meta.py`（純 Python，只用 `json`、`os`、`math`）
- Test: `DESC/isaac/tests/test_policy_meta.py`

**Interfaces:**
- Produces：
  - `class PolicyMetaError(ValueError)`
  - `REQUIRED_KEYS: tuple[str, ...]` = `("task", "run", "checkpoint", "joint_names", "default_joint_pos", "action_scale", "clip_actions", "obs_terms", "sim_dt", "decimation", "actuator", "physx", "materials", "command_ranges", "spawn_height")`
  - `obs_dim(meta: dict) -> int`：`obs_terms` 的 `dim` 總和。
  - `validate_policy_meta(meta: dict, policy_input_dim: int | None = None, policy_output_dim: int | None = None) -> None`：任何問題都 raise `PolicyMetaError`，訊息要指出欄位名稱。規則：
    - 缺少 `REQUIRED_KEYS` 中的鍵（訊息列出全部缺少的鍵）。
    - `joint_names` 非空、沒有重複、長度等於 `default_joint_pos`。
    - `obs_terms` 每項都有 `name`（str）、`dim`（int > 0）、`scale`（null 或數字）、`clip`（null 或兩個數字）。
    - `policy_input_dim` 有給時必須等於 `obs_dim(meta)`，訊息要同時寫出兩個數字。
    - `policy_output_dim` 有給時必須等於 `len(joint_names)`。
    - `sim_dt` > 0；`decimation` 是 int 且 ≥ 1。
    - `actuator.type == "DCMotor"`；`stiffness`、`effort_limit`、`saturation_effort`、`velocity_limit` > 0；`damping` ≥ 0。
    - `physx` 有 `solver_position_iterations`、`solver_velocity_iterations`、`self_collisions`。
    - `materials.ground` 有 `static_friction`、`dynamic_friction`、`restitution`、`friction_combine_mode`、`restitution_combine_mode`；`materials.robot` 有 `static_friction`、`dynamic_friction`、`restitution`。
    - `command_ranges` 的 `lin_vel_x`、`lin_vel_y`、`ang_vel_z` 都是 `[lo, hi]` 且 lo ≤ hi。
  - `save_policy_meta(meta: dict, path: str) -> None`：先檢查，再用 `json.dumps(meta, indent=2)` 轉成字串，**成功後才寫檔**。
  - `load_policy_meta(path: str, policy_input_dim: int | None = None, policy_output_dim: int | None = None) -> dict`：讀檔並檢查。

- [ ] **Step 1：寫失敗的測試** `DESC/isaac/tests/test_policy_meta.py`

fixture `valid_meta()` 依 spec 第 4.4 節的範例建立：12 個關節名稱（`FL_hip_joint` …）、`default_joint_pos` 長度 12、7 個 obs 項（`base_lin_vel` 3、`base_ang_vel` 3、`projected_gravity` 3、`velocity_commands` 3、`joint_pos` 12、`joint_vel` 12、`actions` 12，scale／clip 都是 None），其他欄位照 spec 的數值。

```python
def test_valid_meta_passes(valid_meta):
    validate_policy_meta(valid_meta, policy_input_dim=48, policy_output_dim=12)
    assert obs_dim(valid_meta) == 48

def test_missing_keys_are_all_named(valid_meta):
    del valid_meta["materials"]; del valid_meta["sim_dt"]
    with pytest.raises(PolicyMetaError, match="materials") as e:
        validate_policy_meta(valid_meta)
    assert "sim_dt" in str(e.value)

def test_obs_dim_mismatch_reports_both(valid_meta):
    with pytest.raises(PolicyMetaError) as e:
        validate_policy_meta(valid_meta, policy_input_dim=45)
    assert "45" in str(e.value) and "48" in str(e.value)

def test_output_dim_mismatch(valid_meta):
    with pytest.raises(PolicyMetaError, match="joint"):
        validate_policy_meta(valid_meta, policy_output_dim=8)

def test_joint_lengths_must_match(valid_meta):
    valid_meta["default_joint_pos"] = valid_meta["default_joint_pos"][:11]
    with pytest.raises(PolicyMetaError, match="default_joint_pos"):
        validate_policy_meta(valid_meta)

def test_duplicate_joint_names(valid_meta):
    valid_meta["joint_names"][1] = valid_meta["joint_names"][0]
    with pytest.raises(PolicyMetaError, match="duplicate"):
        validate_policy_meta(valid_meta)

def test_bad_actuator_type(valid_meta):
    valid_meta["actuator"]["type"] = "IdealPD"
    with pytest.raises(PolicyMetaError, match="DCMotor"):
        validate_policy_meta(valid_meta)

def test_bad_command_range(valid_meta):
    valid_meta["command_ranges"]["ang_vel_z"] = [1.0, -1.0]
    with pytest.raises(PolicyMetaError, match="ang_vel_z"):
        validate_policy_meta(valid_meta)

def test_roundtrip(valid_meta, tmp_path):
    p = tmp_path / "policy.json"
    save_policy_meta(valid_meta, str(p))
    assert load_policy_meta(str(p), policy_input_dim=48, policy_output_dim=12) == valid_meta

def test_save_refuses_invalid_and_writes_nothing(valid_meta, tmp_path):
    del valid_meta["task"]
    p = tmp_path / "policy.json"
    with pytest.raises(PolicyMetaError):
        save_policy_meta(valid_meta, str(p))
    assert not p.exists()

def test_save_leaves_no_file_on_unserializable_value(valid_meta, tmp_path):
    valid_meta["spawn_height"] = object()
    p = tmp_path / "policy.json"
    with pytest.raises((TypeError, PolicyMetaError)):
        save_policy_meta(valid_meta, str(p))
    assert not p.exists()
```

- [ ] **Step 2：確認失敗**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests/test_policy_meta.py -q`
預期：FAIL，`ModuleNotFoundError: ... policy_meta`

- [ ] **Step 3：實作 `robot_dog_sim/policy_meta.py`**

- [ ] **Step 4：確認通過**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests -q`
預期：`15 passed`

- [ ] **Step 5：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaac/robot_dog_sim/policy_meta.py robot_dog_description/isaac/tests/test_policy_meta.py
git commit -m "Add policy.json schema validation shared by export and Isaac Sim"
```

---

### Task 5：任務設定修改、訓練／回放腳本、smoke test

**Files:**
- Modify: `DESC/isaaclab/robot_dog_lab/tasks/locomotion/velocity/flat_env_cfg.py`
- Create: `DESC/isaaclab/scripts/train.py`、`DESC/isaaclab/scripts/play.py`
- Create: `DESC/isaaclab/tests/conftest.py`（把 `DESC/isaaclab` 加到 `sys.path[0]`）
- Test: `DESC/isaaclab/tests/test_flat_env_cfg.py`
- Delete: `DESC/isaaclab/setup_robot_dog_lab.sh`
- Modify: `DESC/isaaclab/README.md`（改寫「步驟」：不再建外部專案，改用 scripts；訓練與回放指令）、`DESC/README.md`（「Isaac Lab 3.0」一節同步）

**Interfaces:**
- `flat_env_cfg.py` 的 `RobotDogFlatEnvCfg.__post_init__` 增加（spec 第 4.2 節）：
  - `self.commands.base_velocity.heading_command = False`
  - `self.commands.base_velocity.rel_standing_envs = 0.1`
  - `self.rewards.track_lin_vel_xy_exp.params["std"] = 0.25`
  - `self.rewards.track_ang_vel_z_exp.params["std"] = 0.25`
- `scripts/train.py`、`scripts/play.py`：比照 `~/robotdog_ws/dog_rl/scripts/train.py`，`sys.path.insert(0, <DESC/isaaclab>)`、`wp.config.enable_backward = False`、`import robot_dog_lab.tasks`，再 `raise SystemExit(run_train_cli(sys.argv[1:]))`（play 用 `run_play_cli`，兩者都從 `isaaclab_rl.entrypoints` import）。
- 訓練紀錄位置：`~/IsaacLab/logs/rsl_rl/robot_dog_flat/<時間>_<run_name>/`。

- [ ] **Step 1：寫失敗的測試** `DESC/isaaclab/tests/test_flat_env_cfg.py`

```python
import pytest
from robot_dog_lab.tasks.locomotion.velocity.flat_env_cfg import RobotDogFlatEnvCfg

def test_commands_match_spec():
    c = RobotDogFlatEnvCfg().commands.base_velocity
    assert c.heading_command is False
    assert c.rel_standing_envs == pytest.approx(0.1)
    assert tuple(c.ranges.lin_vel_x) == (-0.8, 0.8)
    assert tuple(c.ranges.lin_vel_y) == (-0.5, 0.5)
    assert tuple(c.ranges.ang_vel_z) == (-1.0, 1.0)

def test_tracking_reward_std():
    r = RobotDogFlatEnvCfg().rewards
    assert r.track_lin_vel_xy_exp.params["std"] == pytest.approx(0.25)
    assert r.track_ang_vel_z_exp.params["std"] == pytest.approx(0.25)

def test_timing_and_action_scale():
    cfg = RobotDogFlatEnvCfg()
    assert cfg.sim.dt == pytest.approx(0.005)
    assert cfg.decimation == 4
    assert cfg.actions.joint_pos.scale == pytest.approx(0.25)
```

- [ ] **Step 2：確認失敗**

執行：`cd ~/IsaacLab && uv run --extra isaacsim --with pytest python -m pytest ~/dog_12dof_ws/robot_dog_description/isaaclab/tests/test_flat_env_cfg.py -q`
預期：`test_commands_match_spec` 與 `test_tracking_reward_std` FAIL（heading_command 是 True、std 是 0.5），`test_timing_and_action_scale` PASS。
若連 import 都失敗（設定類別需要 `SimulationApp`），在 `conftest.py` 用 `isaaclab.app.AppLauncher(headless=True)` 建立 session 層級的 app，再重跑。

- [ ] **Step 3：修改 `flat_env_cfg.py`**（上面四行）

- [ ] **Step 4：確認通過**：同 Step 2 的指令，預期 `3 passed`

- [ ] **Step 5：建立 `scripts/train.py`、`scripts/play.py`，刪除 `setup_robot_dog_lab.sh`**

- [ ] **Step 6：smoke test**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/train.py \
    --task RobotDog-Velocity-Flat --num_envs 64 --max_iterations 5 --run_name smoke
echo "exit=$?"; ls ~/IsaacLab/logs/rsl_rl/robot_dog_flat/*_smoke/
```
預期：`exit=0`；資料夾裡有 `params/env.yaml` 和至少一個 `model_*.pt`。第一次執行會把 URDF 轉成 USD，會比較久。
若失敗：常見原因是 `flat_env_cfg.py` 裡的連桿名稱（`base_link`、`.*_calf`、`.*_thigh`）和轉出來的 USD 對不上。用錯誤訊息裡列出的 body 名稱修正。

- [ ] **Step 7：更新兩份 README**（刪掉 `setup_robot_dog_lab.sh`、`~/projects` 的步驟，改成 Step 6 的指令與回放指令）

```bash
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/play.py \
    --task RobotDog-Velocity-Flat --checkpoint <run 資料夾>/model_<N>.pt --viz kit
```

- [ ] **Step 8：Commit**

```bash
cd ~/dog_12dof_ws && git add -A robot_dog_description/isaaclab robot_dog_description/README.md
git commit -m "Train robot_dog from the IsaacLab env: heading off, std 0.25, 10% standing"
```

---

### Task 6：評估判準、評估用環境設定、站立測試

**Files:**
- Create: `DESC/isaaclab/robot_dog_lab/gait_criteria.py`（純 Python）
- Create: `DESC/isaaclab/robot_dog_lab/eval_env.py`
- Create: `DESC/isaaclab/scripts/stand_test.py`
- Test: `DESC/isaaclab/tests/test_gait_criteria.py`（系統 python3）
- Test: `DESC/isaaclab/tests/test_flat_env_cfg.py`（加一個測試）

**Interfaces:**
- Produces（`robot_dog_lab/gait_criteria.py`）：
  - `@dataclass(frozen=True) class GaitCase: name: str; command: tuple[float, float, float]; axis: int | None`（axis：0 = vx、1 = vy、2 = wz、None = 站立）
  - `CASES: tuple[GaitCase, ...]`：17 個，依 spec 第 4.3 節。vx ∈ {±0.1, ±0.3, ±0.5}、vy ∈ {±0.1, ±0.3}、wz ∈ {±0.2, ±0.5, ±0.8}、站立 (0, 0, 0)。名稱格式例如 `"vx+0.10"`、`"wz-0.80"`、`"stand"`。
  - 常數：`LIN_TOL = 0.1`、`ANG_TOL = 0.15`、`DRIFT_TOL = 0.1`、`WARMUP_S = 2.0`、`MEASURE_S = 5.0`、`STAND_MEASURE_S = 10.0`、`STAND_HEIGHT_RANGE = (0.20, 0.28)`
  - `judge_case(case: GaitCase, mean_vel: tuple[float, float, float], drift: float, falls: int) -> tuple[bool, str]`：falls > 0 一律失敗，原因含 `"fall"`；axis 為 0、1 時看該軸誤差是否 < `LIN_TOL`，為 2 時看是否 < `ANG_TOL`，失敗原因含軸名（`vx`/`vy`/`wz`）；axis 為 None 時看 drift 是否 < `DRIFT_TOL`。
  - `judge_stand(heights: Sequence[float], falls: int) -> tuple[bool, str]`
- Produces（`robot_dog_lab/eval_env.py`）：`make_eval_env_cfg(num_envs: int, episode_length_s: float = 1000.0) -> RobotDogFlatEnvCfg`。依序：`cfg = RobotDogFlatEnvCfg()`、`cfg.play_mode()`、`cfg.sim.physics = cfg.sim.physics.isaacsim_physx`、`cfg.scene.num_envs = num_envs`、`cfg.episode_length_s = episode_length_s`。指令改成 `resampling_time_range = (1000.0, 1000.0)`、`rel_standing_envs = 0.0`、`debug_vis = False`；事件改成 `reset_base.params["pose_range"] = {}`、`reset_base.params["velocity_range"] = {}`、`reset_robot_joints.params["position_range"] = (1.0, 1.0)`。`play_mode()` 已經會關掉推力與觀測雜訊。Task 7、Task 8 和計畫 2 的一致性測試都用這個函式。
- `scripts/stand_test.py`：參數 `--num_envs`（預設 16）、`--seconds`（預設 5.0），加上 `isaaclab.app.add_launcher_args`。用 `make_eval_env_cfg`，在 `launch_simulation(env_cfg, args)` 內 `gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)`，送全零動作 `seconds / step_dt` 步，統計 `terminated` 總數。結束時讀 `robot.data.root_pos_w.torch[:, 2]`。印出 `[STAND] height mean=… min=… max=… falls=…` 和 `judge_stand` 的結果，通過 exit 0，否則 exit 1。

- [ ] **Step 1：寫失敗的測試** `DESC/isaaclab/tests/test_gait_criteria.py`

```python
from robot_dog_lab.gait_criteria import CASES, judge_case, judge_stand

def _case(name):
    return next(c for c in CASES if c.name == name)

def test_cases_match_spec():
    cmds = {c.command for c in CASES}
    expected = {(s * v, 0.0, 0.0) for v in (0.1, 0.3, 0.5) for s in (1, -1)} \
             | {(0.0, s * v, 0.0) for v in (0.1, 0.3) for s in (1, -1)} \
             | {(0.0, 0.0, s * v) for v in (0.2, 0.5, 0.8) for s in (1, -1)} | {(0.0, 0.0, 0.0)}
    assert len(CASES) == 17 and cmds == expected
    assert sum(c.axis is None for c in CASES) == 1

def test_linear_tolerance():
    assert judge_case(_case("vx+0.30"), (0.38, 0.0, 0.0), 0.0, 0)[0]
    ok, why = judge_case(_case("vx+0.30"), (0.41, 0.0, 0.0), 0.0, 0)
    assert not ok and "vx" in why
    assert not judge_case(_case("vy-0.10"), (0.0, 0.01, 0.0), 0.0, 0)[0]

def test_angular_tolerance():
    assert judge_case(_case("wz+0.50"), (0.0, 0.0, 0.64), 0.0, 0)[0]
    assert not judge_case(_case("wz+0.50"), (0.0, 0.0, 0.66), 0.0, 0)[0]

def test_stand_uses_drift():
    assert judge_case(_case("stand"), (0.05, 0.0, 0.0), 0.09, 0)[0]
    assert not judge_case(_case("stand"), (0.0, 0.0, 0.0), 0.11, 0)[0]

def test_any_fall_fails():
    ok, why = judge_case(_case("vx+0.10"), (0.1, 0.0, 0.0), 0.0, 1)
    assert not ok and "fall" in why

def test_judge_stand():
    assert judge_stand([0.22] * 16, 0)[0]
    assert not judge_stand([0.22] * 15 + [0.15], 0)[0]
    assert not judge_stand([0.22] * 16, 1)[0]
```

在 `test_flat_env_cfg.py` 加上：

```python
def test_eval_env_cfg_is_deterministic():
    from robot_dog_lab.eval_env import make_eval_env_cfg
    cfg = make_eval_env_cfg(17)
    assert cfg.scene.num_envs == 17
    assert cfg.events.push_robot is None
    assert cfg.events.reset_base.params["pose_range"] == {}
    assert cfg.events.reset_robot_joints.params["position_range"] == (1.0, 1.0)
    assert cfg.commands.base_velocity.rel_standing_envs == 0.0
    assert cfg.observations.policy.enable_corruption is False
```

- [ ] **Step 2：確認失敗**

執行：`cd ~/dog_12dof_ws/robot_dog_description/isaaclab && python3 -m pytest tests/test_gait_criteria.py -q`，預期 `ModuleNotFoundError`。
再執行：`cd ~/IsaacLab && uv run --extra isaacsim --with pytest python -m pytest ~/dog_12dof_ws/robot_dog_description/isaaclab/tests/test_flat_env_cfg.py -q`，預期 `test_eval_env_cfg_is_deterministic` FAIL。

- [ ] **Step 3：實作 `gait_criteria.py`、`eval_env.py`**

- [ ] **Step 4：確認通過**：上面兩個指令分別為 `6 passed`、`4 passed`

- [ ] **Step 5：實作 `scripts/stand_test.py` 並執行**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/stand_test.py
echo "exit=$?"
```
預期：`[STAND] height mean=0.2x ...`，16 隻都在 0.20–0.28 m，`falls=0`，`exit=0`。
若高度太低（< 0.20）或倒下：在回報中附上數字，**不要**改 kp 或質量（那是設計變更），交給使用者決定。

- [ ] **Step 6：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaaclab
git commit -m "Add gait criteria, deterministic eval env and stand test"
```

---

### Task 7：匯出策略（`policy.pt` + `policy.json`）

**Files:**
- Create: `DESC/isaaclab/robot_dog_lab/checkpoints.py`（純 Python）
- Create: `DESC/isaaclab/scripts/export_policy.py`
- Test: `DESC/isaaclab/tests/test_checkpoints.py`（系統 python3）
- Modify: `DESC/isaaclab/README.md`（加「匯出」一節）

**Interfaces:**
- Consumes：`robot_dog_sim.policy_meta.validate_policy_meta`、`save_policy_meta`（Task 4，`sys.path` 要加 `DESC/isaac`）；`robot_dog_lab.eval_env.make_eval_env_cfg`（Task 6）。
- Produces（`checkpoints.py`）：
  - `latest_checkpoint(run_dir: str) -> str`：回傳檔名中數字最大的 `model_<N>.pt`（只回傳檔名），沒有時 raise `FileNotFoundError`。
  - `prepare_out_dir(path: str, force: bool) -> None`：資料夾不存在就建立；已存在且非空、又沒有 `force` 時，raise `FileExistsError`，訊息含 `--force`。
- Produces（`export_policy.py`）：參數 `--run_dir`（必填，支援 `~`）、`--checkpoint`（預設 `latest_checkpoint(run_dir)`）、`--out`（預設 `DESC/isaac/policies/<run_dir 的資料夾名稱>`）、`--force`，以及 `add_launcher_args`。輸出 `<out>/policy.pt`（TorchScript）和 `<out>/policy.json`（spec 第 4.4 節格式）。計畫 2 的 Isaac Sim 端讀這兩個檔。

- [ ] **Step 1：寫失敗的測試** `DESC/isaaclab/tests/test_checkpoints.py`

```python
import pytest
from robot_dog_lab.checkpoints import latest_checkpoint, prepare_out_dir

def test_latest_checkpoint_is_numeric(tmp_path):
    for n in (50, 999, 1000):
        (tmp_path / f"model_{n}.pt").write_bytes(b"")
    assert latest_checkpoint(str(tmp_path)) == "model_1000.pt"

def test_latest_checkpoint_ignores_other_files(tmp_path):
    (tmp_path / "model_7.pt").write_bytes(b"")
    (tmp_path / "model_best.pt").write_bytes(b"")
    (tmp_path / "events.out").write_bytes(b"")
    assert latest_checkpoint(str(tmp_path)) == "model_7.pt"

def test_latest_checkpoint_empty_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        latest_checkpoint(str(tmp_path))

def test_prepare_out_dir_creates(tmp_path):
    prepare_out_dir(str(tmp_path / "a" / "b"), force=False)
    assert (tmp_path / "a" / "b").is_dir()

def test_prepare_out_dir_refuses_existing(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    with pytest.raises(FileExistsError, match="--force"):
        prepare_out_dir(str(tmp_path), force=False)

def test_prepare_out_dir_force(tmp_path):
    (tmp_path / "policy.json").write_text("{}")
    prepare_out_dir(str(tmp_path), force=True)
```

- [ ] **Step 2：確認失敗**：`cd ~/dog_12dof_ws/robot_dog_description/isaaclab && python3 -m pytest tests/test_checkpoints.py -q`，預期 `ModuleNotFoundError`

- [ ] **Step 3：實作 `checkpoints.py`**

- [ ] **Step 4：確認通過**：同 Step 2 的指令，預期 `6 passed`

- [ ] **Step 5：實作 `scripts/export_policy.py`**

在 `launch_simulation(env_cfg, args)` 內，依序：
1. `env_cfg = make_eval_env_cfg(1)`、`agent_cfg = RobotDogFlatPPORunnerCfg()`；`env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)`；`uenv = env.unwrapped`、`robot = uenv.scene["robot"]`。
2. `wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)`；
   `runner = OnPolicyRunner(wrapped, to_dict(agent_cfg), log_dir=None, device=agent_cfg.device)`。
   import 來源：`rsl_rl.runners.OnPolicyRunner`、`isaaclab.utils.to_dict`、`isaaclab_rl.rsl_rl.RslRlVecEnvWrapper`，與 `isaaclab_rl/entrypoints/backends/play_rsl_rl.py` 相同。
3. `runner.load(<run_dir>/<checkpoint>)`，然後 `runner.export_policy_to_jit(path=out, filename="policy.pt")`。
4. 組 `policy.json`，每個值都轉成 Python 內建型別（`float(...)`、`.tolist()`）：
   - `joint_names` = `list(uenv.action_manager.get_term("joint_pos")._joint_names)`。
   - `default_joint_pos` = `robot.data.default_joint_pos.torch[0]` 依 `joint_names` 在 `robot.joint_names` 中的索引取值。
   - `obs_terms`：依序 zip `uenv.observation_manager.active_terms["policy"]`、`group_obs_term_dim["policy"]`、`_group_obs_term_cfgs["policy"]`，`dim` = shape 各維相乘，`scale`/`clip` 取自 term cfg（None 就存 null）。
   - `action_scale` = `env_cfg.actions.joint_pos.scale`；`clip_actions` = `agent_cfg.clip_actions`；`sim_dt` = `env_cfg.sim.dt`；`decimation` = `env_cfg.decimation`。
   - `actuator` 取自 `ROBOT_DOG_CFG.actuators["legs"]`：`stiffness`、`damping`、`actuator_effort_limit`→`effort_limit`、`saturation_effort`、`actuator_velocity_limit`→`velocity_limit`。
   - `physx` 取自 `ROBOT_DOG_CFG.spawn.articulation_props[0]`：`solver_position_iteration_count`、`solver_velocity_iteration_count`、`enabled_self_collisions`。
   - `materials.ground` 取自 `env_cfg.scene.terrain.physics_material`；`materials.robot` 取自 `env_cfg.events.physics_material.params` 三個 `*_range` 的平均值。
   - `command_ranges` 取自 `env_cfg.commands.base_velocity.ranges`；`spawn_height` = `ROBOT_DOG_CFG.init_state.pos[2]`；`task`、`run`（run_dir 的資料夾名稱）、`checkpoint`（檔名）。
5. 檢查（任一項不符就 `raise SystemExit(1)`，並印出原因）：
   - `sorted(joint_names) == sorted(robot.joint_names)` 且長度為 12（Review Focus 1），否則印出兩份清單。
   - `obs = wrapped.get_observations()`；`jit = torch.jit.load(<out>/policy.pt)`；`a_jit = jit(obs["policy"].cpu())`；`a_ref = runner.get_inference_policy(device=uenv.device)(obs).cpu()`；`(a_jit - a_ref).abs().max() < 1e-5`。
   - `validate_policy_meta(meta, policy_input_dim=obs["policy"].shape[1], policy_output_dim=a_jit.shape[1])`。
6. `save_policy_meta(meta, <out>/policy.json)`，印出 `[EXPORT] wrote <out>`、`[EXPORT] joint order: [...]`、`[EXPORT] obs dim 48, max |jit - runner| = ...`。

輸出資料夾在第 1 步之前就先用 `prepare_out_dir(out, args.force)` 準備好。

- [ ] **Step 6：用 smoke run 驗證匯出**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
RUN=$(ls -d ~/IsaacLab/logs/rsl_rl/robot_dog_flat/*_smoke | tail -1)
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/export_policy.py --run_dir $RUN --out $RUN/policy_export
echo "exit=$?"
python3 -c "import json,sys; m=json.load(open('$RUN/policy_export/policy.json')); print(len(m['joint_names']), sum(t['dim'] for t in m['obs_terms']), m['joint_names'])"
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/export_policy.py --run_dir $RUN --out $RUN/policy_export; echo "second exit=$?"
```
預期：第一次 `exit=0`，`max |jit - runner|` < 1e-5；`python3` 印出 `12 48 [...]`，關節集合和 URDF 的 12 個關節相同。第二次因資料夾已存在而失敗，訊息含 `--force`，`second exit` ≠ 0。
**把印出的 joint order 記在回報裡**：它決定 spec 對 README 關節順序的懷疑是否成立。

- [ ] **Step 7：README 加「匯出」一節**（指令、輸出位置、`--force`）

- [ ] **Step 8：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaaclab
git commit -m "Export policy.pt + policy.json with joint-order and jit parity checks"
```

---

### Task 8：`gait_eval.py`：依 spec 第 4.3 節評估

**Files:**
- Create: `DESC/isaaclab/scripts/gait_eval.py`
- Modify: `DESC/isaaclab/README.md`（加「評估」一節）

**Interfaces:**
- Consumes：`gait_criteria.CASES`、`judge_case`、`WARMUP_S`、`MEASURE_S`、`STAND_MEASURE_S`（Task 6）；`make_eval_env_cfg`（Task 6）；`robot_dog_sim.policy_meta.load_policy_meta`（Task 4）；Task 7 輸出的資料夾。
- Produces：`gait_eval.py --policy_dir <dir>`，加上 `add_launcher_args`。印出每個 case 一行，最後一行 `[GAIT] PASS k/17`。17 個都通過時 exit 0，否則 exit 1。

- [ ] **Step 1：實作 `scripts/gait_eval.py`**
  - `meta = load_policy_meta(<dir>/policy.json)`；`policy = torch.jit.load(<dir>/policy.pt)`，放到 env device。
  - `env_cfg = make_eval_env_cfg(len(CASES))`；建 env 之後，確認 `meta["joint_names"]` 和動作項的 `_joint_names` 相同，否則 exit 1，提示「policy 與目前任務的關節順序不符」。
  - `cmd_term = uenv.command_manager.get_term("base_velocity")`；`cmds` = 各 case 的 command 疊成 `(17, 3)` tensor。
  - `obs, _ = env.reset()`；總步數 = `round((WARMUP_S + STAND_MEASURE_S) / step_dt)`。每一步：先 `cmd_term.vel_command_b[:] = cmds`，再 `env.step(policy(obs["policy"]))`。
  - 統計：每個 env 的 `terminated` 次數；在 `[WARMUP_S, WARMUP_S + MEASURE_S)` 期間累加 `robot.data.root_lin_vel_b.torch[:, :2]` 與 `root_ang_vel_b.torch[:, 2]` 取平均；`drift` = `root_pos_w.torch[:, :2]` 在 `WARMUP_S` 與結束時的距離。
  - 每個 case 印 `[GAIT] <name> cmd=(…) got=(…) drift=… falls=… PASS/FAIL <原因>`。

- [ ] **Step 2：用 smoke 策略跑一次，確認流程能跑完**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
RUN=$(ls -d ~/IsaacLab/logs/rsl_rl/robot_dog_flat/*_smoke | tail -1)
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/gait_eval.py --policy_dir $RUN/policy_export
echo "exit=$?"
```
預期：印出 17 行 case 和 `[GAIT] PASS k/17`；未訓練的策略大多會 FAIL，所以 `exit=1` 是正常的。重點是沒有例外、17 行都有數字。

- [ ] **Step 3：README 加「評估」一節**（指令與通過標準表，數值抄自 spec 第 4.3 節）

- [ ] **Step 4：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaaclab
git commit -m "Add gait_eval with spec 4.3 pass criteria"
```

---

### Task 9：正式訓練、通過評估、提交策略（完成 M1）

**Files:**
- Create: `DESC/isaac/policies/<run>/policy.pt`、`policy.json`（匯出產生）
- Modify（只在需要調參時）：`flat_env_cfg.py`、`tests/test_flat_env_cfg.py`、spec 第 4.2 節
- Modify: `DESC/isaaclab/README.md`（記錄目前採用的策略版本與評估結果）

**Interfaces:**
- Produces：`DESC/isaac/policies/<run>/` 通過 `gait_eval`，這是計畫 2 的輸入。

- [ ] **Step 1：正式訓練**（約 35–50 分鐘，背景執行）

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/train.py \
    --task RobotDog-Velocity-Flat --run_name v1 > ~/IsaacLab/logs/robot_dog_v1_train.log 2>&1
echo "exit=$?"; tail -40 ~/IsaacLab/logs/robot_dog_v1_train.log
```
預期：`exit=0`；跑完 1000 iterations；log 尾端的 `Mean reward` 明顯高於前 50 個 iteration；`Mean episode length` 接近 1000 步（20 s ÷ 0.02 s）。

- [ ] **Step 2：匯出到 repo**

```bash
RUN=$(ls -d ~/IsaacLab/logs/rsl_rl/robot_dog_flat/*_v1 | tail -1)
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/export_policy.py --run_dir $RUN
```
預期：`[EXPORT] wrote .../robot_dog_description/isaac/policies/<run 資料夾名稱>`

- [ ] **Step 3：評估**

```bash
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/gait_eval.py \
    --policy_dir ~/dog_12dof_ws/robot_dog_description/isaac/policies/$(basename $RUN)
```
預期：`[GAIT] PASS 17/17`，`exit=0`。

- [ ] **Step 4：不通過時依序調整**。每次只改一項，用新的 `--run_name`（v2、v3…）重訓、重新匯出、重新評估；同時更新 `test_flat_env_cfg.py` 的期望值和 spec 第 4.2 節。前一版沒過的策略資料夾從 `DESC/isaac/policies/` 刪掉。
  1. 只有低速 case 差一點：從上一版的最後 checkpoint 接著訓練 1000 iterations：`--checkpoint $RUN/<latest> --max_iterations 1000 --run_name v2`。
  2. 追蹤誤差普遍偏大：`track_lin_vel_xy_exp.weight` 1.5 → 2.0、`track_ang_vel_z_exp.weight` 0.75 → 1.0。
  3. 仍有低速死區：兩個追蹤 `std` 從 0.25 → 0.2。
  4. 只有站立漂移不過：`rel_standing_envs` 0.1 → 0.2。

  **三輪調整後仍未通過**：停下來，把三輪的 `[GAIT]` 結果整理給使用者決定，不要繼續猜。

- [ ] **Step 5：目視確認**（使用者在場時，選做）

```bash
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/play.py \
    --task RobotDog-Velocity-Flat --checkpoint $RUN/$(cd $RUN && ls model_*.pt | sort -t_ -k2 -n | tail -1) --viz kit
```

- [ ] **Step 6：README 記錄採用的版本**（run 名稱、iterations、`[GAIT]` 17 行結果、關節順序）

- [ ] **Step 7：Commit**

```bash
cd ~/dog_12dof_ws && git add robot_dog_description/isaac/policies robot_dog_description/isaaclab docs
git commit -m "Add trained gait policy <run> (gait_eval 17/17)"
```

---

## 計畫 1 完成條件（= spec M0 + M1）

- `python3 -m pytest robot_dog_description/isaac/tests robot_dog_description/isaaclab/tests/test_gait_criteria.py robot_dog_description/isaaclab/tests/test_checkpoints.py -q` 全部通過；`uv run ... pytest .../test_flat_env_cfg.py` 全部通過。
- `robot_dog_isaac.py --headless --max-frames 600`：四項感測器 ok；ros2_control 能讓狗站起來。
- rclpy 探針結果已記錄（通過，或已記下要改用備案）。
- `DESC/isaac/policies/<run>/` 有 `policy.pt` + `policy.json`，`gait_eval` 17/17。
