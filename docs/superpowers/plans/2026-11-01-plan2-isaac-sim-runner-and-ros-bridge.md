# 計畫 2：Isaac Sim 執行策略 + 導航模式 ROS 2 橋接（M2 + M3）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 讓訓練好的 v7 策略在 Isaac Sim 6.2 裡同步步進地走路，速度和 Isaac Lab 一致（M2）；再把它包成導航模式：收 `/cmd_vel`，發佈感測器、`/odom_gt`、`/joint_states`、`/clock`，並在 ROS 端產生 `odom→base_footprint→base_link` TF 和去掉地面點的 `/scan_filtered`（M3）。

**Architecture:** Isaac Sim 腳本用和 Isaac Lab 相同的同步步進：關掉 Kit 自己的物理步進，每個循環手動跑 `decimation` 個 0.005 s 物理步（每步前由 `actuator.py` 算 DC 馬達力矩並施加），每 `decimation` 步跑一次策略，再呼叫一次 `app.update()` 讓渲染和 OmniGraph 感測器更新。觀測、致動器、指令緩衝、站立保持都是可單元測試的純函式或類別，設定全部來自 `policy.json`。ROS 端新增 ament_python 套件 `robot_dog_navigation`，先放計畫 2 需要的兩個節點，Nav2 留給計畫 3。

**Tech Stack:** Isaac Sim 6.2（pip 版，`~/IsaacLab/.venv`）、`isaacsim.core.experimental.prims.Articulation`、`isaacsim.core.simulation_manager.SimulationManager`、OmniGraph ROS 2 橋接、rclpy（同一程序，計畫 1 已驗證可用）、Isaac Lab 3.0（一致性參考）、ROS 2 Jazzy、pytest

**Spec:** `docs/superpowers/specs/2026-11-01-robot-dog-isaac-nav2-design.md`（第 3、5、6.1–6.2、7 M2/M3、8、9 節，以及「實作分期」裡計畫 1 留下的事項）

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`DESC` = `~/dog_12dof_ws/robot_dog_description`，`NAV` = `~/dog_12dof_ws/robot_dog_navigation`。
- Isaac 指令一律在 `~/IsaacLab` 下用 `uv run --extra isaacsim python <script>`，前置 `export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1`；需要 ROS 的先 `source /opt/ros/jazzy/setup.bash`。同一時間只跑一個 Isaac 程序。
- 自己建 `SimulationApp` 的腳本用 `--headless`；Isaac Lab launcher（`add_launcher_args`）沒有 `--headless`，看畫面才加 `--viz kit`。
- 在 `launch_simulation` 區塊內用 `raise SystemExit(n)` 才保得住 exit code；自己的 `SimulationApp` 用 `simulation_app.close(exit_code=n)`，例外要先 `traceback.print_exc()`（計畫 1 的做法）。
- API 以已安裝原始碼為準：`~/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/exts/`、`~/IsaacLab/source/`。
- **四元數**：Isaac Sim 的 `Articulation.get_world_poses()` 回傳 **wxyz**；Isaac Lab 3.0 的 `isaaclab.utils.math` 用 **xyzw**。本計畫的純函式一律收 wxyz，測試要換算後才和 Isaac Lab 比。
- **機身速度**：Isaac Lab 的 `root_lin_vel_b` / `root_ang_vel_b` 是**質心**速度，轉到機身（root link）座標系。
- 一律依名稱對應關節。策略的關節順序：`FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …, RR_calf`（`policy.json` 的 `joint_names`）。
- 使用的策略：`DESC/isaac/policies/2026-11-01_20-43-36_v7/`（若 Task 1 重新匯出，路徑不變）。
- `robot_dog_sim` 的純模組（`util`、`robot_spec`、`policy_meta`、`parity`、`command_buffer`）不得 import `omni`/`pxr`/`isaacsim`/`torch`，用系統 `python3 -m pytest` 測；`actuator.py`、`observations.py` 用 torch，在 uv 環境測：`cd ~/IsaacLab && uv run --extra isaacsim --with pytest python -m pytest <檔案> -q -p no:cacheprovider`。
- 程式註解英文、文件繁體中文；commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。不動 `~/robotdog_ws`。

## Review Focus

1. **質心 vs. 連桿原點速度**：若 Isaac Sim 的 `get_velocities()` 是 root link 原點速度，機身轉動時線速度會和訓練時的質心速度差 ω × r。Task 3 的觀測函式要明確收「質心世界速度」，Task 5 要從 Isaac Sim 正確取得它（`policy_parity` 會抓到錯誤，但要先在程式裡寫對）。
2. **步進不同步**：`app.update()` 若自己又多跑物理步，或 `/clock` 和 `SimulationManager.get_simulation_time()` 不一致，策略和 ROS 時間都會錯。Task 2 的探針要量到「每循環剛好 4 步、時間剛好 +0.02 s、`/clock` 一致」才能往下。
3. **`/cmd_vel` 停送**：Nav2 當掉或 teleop 關掉後，狗要在 0.5 s（sim time）內站住，不能沿用最後的指令（Task 4 `test_timeout_zeroes_command`、Task 8 實測）。
4. **超出訓練範圍的指令**：Nav2/teleop 送 vx 1.5 m/s，要夾在 `command_ranges` 內，不能直接餵給策略（Task 4 `test_commands_are_clipped_to_training_ranges`）。
5. **雷射打地點與真障礙物**：濾掉的只能是打到地面的點；前方 1 m 高的牆即使機身前傾也必須保留（Task 7 `test_wall_is_kept_when_pitched`）。

---

### Task 1：`policy.json` 補上剛體與 PhysX scene 設定，重新匯出 v7

**Files:**
- Modify: `DESC/isaac/robot_dog_sim/policy_meta.py`、`DESC/isaac/tests/test_policy_meta.py`
- Modify: `DESC/isaaclab/scripts/export_policy.py`
- Modify: `DESC/isaac/policies/2026-11-01_20-43-36_v7/policy.json`（重新匯出）
- Modify: spec 第 4.4 節（JSON 範例）

**Interfaces:**
- Produces（`policy.json` 新增兩個必填物件，`policy_meta.REQUIRED_KEYS` 也加上）：
  - `"rigid_body": {"max_depenetration_velocity": 1.0, "linear_damping": 0.0, "angular_damping": 0.0, "max_linear_velocity": 1000.0, "max_angular_velocity": 1000.0}`
  - `"physx_scene": {"solver_type": 1, "bounce_threshold_velocity": 0.5, "friction_offset_threshold": 0.04, "friction_correlation_distance": 0.025, "enable_stabilization": false}`
  - 驗證：`rigid_body` 五個都是有限數且 ≥ 0；`solver_type` 是 0 或 1 的 int；三個門檻是有限數且 ≥ 0；`enable_stabilization` 是 bool。錯誤訊息要含欄位名稱。

- [ ] **Step 1：寫失敗的測試**：fixture `valid_meta` 加上這兩個物件（值如上）；新增 `test_rigid_body_and_physx_scene_required`（各刪一個鍵 → `PolicyMetaError` 含該鍵名）和 `test_physx_scene_values_checked`（`solver_type: 2`、`enable_stabilization: "no"`、`bounce_threshold_velocity: nan` 各自被拒，訊息含欄位名）。
- [ ] **Step 2：確認失敗**：`cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests/test_policy_meta.py -q`，預期新測試 FAIL。
- [ ] **Step 3：實作驗證**；`export_policy.py` 的 `build_meta` 從 `env_cfg.scene.robot.spawn.rigid_props`（`PhysxRigidBodyCfg`）與 `env_cfg.sim.physics`（PhysX 設定物件，欄位名以 `~/IsaacLab/source/isaaclab_physx` 為準）讀出這些值，`float()`/`int()`/`bool()` 轉型。
- [ ] **Step 4：確認通過**：同 Step 2，全部 PASS。
- [ ] **Step 5：重新匯出 v7 並比對訓練紀錄**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
RUN=$(ls -d ~/IsaacLab/logs/rsl_rl/robot_dog_flat/*_v7 | tail -1)
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/export_policy.py --run_dir $RUN --checkpoint model_2450.pt --force
grep -n -A3 "max_depenetration_velocity\|solver_type\|bounce_threshold\|friction_offset\|friction_correlation\|enable_stabilization" $RUN/params/env.yaml | head -30
```
預期：exit 0；`policy.json` 的 10 個新值和 `env.yaml` 一致（若 env.yaml 與上面寫的值不同，以 env.yaml 為準並更新 spec）。`policy.pt` 權重不變（位元組可能不同，比對 `state_dict` 最大差 0 後 `git checkout` 回舊檔，只提交 `policy.json`）。
- [ ] **Step 6：更新 spec 第 4.4 節範例，commit**：`git commit -m "policy.json: record rigid-body and PhysX scene settings"`

---

### Task 2：同步步進 `stepping.py` 與探針

**Files:**
- Create: `DESC/isaac/robot_dog_sim/stepping.py`
- Create: `DESC/isaac/tools/stepping_probe.py`

**Interfaces:**
- Produces（`stepping.py`，SimulationApp 之後 import）：
  - `class Lockstep: __init__(self, app, physics_dt: float, decimation: int)`：設定物理 dt、關掉 Kit 的自動物理步進、`timeline.play()`。
  - `Lockstep.step(self, pre_physics: Callable[[], None]) -> None`：`decimation` 次「`pre_physics()` → 一個物理步」，然後一次 `app.update()`（渲染、OmniGraph、感測器）。
  - `Lockstep.sim_time` → `SimulationManager.get_simulation_time()`；`Lockstep.physics_steps` → `SimulationManager.get_num_physics_steps()`。
- 作法 A（先試）：照 Isaac Lab `isaaclab_physx/physics/physx_manager.py`（約 L432–445 的 `playSimulations` 開關、L533–546 的 `simulate(dt, 0)` + `fetch_results()`），`physxScene:timeStepsPerSecond = 1/physics_dt`、`/persistent/simulation/minFrameRate = steps_per_sec // decimation`。
- 作法 B（A 不合格才用）：保留 Kit 自動步進，`timeStepsPerSecond` 200、`minFrameRate` 50，用 `SimulationManager.register_callback(cb, IsaacEvents.PRE_PHYSICS_STEP)` 在每個物理步前呼叫 `pre_physics`。

- [ ] **Step 1：實作 `stepping.py`（作法 A）與 `tools/stepping_probe.py`**
  探針：`require_ros_jazzy()` → headless `SimulationApp` → `kit.enable_extensions()` → `world.demo_world(stage, 200)` → 載入機器人（`robot.convert_urdf/load_robot`，依計畫 1 選 `physics` variant）→ `ros_graph.clock_graph(stage)` → rclpy 訂閱 `/clock` → `Lockstep(app, 0.005, 4)`，跑 200 次 `step(counter)`（`counter` 只計數）。每次記錄 `physics_steps` 增量、`sim_time` 增量、最新收到的 `/clock`。
  通過條件（印出並 exit 0，否則 exit 1）：每次增量都是 4 步與 0.020 s（誤差 < 1e-6）；`counter` 每次被呼叫 4 次；最後一則 `/clock` 與 `sim_time` 相差 < 0.02 s（OmniGraph 在 `app.update()` 時發佈）。
- [ ] **Step 2：執行**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/tools/stepping_probe.py; echo "exit=$?"
```
預期：`[stepping_probe] 200 cycles: 4 steps / 0.020 s each, /clock lag ...` 且 `exit=0`。不合格就改用作法 B 再跑；兩者都不合格時停下來回報實測數字（不要繼續猜）。
- [ ] **Step 3：Commit**：`git commit -m "Add lockstep stepping (4 x 5 ms physics per app update) and its probe"`，訊息寫明採用作法 A 或 B。

---

### Task 3：`actuator.py` 與 `observations.py`

**Files:**
- Create: `DESC/isaac/robot_dog_sim/actuator.py`、`DESC/isaac/robot_dog_sim/observations.py`
- Test: `DESC/isaac/tests/test_actuator.py`、`DESC/isaac/tests/test_observations.py`（uv 環境）

**Interfaces:**
- `actuator.py`：`dc_motor_torque(q_target, q, qd, actuator: dict) -> torch.Tensor`，`actuator` 是 `policy.json` 的 `actuator` 物件；公式見 spec 第 5.2 節（與 Isaac Lab `DCMotor._clip_effort` 相同）。
- `observations.py`：
  - `quat_rotate_inverse_wxyz(q: Tensor(4), v: Tensor(3)) -> Tensor(3)`
  - `joint_index_map(policy_joint_names: list[str], dof_names: list[str]) -> list[int]`：回傳每個策略關節在 articulation 中的索引；缺少時 raise `PolicyMetaError`，訊息列出兩邊的名稱。
  - `check_obs_terms(meta) -> None`：`obs_terms` 的名稱與順序必須正好是 `["base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos", "joint_vel", "actions"]`，且 `scale`、`clip` 都是 null，否則 raise `PolicyMetaError`（計畫 1 留下的檢查）。
  - `build_observation(meta, root_quat_wxyz, com_lin_vel_w, ang_vel_w, command, joint_pos, joint_vel, last_action) -> Tensor(48)`：`joint_pos`/`joint_vel` 已依策略順序排列；`joint_pos_rel = q − default_joint_pos`，`joint_vel_rel = qd`；重力方向 = 把 (0, 0, −1) 轉到機身座標。

- [ ] **Step 1：寫失敗的測試**
  - `test_dc_motor_matches_isaac_lab`：建 Isaac Lab `DCMotor`（建構方式照 `~/IsaacLab/source/isaaclab/test/actuators/test_dc_motor.py`），stiffness 20、damping 0.5、effort/saturation 25、velocity 30；隨機 1000 組 `(q_target, q, qd)`（qd 在 ±40 內，涵蓋截斷區），兩邊力矩最大差 < 1e-5。
  - `test_dc_motor_saturates_with_speed`：qd = 30（= velocity_limit）、大正誤差 → 力矩 ≤ 0 + 1e-6；qd = 0、大正誤差 → 25。
  - `test_rotate_inverse_matches_isaac_lab`：隨機單位四元數（wxyz）與向量，和 `isaaclab.utils.math.quat_apply_inverse`（xyzw，要換順序）比，差 < 1e-6。
  - `test_projected_gravity_level_and_pitched`：單位四元數 → (0, 0, −1)；繞機身 y 軸 +30°（wxyz = (cos 15°, 0, sin 15°, 0)）→ (0.5, 0, −0.866)，誤差 < 1e-6。
  - `test_joint_index_map_by_name`：dof 順序打亂後，索引能把它還原成策略順序；缺一個名稱 → `PolicyMetaError`，訊息含缺少的名稱。
  - `test_obs_terms_checked`：v7 `policy.json` 通過；把兩項對調或加 `scale: 2.0` → `PolicyMetaError`。
  - `test_build_observation_layout`：用 v7 `policy.json`，指定 command=(0.3, −0.1, 0.5)、`last_action=arange(12)`；檢查長度 48，`[9:12]` 等於 command，`[12:24]` 等於 `q − default`，`[36:48]` 等於 `last_action`。
- [ ] **Step 2：確認失敗**（uv 環境 pytest，`ModuleNotFoundError`）
- [ ] **Step 3：實作兩個模組**
- [ ] **Step 4：確認通過**，再跑一次 `python3 -m pytest tests -q`（純模組測試不受影響）
- [ ] **Step 5：Commit**：`git commit -m "Add DC-motor torque and observation builders for the Isaac Sim runner"`

---

### Task 4：指令緩衝、站立保持、跌倒判斷（純 Python）

**Files:**
- Create: `DESC/isaac/robot_dog_sim/command_buffer.py`
- Test: `DESC/isaac/tests/test_command_buffer.py`（系統 python3）

**Interfaces:**
- `class CommandBuffer: __init__(self, ranges: dict, timeout: float = 0.5)`；`update(self, cmd: tuple[float, float, float], t: float)`；`get(self, t: float) -> tuple[float, float, float]`：超過 `timeout` 沒更新回 (0, 0, 0)，否則回夾在 `ranges`（`policy.json` 的 `command_ranges`）內的指令。
- `class IdleHold: __init__(self, enabled: bool, hold_after: float = 0.3, lin_eps: float = 0.02, ang_eps: float = 0.05)`；`update(self, cmd, t) -> bool`：回傳這個策略步要不要保持站姿（指令小於門檻持續 `hold_after` 秒後為 True）；`resumed(self) -> bool`：上一步保持、這一步不保持時為 True（呼叫端要把 `last_action` 歸零）。
- `is_fallen(base_height: float, projected_gravity_z: float) -> bool`：高度 < 0.12 m 或傾斜 > 60°（`projected_gravity_z > −cos 60° = −0.5`）。

- [ ] **Step 1：寫失敗的測試**：`test_timeout_zeroes_command`（t=0 更新 (0.3, 0, 0)，t=0.49 → 原值，t=0.51 → 零）、`test_commands_are_clipped_to_training_ranges`（(1.5, −0.9, 2.0) → (0.8, −0.5, 1.0)）、`test_hold_after_idle_period`（零指令 0.29 s → False，0.31 s → True；收到 (0.1, 0, 0) → False 且 `resumed()` True 一次）、`test_hold_disabled`（`enabled=False` 永遠 False）、`test_small_command_is_not_idle`（(0.03, 0, 0) 不算 idle）、`test_is_fallen`（0.25 m 水平 → False；0.10 m → True；傾斜 70° → True）。
- [ ] **Step 2：確認失敗**；**Step 3：實作**；**Step 4：確認通過**（`python3 -m pytest tests -q`）
- [ ] **Step 5：Commit**：`git commit -m "Add command buffer with timeout and clipping, idle hold, fall check"`

---

### Task 5：`policy_runner.py` 與 Isaac Sim 端設定

**Files:**
- Create: `DESC/isaac/robot_dog_sim/policy_runner.py`
- Modify: `DESC/isaac/robot_dog_sim/robot.py`（加 `configure_for_policy`）、`DESC/isaac/robot_dog_sim/world.py`（加 `apply_physics_settings`、`policy_ground`）

**Interfaces:**
- `world.apply_physics_settings(stage, meta) -> None`：`physxScene:timeStepsPerSecond = round(1/sim_dt)`、`physx_scene` 各值、重力 9.81。
- `world.policy_ground(stage, meta) -> UsdShade.Material`：地面材質用 `materials.ground`（含 multiply 組合模式，`physxMaterial:frictionCombineMode` 等）。
- `robot.configure_for_policy(stage, root, articulation, meta) -> None`：所有連桿碰撞套 `materials.robot` 材質；`rigid_body` 各值；articulation solver iterations、關閉自碰撞；關節驅動改力矩模式、stiffness/damping 0；`set_dof_armatures`（`actuator.armature`）、`set_dof_max_efforts`（`effort_limit`）、`set_dof_max_velocities`（`velocity_limit`）；初始姿態 `default_joint_pos`、出生高度 `spawn_height`。
- `class PolicyRunner: __init__(self, meta: dict, policy_path: str, articulation, device: str = "cpu", hold: bool = True)`：
  - 啟動時 `check_obs_terms(meta)`，以 `joint_index_map` 建索引，載入 `policy.pt`（TorchScript），確認輸入 48、輸出 12。
  - `pre_physics(self) -> None`：讀關節位置與速度 → `dc_motor_torque(self.q_target, …)` → `set_dof_efforts`（依 articulation 索引）。
  - `policy_step(self, command: tuple, t: float) -> None`：`IdleHold.update`；保持時 `q_target = default`；否則讀根部姿態、**質心**世界線速度與角速度（`get_velocities()` 若是連桿原點速度，加上 ω × (R·r_com)，r_com 由 base_link 的 USD `physics:centerOfMass` 讀取），`build_observation` → 推論 → `last_action = action`，`q_target = default + action_scale·action`（`clip_actions` 非 null 時先夾住）。
  - 屬性 `fallen: bool`（`is_fallen`），由呼叫端決定要不要印。

- [ ] **Step 1：實作**（Isaac 相關部分沒有單元測試，正確性由 Task 6 的一致性測試驗證）
- [ ] **Step 2：靜態檢查**：`python3 -m py_compile` 所有 `robot_dog_sim/*.py`；`python3 -m pytest tests -q` 與 uv 環境的 Task 3 測試都通過。
- [ ] **Step 3：Commit**：`git commit -m "Add Isaac Sim policy runner with policy.json-driven physics setup"`

---

### Task 6：一致性測試（M2）

**Files:**
- Create: `DESC/isaac/robot_dog_sim/parity.py`（純 Python）、`DESC/isaac/tests/test_parity.py`
- Create: `DESC/isaaclab/scripts/parity_reference.py`（Isaac Lab 端，寫出參考數據）
- Create: `DESC/isaac/policy_parity.py`（Isaac Sim 端，比對）

**Interfaces:**
- `parity.py`：
  - `SEGMENTS: tuple[tuple[str, float, tuple[float, float, float]], ...]`：共 60 s，(名稱, 秒數, 指令)：站立 5、前進 0.3 8、前進 0.5 6、慢速前進 0.1 6、左側移 0.3 6、慢速右側移 0.1 6、左轉 0.8 5、慢速右轉 0.2 6、邊走邊轉 (0.3, 0, 0.5) 7、站立 5。
  - `WARMUP_FRACTION = 1/3`（每段前 1/3 不算，與 demo 相同）。
  - `segment_means(samples: list[tuple[float, float, float]], dt: float) -> list[tuple[float, float, float]]`：依 `SEGMENTS` 切段、去掉暖身、取平均。
  - `compare(ref: list, got: list, lin_tol=0.05, ang_tol=0.1) -> list[str]`：回傳每個超出容許值的段落描述，空清單代表通過。
- 參考檔：`parity_reference.py --policy_dir <dir> --out <json>`，用 `make_eval_env_cfg(1)`（標準機器人、seed 42），每個策略步覆寫指令（同 `gait_eval`），記錄機身座標系 (vx, vy, wz)，輸出 `{"policy": …, "dt": 0.02, "segments": [...], "means": [...], "falls": n}`。
- 比對：`policy_parity.py --policy <dir> --reference <json> [--headless]`：不接 ROS、不建感測器（`require_ros_jazzy` 不呼叫），`world.policy_ground` + 地面 + 機器人 + `PolicyRunner` + `Lockstep`；同樣的指令序列、同樣記錄方式；印出每段兩邊數值與差異，`compare` 為空且兩邊都沒跌倒 → exit 0。

- [ ] **Step 1：寫失敗的測試** `test_parity.py`：`SEGMENTS` 總長 60 s；`segment_means` 對常數輸入回傳該常數、能正確丟掉暖身；`compare` 在 0.049 / 0.051 m/s 與 0.099 / 0.101 rad/s 的邊界正確判斷，回傳訊息含段落名稱。
- [ ] **Step 2：確認失敗**；**Step 3：實作 `parity.py`**；**Step 4：確認通過**
- [ ] **Step 5：實作 `parity_reference.py` 並產生參考數據**

```bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
P=~/dog_12dof_ws/robot_dog_description/isaac/policies/2026-11-01_20-43-36_v7
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/parity_reference.py --policy_dir $P --out $P/parity_reference.json
```
預期：exit 0，`falls: 0`，每段平均速度接近指令（和 v7 的 `gait_eval` 一致）。
- [ ] **Step 6：實作 `policy_parity.py` 並執行**

```bash
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/policy_parity.py --policy $P --reference $P/parity_reference.json --headless; echo "exit=$?"
```
預期：10 段全部 PASS、`exit=0`。**不通過時**依序檢查（每次只改一處、重跑）：(1) 關節順序與 `default_joint_pos`；(2) 質心速度修正與四元數順序；(3) 致動器力矩與 armature 是否真的設上（印第一個策略步的力矩、`get_dof_armatures()`）；(4) 材質與組合模式；(5) PhysX scene / solver 設定。修三輪仍不過就停下來回報數據。
- [ ] **Step 7：Commit**（含 `parity_reference.json`）：`git commit -m "M2: Isaac Sim runner matches Isaac Lab within 0.05 m/s (policy_parity)"`

---

### Task 7：ROS 套件 `robot_dog_navigation` 與兩個節點

**Files:**
- Create: `NAV/package.xml`、`NAV/setup.py`、`NAV/setup.cfg`、`NAV/resource/robot_dog_navigation`、`NAV/robot_dog_navigation/__init__.py`
- Create: `NAV/robot_dog_navigation/footprint_math.py`、`odom_footprint_node.py`、`ground_filter_math.py`、`scan_ground_filter_node.py`
- Create: `NAV/launch/bridge.launch.py`
- Test: `NAV/test/test_footprint_math.py`、`NAV/test/test_ground_filter_math.py`

**Interfaces:**
- `footprint_math.py`：`quat_to_rpy(x, y, z, w) -> (roll, pitch, yaw)`、`rpy_to_quat(roll, pitch, yaw) -> (x, y, z, w)`、`split_footprint(position, quat_xyzw) -> (footprint_xy_yaw: (x, y, yaw), tilt: (z, roll, pitch))`（ROS 訊息是 xyzw）。
- `odom_footprint_node`：訂閱 `odom_gt`（nav_msgs/Odometry，`odom`→`base_link`），發佈 `odom`（`odom`→`base_footprint`，x、y、yaw 與平面速度）與 TF `odom→base_footprint`、`base_footprint→base_link`；邏輯移植自 `~/robotdog_ws/dog_gait/dog_gait/odom_footprint_node.py`（訂閱話題名稱改為 `odom_gt`）。
- `ground_filter_math.py`：`floor_mask(ranges, angle_min, angle_increment, range_min, laser_quat_xyzw, laser_z, min_height=0.05) -> np.ndarray[bool]`：把每個有限回波轉到 `odom` 座標，高度 < `min_height` 的為 True。
- `scan_ground_filter_node`：訂閱 `scan`，用掃描時間戳查 TF `odom→laser_link`，把 `floor_mask` 的點設為 `inf`，發佈 `scan_filtered`；移植自舊工作區 `scan_ground_filter_node.py`。**這是 spec 第 6.2 節作法的修正**（spec 原寫「打地距離 − 0.05 m」，改為已在舊狗驗證過的「回波高度 < 0.05 m」），同 Task 更新 spec。
- `bridge.launch.py`：robot_state_publisher（`robot_dog_description/urdf/robot_dog.urdf.xacro sim_mode:=none`）、`odom_footprint_node`、`scan_ground_filter_node`，全部 `use_sim_time: true`。

- [ ] **Step 1：寫失敗的測試**：`test_split_recomposes_original`（隨機 roll/pitch/yaw ±0.5 rad：`Rz(yaw)·R(roll,pitch)` 等於原四元數，誤差 < 1e-9）、`test_level_body_has_no_tilt`；`test_level_scan_drops_nothing`（雷射高 0.34 m、水平、所有距離 5 m → 全 False）、`test_pitched_scan_drops_floor_hits`（前傾 5°，正前方 10 m 的回波 → True；正後方 → False）、`test_wall_is_kept_when_pitched`（前傾 5°，正前方 1 m 的回波高度約 0.25 m → False）、`test_inf_and_short_ranges_ignored`。
- [ ] **Step 2：確認失敗**（`cd ~/dog_12dof_ws/robot_dog_navigation && python3 -m pytest test -q`）
- [ ] **Step 3：實作數學模組、節點、套件檔、launch**；`setup.py` 的 `console_scripts`：`odom_footprint_node`、`scan_ground_filter_node`；`data_files` 安裝 `launch/`。
- [ ] **Step 4：確認通過**，並 `cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select robot_dog_navigation`，預期 `1 package finished`。
- [ ] **Step 5：更新 spec 第 6.2 節的濾波作法，commit**：`git commit -m "Add robot_dog_navigation: odom_footprint and scan_ground_filter nodes"`

---

### Task 8：導航模式 `robot_dog_nav_sim.py` 與 M3 驗收

**Files:**
- Modify: `DESC/isaac/robot_dog_sim/world.py`（加 `room_world(stage, meta)`，spec 第 5.5 節）
- Create: `DESC/isaac/robot_dog_nav_sim.py`
- Modify: `DESC/README.md`（Isaac Sim 導航模式一節）、新增 `NAV/README.md`（bridge 啟動與話題/TF 表）

**Interfaces:**
- `world.room_world(stage, meta)`：10 × 10 m 房間，外牆高 1.0 m、厚 0.1 m；兩道內牆把房間分成三區、各留 1.0 m 通道；4 個方塊（0.4–0.6 m）與 3 個圓柱（直徑 0.3 m），出生點 1 m 內淨空；地面用 `policy_ground`。牆和障礙物的座標寫成模組常數，計畫 3 的 `nav_eval` 目標點會用。
- `robot_dog_nav_sim.py` 參數：`--policy`（預設 `policies/` 下名稱排序最後的資料夾）、`--world`（USD，選填）、`--headless`、`--no-camera`、`--no-hold`、`--max-frames`。
  - 啟動：`require_ros_jazzy()` → `SimulationApp` → `load_policy_meta` → 世界與機器人（`configure_for_policy`）→ OmniGraph：`clock_graph`、`joint_state_graph(stage, art, '/joint_states')`、IMU、LiDAR（**失敗就結束，exit 1**）、相機（失敗或 `--no-camera` 只警告）。
  - rclpy 節點（同程序）：訂閱 `/cmd_vel` → `CommandBuffer.update(cmd, sim_time)`；每個策略步發佈 `/odom_gt`（nav_msgs/Odometry，`odom`→`base_link`，根部位置/姿態與質心速度，twist 用機身座標系，stamp = `sim_time`）。
  - 主迴圈：`Lockstep.step(runner.pre_physics)`；每循環 `rclpy.spin_once(node, timeout_sec=0)`、`runner.policy_step(buffer.get(sim_time), sim_time)`、發佈 `/odom_gt`；`runner.fallen` 第一次變 True 時印 `[robot_dog] FALLEN at t=… s`。

- [ ] **Step 1：實作 `room_world` 與 `robot_dog_nav_sim.py`**
- [ ] **Step 2：smoke**：`... robot_dog_nav_sim.py --headless --no-camera --max-frames 300`，預期 IMU、LiDAR 等 `ok`、沒有 `FALLEN`、exit 0。
- [ ] **Step 3：話題與 TF**（終端 A：nav sim 不加 `--max-frames`；終端 B：`ros2 launch robot_dog_navigation bridge.launch.py`）
  - `ros2 topic hz` 換算成 sim time：`/clock`、`/joint_states`、`/imu`、`/odom_gt`、`/odom` 約 50 Hz，`/scan` 與 `/scan_filtered` 約 10 Hz，`/camera/image_raw` 約 15 Hz（沒加 `--no-camera` 時）。
  - `ros2 run tf2_tools view_frames` 的樹：`odom → base_footprint → base_link → {腿、imu_link、laser_link、camera_link → camera_optical_link}`，每段只有一個發佈者。
- [ ] **Step 4：運動與逾時**（用指令代替 teleop，`ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist ...` 各送 5 s，讀 `/odom` 位移）
  - 前進 vx 0.3：x 增加 1.0–2.0 m；側移 vy 0.3：y 增加 1.0–2.0 m；轉向 wz 0.8：yaw 改變 2.5–5.5 rad。
  - 停止發佈後：0.5 s（sim time）內 `/odom_gt` 的平面速度 < 0.05 m/s，沒有 `FALLEN`。
  - 發佈 vx 1.5：實際速度不超過約 0.85 m/s（被夾在 0.8）。
- [ ] **Step 5：README**：`DESC/README.md` 導航模式啟動方式與參數；`NAV/README.md` bridge launch、話題/TF 表（spec 第 3.1、3.2 節）。
- [ ] **Step 6：Commit**：`git commit -m "M3: Isaac Sim navigation mode with /cmd_vel, sensors, odometry and TF"`

---

## 計畫 2 完成條件（= spec M2 + M3）

- `python3 -m pytest DESC/isaac/tests NAV/test -q` 全部通過；uv 環境的 `test_actuator.py`、`test_observations.py` 通過。
- `stepping_probe.py` exit 0（每循環 4 步、0.02 s）。
- `policy_parity.py` 10 段全部在 0.05 m/s、0.1 rad/s 內，兩邊都沒跌倒。
- 導航模式：話題頻率與 TF 樹符合 spec 3.1/3.2；前進、側移、轉向都動得起來；停送 `/cmd_vel` 0.5 s 內站住；超範圍指令被夾住。
