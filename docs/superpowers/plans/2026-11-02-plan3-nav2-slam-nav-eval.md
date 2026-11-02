# 計畫 3：slam_toolbox 建圖 + AMCL + Nav2 導航 + nav_eval（M4 + M5）實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Isaac Sim 的房間裡，用 slam_toolbox 建出地圖（M4），並讓 Nav2 用 SLAM 與 AMCL 兩種模式都能依序抵達 5 個目標點（M5），一條 `ros2 launch` 啟動整套 ROS 端。

**Architecture:** Isaac Sim 端不動（`robot_dog_nav_sim.py` 收 `/cmd_vel`、發感測器與 `/odom_gt`）。ROS 端 `robot_dog_navigation` 新增：Nav2 參數（從 Jazzy 預設的 `nav2_params.yaml` 改成這隻狗：`base_footprint`、`/scan_filtered`、MPPI Omni、AMCL Omni）、`nav.launch.py`（bridge + `nav2_bringup/bringup_launch.py` + RViz）、房間佈局與目標點（純 Python，有測試）、地圖品質檢查 `map_check`、`nav_eval`（`nav2_simple_commander`）。

**Tech Stack:** ROS 2 Jazzy、Nav2（MPPI、NavFn、velocity_smoother、collision_monitor）、slam_toolbox、nav2_amcl、nav2_map_server、nav2_simple_commander、RViz2、pytest；Isaac Sim 6.2（計畫 2 的導航模式）

**Spec:** `docs/superpowers/specs/2026-11-01-robot-dog-isaac-nav2-design.md`（第 3、6.3–6.5、7 M4/M5 節，以及「實作分期」裡計畫 2 留下的事項）

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`DESC` = `~/dog_12dof_ws/robot_dog_description`，`NAV` = `~/dog_12dof_ws/robot_dog_navigation`。
- Isaac Sim：`source /opt/ros/jazzy/setup.bash && cd ~/IsaacLab && OMNI_KIT_ACCEPT_EULA=yes uv run --extra isaacsim python $DESC/isaac/robot_dog_nav_sim.py --headless`（自動化驗證時加 `--no-camera`）。同一時間只跑一個 Isaac 程序。
- ROS 端：`cd ~/dog_12dof_ws && source install/setup.bash`；改了 `NAV` 後 `colcon build --symlink-install --packages-select robot_dog_navigation`。
- 所有 Nav2 / slam_toolbox / AMCL 節點 `use_sim_time: true`；機器人座標系一律 `base_footprint`；雷射一律 `/scan_filtered`。
- 速度上限（spec 6.4）：vx −0.3～0.5、vy ±0.3、wz ±0.8，都在策略的訓練範圍內（vx ±0.8、vy ±0.5、wz ±1.0）。
- 房間佈局只有一個來源：`DESC/isaac/robot_dog_sim/room_layout.py`（純 Python）；`world.py`、目標點、地圖檢查都從它讀。
- 背景程序結束時用 PID 停止（`ps -eo pid,args | awk ...` 先存 PID 再 `kill`），不要用會比對到自己指令列的 `pkill -f`。
- 程式註解英文、文件繁體中文；commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。

## Review Focus

1. **到不了的目標**：目標點落在障礙物上或被包住時，`nav_eval` 要在逾時後取消並記為失敗、繼續下一個，不能卡住（Task 2 `test_judge_*`、Task 6 實測一個故意放在箱子上的目標）。
2. **AMCL 初始位置**：地圖模式啟動後，`map→odom` 要從原點開始，不能等人在 RViz 按 2D Pose Estimate（Task 4 參數 `set_initial_pose: true`，Task 6 檢查第一個目標前的定位誤差 < 0.1 m）。
3. **Isaac Sim 比即時慢**：Nav2 的進度檢查、BT 逾時、TF 容忍度都要以 sim time 計，模擬變慢不能讓導航誤判卡住（Task 1 先量即時率，Task 4 的 `transform_tolerance`、progress checker 以實測為準）。
4. **地圖和世界座標對齊**：狗從原點出發、`map` 原點等於出生點，`map_check` 才能直接拿 `room_layout` 比對；若 slam_toolbox 的 `map` 原點不是出生點要先換算（Task 3 測試、Task 5 實測）。
5. **抵達判定**：Nav2 回報 SUCCEEDED 不夠，`nav_eval` 要用 `/odom_gt`（真值）量最終誤差（0.25 m、0.3 rad），並在途中偵測跌倒（機身 < 0.12 m）。

---

### Task 1：計畫 2 留下的三件事（即時率、`/cmd_vel` 積壓、雷射 TF 等待）

**Files:**
- Modify: `DESC/isaac/robot_dog_nav_sim.py`（每循環把待處理的 ROS 回呼全部處理完，最多 20 個）
- Modify: `NAV/robot_dog_navigation/scan_ground_filter_node.py`（`TransformListener(self.buf, self, spin_thread=True)`，查 TF 的等待改成 0，保留現有的「最新 TF」備案）
- Modify: `DESC/README.md`（記錄即時率）

**Interfaces:**
- `robot_dog_nav_sim.py`：節點加入 `rclpy.executors.SingleThreadedExecutor`；每個循環重複 `executor.spin_once(timeout_sec=0.0)`，最多 20 次，讓這段時間收到的 `/cmd_vel` 都處理掉，`CommandBuffer` 只留最新一則。

- [ ] **Step 1：量即時率**：導航模式 headless，分別「有相機」與 `--no-camera` 各跑 30 s 牆鐘時間，讀 `/clock` 前後差，算 sim/wall。記到 `DESC/README.md` 導航模式一節。
- [ ] **Step 2：實作兩個修改**
- [ ] **Step 3：驗證**：nav sim + `bridge.launch.py`，用 `ros2 topic pub -r 50 /cmd_vel ...`（比策略頻率快）送 vx 0.3 五秒再停，停送後 1 s 內 `/odom_gt` 平面速度 < 0.05 m/s（沒有積壓的舊指令）；`/scan_filtered` 與 `/scan` 的時間戳差 < 0.03 s（不再落後一個循環）。
- [ ] **Step 4：Commit**：`git commit -m "Drain /cmd_vel each cycle, non-blocking TF in scan filter, record real-time factor"`

---

### Task 2：房間佈局單一來源與目標點

**Files:**
- Create: `DESC/isaac/robot_dog_sim/room_layout.py`（純 Python）
- Modify: `DESC/isaac/robot_dog_sim/world.py`（`room_world` 改從 `room_layout` 讀；移除 world.py 內的同名常數）
- Create: `NAV/robot_dog_navigation/nav_goals.py`（純 Python）
- Test: `DESC/isaac/tests/test_room_layout.py`、`NAV/test/test_nav_goals.py`

**Interfaces:**
- `room_layout.py`：`ROOM_HALF = 5.0`、`WALL_HEIGHT = 1.0`、`WALL_THICKNESS = 0.1`、`INNER_WALLS`、`BOXES`、`CYLINDERS`、`CYLINDER_RADIUS`（值照計畫 2 的 `world.py`）；
  `obstacles() -> list[tuple[str, tuple[float, float, float, float]]]`：每個障礙物（含外牆、內牆分段、箱子）的 2D 外框 `(xmin, ymin, xmax, ymax)`，圓柱用外接正方形；
  `clearance(x, y) -> float`：點到最近障礙物外框的距離（在外框內為 0）；
  `zone(x) -> int`：0 = 西區（x < −1.7）、1 = 中區、2 = 東區。
- `nav_goals.py`（NAV 內，用 `sys.path` 插入 `DESC/isaac` 匯入 `room_layout`，找不到時用 `ament_index` 的 `robot_dog_description` share 路徑）：
  - `GOALS: tuple[tuple[str, float, float, float], ...]`：(名稱, x, y, yaw)，5 個：中區東北角、穿過東側門 (x 1.7, y −3) 到東區、東區北側、穿回中區再穿過西側門 (x −1.7, y 3) 到西區、西區南側。座標由實作者在下列限制下選定並寫死。
  - `pose_error(goal, x, y, yaw) -> (dist, yaw_err)`：`yaw_err` 取 [−π, π] 內的絕對值。
  - `judge(goal, final_pose, status, fell, elapsed, timeout=120.0) -> tuple[bool, str]`：status 不是 `'SUCCEEDED'`、跌倒、逾時、`dist > 0.25` 或 `yaw_err > 0.3` 都失敗，原因字串含對應關鍵字（`status`、`fall`、`timeout`、`distance`、`yaw`）。

- [ ] **Step 1：寫失敗的測試**
  - `test_room_layout.py`：`clearance(0, 0) >= 1.0`（出生點淨空）；`clearance` 在箱子中心為 0；`zone(-3) == 0`、`zone(0) == 1`、`zone(3) == 2`；`obstacles()` 含 4 道外牆 + 4 段內牆 + 4 箱 + 3 柱。
  - `test_nav_goals.py`：5 個目標都在房內且 `clearance >= 0.5`；三個區都至少有一個目標；相鄰目標（含從原點到第 1 個）至少有兩次換區（= 穿過內牆門口）；`judge` 的五種失敗各有一個案例、一個成功案例；`pose_error` 的 yaw 環繞（3.1 與 −3.1 差 0.08）。
- [ ] **Step 2：確認失敗**；**Step 3：實作**（`world.py` 改用 `room_layout`，行為不變）；**Step 4：確認通過**（`python3 -m pytest DESC/isaac/tests NAV/test -q`）
- [ ] **Step 5：nav sim smoke 不變**：`robot_dog_nav_sim.py --headless --no-camera --max-frames 300` exit 0、LiDAR ok。
- [ ] **Step 6：Commit**：`git commit -m "Room layout as a single source; nav_eval goals and judging"`

---

### Task 3：地圖品質檢查 `map_check`

**Files:**
- Create: `NAV/robot_dog_navigation/map_check.py`（純 Python + numpy；讀 map_saver 的 `.yaml` + `.pgm`）
- Test: `NAV/test/test_map_check.py`

**Interfaces:**
- `load_map(yaml_path) -> (grid: np.ndarray[int8], resolution, origin_xy)`：pgm 依 yaml 的 `occupied_thresh`/`free_thresh`/`negate` 轉成 100 = 佔據、0 = 空、−1 = 未知；列序翻轉成「第 0 列 = y 最小」。
- `occupied_points(grid, resolution, origin) -> np.ndarray (N, 2)`：佔據格中心的世界座標。
- `check_map(points, obstacles, tol=0.10) -> dict`：`{'aligned': 佔據點中距離某障礙物外框 ≤ tol 的比例, 'outer_wall_coverage': 四道外牆內側線上每 0.1 m 取樣、附近 0.15 m 內有佔據點的比例}`。
- 通過條件（M4）：`aligned >= 0.95` 且 `outer_wall_coverage >= 0.80`。

- [ ] **Step 1：寫失敗的測試**：用 `room_layout.obstacles()` 畫出一張合成地圖（0.05 m 格）→ `aligned > 0.99`、`coverage > 0.95`；把同一張圖整體平移 0.3 m → `aligned < 0.5`；只畫一半外牆 → `coverage` 約 0.5；`load_map` 對手寫的 3×3 pgm（P5）與 yaml 讀出正確的值與列序。
- [ ] **Step 2–4：確認失敗、實作、確認通過**
- [ ] **Step 5：Commit**：`git commit -m "Add map_check: compare a saved map with the room layout"`

---

### Task 4：Nav2 參數、`nav.launch.py`、RViz

**Files:**
- Create: `NAV/config/nav2_params.yaml`（複製 `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` 後修改）
- Create: `NAV/launch/nav.launch.py`、`NAV/rviz/nav.rviz`（由 `nav2_bringup/rviz/nav2_default_view.rviz` 修改）
- Modify: `NAV/setup.py`（安裝 `config/`、`rviz/`、`maps/`）、`NAV/package.xml`（exec_depend：nav2_bringup、slam_toolbox、nav2_simple_commander、rviz2）

**Interfaces（`nav2_params.yaml` 相對 Jazzy 預設的修改，其餘保持預設）：**
- 所有 `robot_base_frame` / `base_frame_id` / `base_frame` = `base_footprint`；所有 `use_sim_time: true`。
- `amcl`：`robot_model_type: "nav2_amcl::OmniMotionModel"`、`scan_topic: scan_filtered`、`set_initial_pose: true`、`initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}`、`max_beams` 預設、`laser_max_range` = 雷射實際最遠距離（看 `/scan` 的 `range_max`）。
- `slam_toolbox`（新增區段，`slam:=True` 時 `bringup_launch.py` 用同一個參數檔）：從 `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` 複製，`base_frame: base_footprint`、`odom_frame: odom`、`map_frame: map`、`scan_topic: /scan_filtered`、`resolution: 0.05`、`max_laser_range` 同上、`mode: mapping`、`use_sim_time: true`。
- `controller_server`：`controller_frequency: 20.0`；`FollowPath`（MPPI）`motion_model: "Omni"`、`vx_max: 0.5`、`vx_min: -0.3`、`vy_max: 0.3`、`wz_max: 0.8`；critics 加 `"TwirlingCritic"`（Omni 模型建議），其餘預設；`general_goal_checker`：`xy_goal_tolerance: 0.20`、`yaw_goal_tolerance: 0.25`（比 nav_eval 的 0.25/0.3 嚴一點）。
- `local_costmap` / `global_costmap`：改用 `footprint: "[[0.22, 0.16], [0.22, -0.16], [-0.22, -0.16], [-0.22, 0.16]]"`（刪掉 `robot_radius`）、`inflation_radius: 0.35`、obstacle layer `observation_sources: scan` 且 `scan.topic: /scan_filtered`、`raytrace_max_range` / `obstacle_max_range` 不超過雷射最遠距離。
- `velocity_smoother`：`max_velocity: [0.5, 0.3, 0.8]`、`min_velocity: [-0.3, -0.3, -0.8]`、`max_accel: [1.0, 1.0, 2.0]`、`max_decel: [-1.0, -1.0, -2.0]`、`deadband_velocity: [0.0, 0.0, 0.0]`。
- `collision_monitor`：`observation_sources: ["scan"]`，`scan.topic: "/scan_filtered"`；`base_frame_id: base_footprint`。
- `transform_tolerance` 有出現的地方設 `0.5`。
- `nav.launch.py` 參數：`slam`（預設 `True`）、`map`（預設 `NAV` share 的 `maps/robot_dog_room.yaml`）、`params_file`（預設 `config/nav2_params.yaml`）、`rviz`（預設 `true`）；內容：`bridge.launch.py` + `nav2_bringup/launch/bringup_launch.py`（`slam`、`map`、`params_file`、`use_sim_time: true`、`autostart: true`）+ RViz（`rviz:=true` 時）。
- `nav.rviz`：Fixed Frame `map`；顯示 Map、`/scan_filtered`、RobotModel、TF、local/global costmap、`/plan`、Nav2 面板、`/camera/image_raw` Image 面板。

- [ ] **Step 1：寫 `nav2_params.yaml`**（上面的修改；`maps/` 先放一個佔位說明檔，地圖在 Task 5 產生）
- [ ] **Step 2：寫 `nav.launch.py`、`nav.rviz`，更新 `setup.py`/`package.xml`，`colcon build`**
- [ ] **Step 3：SLAM 模式啟動檢查**（nav sim + `ros2 launch robot_dog_navigation nav.launch.py slam:=True rviz:=false`）：
  `ros2 lifecycle get /bt_navigator`、`/controller_server`、`/planner_server` 都是 `active`；TF `map→odom` 存在；`/map` 有發佈；`/local_costmap/costmap` 有發佈；log 沒有 `ERROR`（警告可接受但要記下）。
- [ ] **Step 4：Commit**：`git commit -m "Nav2 parameters, nav.launch.py and RViz for robot_dog"`

---

### Task 5：M4 建圖與存圖

**Files:**
- Create: `NAV/scripts/tour.py`（以 `nav2_simple_commander` 依序送 `nav_goals.GOALS`，SLAM 模式下邊走邊建圖；`nav_eval` 的前身，只回報到達與否）
- Create: `NAV/maps/robot_dog_room.yaml`、`robot_dog_room.pgm`（`map_saver_cli` 產生）
- Create: `NAV/scripts/check_map.py`（命令列包裝 `map_check`，印出兩個數值，通過 exit 0）

- [ ] **Step 1：實作 `tour.py`、`check_map.py`**（`setup.py` 的 `console_scripts` 加 `tour`、`check_map`）
- [ ] **Step 2：建圖**：nav sim + `nav.launch.py slam:=True rviz:=false` + `ros2 run robot_dog_navigation tour`，預期 5 個目標大多到達（M4 不要求全到，只要繞過三個區）。
- [ ] **Step 3：存圖**：`ros2 run nav2_map_server map_saver_cli -f $NAV/maps/robot_dog_room`（不要加 `use_sim_time:=true`，舊狗紀錄會逾時）。
- [ ] **Step 4：檢查**：`ros2 run robot_dog_navigation check_map $NAV/maps/robot_dog_room.yaml`，預期 `aligned >= 0.95`、`outer_wall_coverage >= 0.80`、exit 0。不通過時看 `map→odom` 的漂移（odom 是真值，所以漂移就是 SLAM 誤差）、`/scan_filtered` 是否還有地面點、`max_laser_range`。
- [ ] **Step 5：Commit 地圖**：`git commit -m "M4: SLAM map of the room (map_check aligned >= 0.95)"`

---

### Task 6：`nav_eval.py` 與 M5

**Files:**
- Create: `NAV/scripts/nav_eval.py`（`console_scripts`：`nav_eval`）
- Modify: `NAV/README.md`、`DESC/README.md`（導航一節）、spec 第 6.5 / 7 節（結果）

**Interfaces:**
- `nav_eval [--slam] [--timeout 120] [--extra-goal x,y,yaw]`：
  - `BasicNavigator()`，`use_sim_time` 參數設 true；SLAM 模式 `waitUntilNav2Active(localizer='robot_localization')`（略過 localizer 生命週期檢查），地圖模式 `waitUntilNav2Active()`（AMCL）。
  - 地圖模式在第一個目標前讀 TF `map→base_footprint` 與 `/odom_gt`，差距 > 0.1 m 就先 `setInitialPose(原點)` 並等 2 s（Review Focus 2），並把該差距印出。
  - 依序 `goToPose`；每 0.1 s 檢查 `isTaskComplete()`、`/odom_gt` 的 z（< 0.12 m 記為跌倒並 `cancelTask()`）、經過的 sim time（超過 `timeout` 就 `cancelTask()`）；結束後用 `/odom_gt` 的位置與 yaw 呼叫 `nav_goals.judge`。
  - 印出每個目標一行 `[NAV] <名稱> <PASS/FAIL> dist=… yaw=… t=… <原因>`，最後 `[NAV] PASS k/5`；5 個都通過 exit 0，否則 1。`--extra-goal` 多加一個目標（只用於 Review Focus 1 的實測，不算進 5 個）。
- [ ] **Step 1：實作 `nav_eval.py`**（判定邏輯全用 Task 2 的 `nav_goals`，這裡只負責 ROS 互動）
- [ ] **Step 2：SLAM 模式**：nav sim（重新啟動，狗回到原點）+ `nav.launch.py slam:=True rviz:=false` + `ros2 run robot_dog_navigation nav_eval --slam`，預期 `[NAV] PASS 5/5`、exit 0。
- [ ] **Step 3：AMCL 模式**：nav sim（重新啟動）+ `nav.launch.py slam:=False rviz:=false` + `ros2 run robot_dog_navigation nav_eval`，預期初始定位差距 < 0.1 m、`[NAV] PASS 5/5`。
- [ ] **Step 4：到不了的目標**：AMCL 模式加 `--extra-goal` 放在一個箱子中心（排在 5 個目標之後），預期它在逾時或 Nav2 回報失敗後印出 FAIL，程式不卡住並正常結束；extra 目標的結果只印出、**不影響 exit code**（exit code 只看 5 個正式目標）。
- [ ] **Step 5：不通過時依序檢查**（每次只改一項、重跑）：(1) 小指令時狗是否真的在動（`/cmd_vel` vs `/odom` 速度）；(2) MPPI critics 權重與 `vx_min`（後退）；(3) inflation 與 footprint 是否讓門口變窄到過不去；(4) goal checker 容許值；(5) progress checker（模擬慢於即時時）。三輪仍不過就停下來回報數據。
- [ ] **Step 6：文件與 commit**：README 加「導航」使用說明（啟動、Nav2 Goal、存圖、nav_eval）；spec 記錄 M4/M5 結果。`git commit -m "M5: Nav2 reaches 5/5 goals in SLAM and AMCL modes (nav_eval)"`

---

## 計畫 3 完成條件（= spec M4 + M5）

- `python3 -m pytest DESC/isaac/tests NAV/test -q` 全部通過。
- `NAV/maps/robot_dog_room.*` 已提交，`check_map` 通過（aligned ≥ 0.95、外牆覆蓋 ≥ 0.80）。
- `nav_eval --slam` 與 `nav_eval`（AMCL）都 `[NAV] PASS 5/5`，全程沒有跌倒；到不了的目標會被判失敗而不會卡住。
- `ros2 launch robot_dog_navigation nav.launch.py` 一條指令啟動 ROS 端；README 說明在 RViz 按 Nav2 Goal 導航。
