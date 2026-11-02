# 12 自由度機器狗：Isaac Lab 步態訓練 + Isaac Sim 模擬 + Nav2 導航 設計

日期：2026-11-01　作者：willie

## 1. 目標與範圍

讓 `robot_dog_description` 的 12 自由度機器狗在 **Isaac Sim 6.2** 裡，用 **Isaac Lab 3.0** 訓練出的步態策略走路，
並由 **Nav2** 自主導航（RViz 點 Nav2 Goal → 狗避開障礙物走到目標）。

| 決定 | 內容 |
|---|---|
| 最終目標 | **只做模擬**，不上實機。里程計、機身線速度可直接用模擬器真值 |
| 場景 | **平地室內**（房間、內牆、方塊、圓柱） |
| 感測器 | IMU、2D LiDAR（建圖與避障唯一來源）、RGB 相機（**只發佈影像**，不進 Nav2） |
| 步態策略執行位置 | **Isaac Sim 腳本內**（同步步進、與訓練同一個 PhysX），ROS 2 只收 `/cmd_vel` |
| 主角 | 12 自由度狗（`~/dog_12dof_ws`）。舊 8 自由度工作區 `~/robotdog_ws` 只作為參考，不修改 |

### 非目標

- sim2real：狀態估測、實機驅動、機上算力。
- 崎嶇地形、樓梯、高度掃描器、3D 感知。
- 相機視覺任務（物體偵測、跟隨），以及深度相機進 costmap。
- 用 RL 訓練導航或避障策略（Nav2 負責上層）。
- Gazebo 的導航支援（現有 Gazebo launch 保持原樣，不擴充）。

### 成功條件

`nav_eval.py` 在 SLAM 模式和 AMCL 模式下，都能讓狗依序到達 5 個固定目標（位置 0.25 m、方向 0.3 rad 內），
全程不跌倒（詳見第 7 節 M5）。

## 2. 現況（2026-11-01）

- `robot_dog_description`：URDF（`robot_dog.urdf.xacro`、`robot_dog_rl.urdf`）、ros2_control（Gazebo / Isaac
  `JointStateTopicSystem`）、`isaac/robot_dog_isaac.py`（353 行：匯入、感測器、OmniGraph ROS 2 圖）、
  Isaac Lab 專案檔 `isaaclab/robot_dog_lab/`（`RobotDog-Velocity-Flat`）。**都還沒實際執行過。** 沒有 Nav2。
- 機器：Ubuntu 24.04、RTX 5070 Ti Laptop 12 GB、30 GB RAM；ROS 2 Jazzy + nav2 + slam_toolbox + robot_localization；
  `~/IsaacLab`（3.0.0，uv 環境，內含 `isaacsim 6.2.0` pip 版、Python 3.12）。沒有 `~/isaacsim` 獨立安裝。
- `~/robotdog_ws`（8 自由度、Gazebo）已驗證過 `odom_footprint_node`、`scan_ground_filter_node`、Nav2 參數、
  RL 策略節點，這次移植前兩者的邏輯。

## 3. 架構

```
┌─────────── Isaac Sim 6.2 程序（isaac/robot_dog_nav_sim.py）───────────┐
│ 世界：world.py 產生的室內場景                                           │
│ 物理 200 Hz ─每 4 步─► policy_runner 50 Hz（torch，讀 articulation 狀態）│
│                       └─► 關節目標 → DC 馬達模型（每物理步）→ 關節力矩      │
│ 感測器：IMU、RTX 2D LiDAR、RGB 相機                                     │
│ OmniGraph ROS 2 發佈器；rclpy 訂閱 /cmd_vel                             │
└───────▲──────────────────────────────────────────────┬──────────────┘
   sub  │ /cmd_vel                        pub │ /clock /scan /imu /camera/*
        │                                     │ /joint_states /odom_gt
┌───────┴──────────────── ROS 2 Jazzy ─────────▼──────────────────────┐
│ odom_footprint：/odom_gt → /odom + TF odom→base_footprint→base_link    │
│ scan_ground_filter：/scan + TF → /scan_filtered                       │
│ robot_state_publisher：/joint_states → 腿與感測器 TF                    │
│ slam_toolbox（SLAM 模式）或 map_server + amcl（地圖模式）                  │
│ Nav2：NavFn + MPPI(Omni) + velocity_smoother + collision_monitor       │
│ RViz：地圖、雷射、costmap、路徑、模型、相機影像、Nav2 Goal                  │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.1 話題

| 話題 | 型別 | 發佈者 | frame | 頻率（sim time） |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Isaac Sim | — | 每個渲染幀（50 Hz） |
| `/joint_states` | sensor_msgs/JointState | Isaac Sim | — | 50 Hz |
| `/imu` | sensor_msgs/Imu | Isaac Sim | `imu_link` | 50 Hz |
| `/scan` | sensor_msgs/LaserScan | Isaac Sim（RTX `Example_Rotary_2D`） | `laser_link` | 每圈一次 |
| `/camera/image_raw`、`/camera/camera_info` | sensor_msgs | Isaac Sim | `camera_optical_link` | 約 15 Hz |
| `/odom_gt` | nav_msgs/Odometry | Isaac Sim | `odom` → `base_link` | 50 Hz |
| `/odom` | nav_msgs/Odometry | odom_footprint | `odom` → `base_footprint` | 50 Hz |
| `/scan_filtered` | sensor_msgs/LaserScan | scan_ground_filter | `laser_link` | 同 `/scan` |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 collision_monitor / teleop | — | Nav2 20 Hz |

### 3.2 TF 樹

`map →(slam_toolbox 或 amcl) odom →(odom_footprint) base_footprint →(odom_footprint) base_link →(robot_state_publisher) 腿、imu_link、laser_link、camera_link、camera_optical_link`

- `base_footprint`：貼地、永遠水平，只含 x、y、yaw。`base_footprint→base_link` 帶高度、roll、pitch。
- **Isaac Sim 不發佈任何 TF。** 每段 TF 都只有一個發佈者。
- URDF 不加 `base_footprint`，它由 `odom_footprint` 動態發佈。

### 3.3 套件分工

| 套件 | 位置 | 角色 |
|---|---|---|
| `robot_dog_description` | `~/dog_12dof_ws/robot_dog_description`（已存在） | URDF、網格、感測器規格、Isaac Sim 執行端、策略檔 |
| `robot_dog_lab` | `robot_dog_description/isaaclab/robot_dog_lab/`，由 `isaaclab/scripts/*.py` 直接 import，在 `~/IsaacLab` 的 uv 環境執行（和舊 8 自由度狗相同，不另建 Isaac Lab 外部專案，以免再裝一份 Isaac Sim） | Isaac Lab 訓練任務、評估、匯出 |
| `robot_dog_navigation` | `~/dog_12dof_ws/robot_dog_navigation`（**新增**，ament_python） | launch、Nav2／slam_toolbox／AMCL 參數、地圖、RViz、兩個 ROS 節點、`nav_eval.py` |

## 4. RL 步態訓練（`robot_dog_lab`）

### 4.1 先讓現有任務跑起來

1. 新增 `isaaclab/scripts/train.py`（註冊任務後呼叫 `isaaclab_rl.entrypoints.run_train_cli`），
   刪除 `setup_robot_dog_lab.sh`。
2. smoke test：`--num_envs 64 --max_iterations 5`，確認 URDF→USD 轉換成功、`.*_calf` 接觸感測器能解析。
3. 站立測試：零動作 5 秒，16 隻的機身高度都在 0.20～0.28 m（MuJoCo 測過 0.226 m，kp 20 承重下會下沉），沒有一隻倒下。

### 4.2 任務內容（`RobotDog-Velocity-Flat`）

- **觀測（48 維，Isaac Lab 預設順序）**：`base_lin_vel`(3, 真值)、`base_ang_vel`(3)、`projected_gravity`(3)、
  `velocity_commands`(3: vx, vy, wz)、`joint_pos_rel`(12)、`joint_vel_rel`(12)、`last_action`(12)。
  訓練時有 Isaac Lab 預設的觀測雜訊，部署時不加。
- **動作**：12 維，`關節目標 = 預設站姿 + 0.25 × action`。站姿：hip 0、thigh 0.8、calf −1.5 rad。
- **致動器**：`DCMotorCfg`，effort 25 N·m、saturation 25 N·m、速度 30 rad/s、kp 20、kd 0.5，**armature 0.01 kg·m²**。
  armature 是減速馬達轉子的等效慣量；沒有它時，這個逐步明確計算的 PD 在輕的小腿（I ≈ 1.3e-3 kg·m²）、dt 0.005 s 下
  會數值發散（固定機身、零動作 2 秒內關節速度到 1e11 rad/s），腳一離地就發散。
- **物理**：sim dt 0.005 s、decimation 4（策略 50 Hz），PhysX，solver 4/0，關閉自碰撞，地面摩擦 1.0。
- **要改的四處：**
  1. 速度追蹤獎勵的 `std` 設定為 **0.25**（Isaac Lab 預設 0.5），讓低速也有動力追準：std 0.5 時忽略 0.1 m/s 指令只少 4% 獎勵，
     run v4 對 ±0.1 指令完全站著不動。從頭訓練用 0.25 不易學會走，所以分兩階段：
     **第 1 階段**從頭訓練，命令列覆寫 std=0.5；**第 2 階段**從第 1 階段的 checkpoint 用預設 0.25 再訓練 1000 iterations。
     `feet_air_time` 的 `threshold` 從 0.5 s 改成 **0.2 s**：這隻狗每步的騰空時間短於 0.5 s，預設值會讓每一步都扣分（run v1–v2）。
     `feet_air_time` 的指令門檻也改掉：Isaac Lab 只在 `|cmd_xy| > 0.1` 時給抬腳獎勵，0.1 m/s 和原地轉向的指令因此
     學成站著不動（run v4–v5）。改用 `robot_dog_lab` 的 `feet_air_time_any_motion`：`|cmd_xy| > 0.05` **或** `|wz| > 0.05` 就給。
     指令取樣改用 `RobotDogVelocityCommand`：Isaac Lab 的均勻取樣之外，**25%** 重新取樣的環境改給單一軸的小指令
     （vx 或 vy 0.05–0.2 m/s，或 wz 0.1–0.3 rad/s，正負隨機，其他兩軸 0）。均勻取樣幾乎不會出現 (0.1, 0, 0)，
     vy = wz = 0 的指令只有站立環境的 0，策略因此把小指令當成站立（run v4–v6）。
  2. `commands.base_velocity.rel_standing_envs` 從 0.02 調成 **0.1**，讓策略學會零指令時站穩不踏步。
  3. 指令範圍維持 vx ±0.8、vy ±0.5、wz ±1.0。Nav2 的上限在範圍之內（第 6 節）。
  4. `commands.base_velocity.heading_command` 從 True 改成 **False**：wz 直接均勻取樣，和 Nav2 直接送 wz 的用法相同
     （Isaac Lab 預設是由航向誤差算出 wz）。
  5. 新增 `base_height_l2`（目標 **0.23 m**，權重 **−50**）：沒有高度獎勵時，策略學成壓低機身、腹部離地 4–5 cm 趴著爬
     （run v7，機身約 0.09 m；計畫 2 一致性測試時發現）。v8 = v7 加這一項接著訓練，站高約 0.226 m。
- 領域隨機化保留 Isaac Lab 預設（摩擦、機身質量、推力、外力）。
- PPO：現有 `rsl_rl_ppo_cfg.py`（128×3 MLP），4096 個環境、1000 iterations。

### 4.3 評估 `gait_eval.py`（Isaac Lab，單一環境、關閉推力與雜訊）

每個指令先暖身 2 秒，再量 5 秒平均速度。誤差除了小於表中門檻，**也要小於指令大小的一半**
（否則站著不動也能通過 ±0.1 的指令，因為那時誤差剛好是 0.1）。另外每個情境的**平均機身高度要 ≥ 0.18 m**（擋掉趴著爬）：

| 項目 | 指令 | 通過 |
|---|---|---|
| 前後 | vx ∈ {±0.1, ±0.3, ±0.5} | \|誤差\| < 0.1 m/s |
| 側移 | vy ∈ {±0.1, ±0.3} | \|誤差\| < 0.1 m/s |
| 轉向 | wz ∈ {±0.2, ±0.5, ±0.8} | \|誤差\| < 0.15 rad/s |
| 站立 | 0 指令 10 秒 | 水平漂移 < 0.1 m |
| 全程 | — | 沒有跌倒 |

### 4.4 匯出 `export_policy.py`

從 Isaac Lab 訓練紀錄（`logs/rsl_rl/<experiment>/<run>`）輸出到
`robot_dog_description/isaac/policies/<run>/`：

- `policy.pt`：TorchScript，含 RSL-RL 的觀測正規化（若有啟用）。輸入 48 維，輸出 12 維。
- `policy.json`：由實際的環境設定物件產生，不手寫：

```json
{
  "task": "RobotDog-Velocity-Flat",
  "run": "<run 資料夾名稱>",
  "checkpoint": "model_999.pt",
  "joint_names": ["<策略的關節順序，取自動作項的關節清單>"],
  "default_joint_pos": [0.0, 0.8, -1.5],
  "action_scale": 0.25,
  "clip_actions": null,
  "obs_terms": [{"name": "base_lin_vel", "dim": 3, "scale": null, "clip": null}],
  "sim_dt": 0.005,
  "decimation": 4,
  "actuator": {"type": "DCMotor", "stiffness": 20.0, "damping": 0.5, "armature": 0.01,
               "effort_limit": 25.0, "saturation_effort": 25.0, "velocity_limit": 30.0},
  "physx": {"solver_position_iterations": 4, "solver_velocity_iterations": 0, "self_collisions": false},
  "materials": {
    "ground": {"static_friction": 1.0, "dynamic_friction": 1.0, "restitution": 0.0,
               "friction_combine_mode": "multiply", "restitution_combine_mode": "multiply"},
    "robot": {"static_friction": 0.8, "dynamic_friction": 0.6, "restitution": 0.0}
  },
  "command_ranges": {"lin_vel_x": [-0.8, 0.8], "lin_vel_y": [-0.5, 0.5], "ang_vel_z": [-1.0, 1.0]},
  "spawn_height": 0.32,
  "rigid_body": {"max_depenetration_velocity": 1.0, "linear_damping": 0.0, "angular_damping": 0.0,
                 "max_linear_velocity": 1000.0, "max_angular_velocity": 1000.0},
  "physx_scene": {"solver_type": 1, "bounce_threshold_velocity": 0.5, "friction_offset_threshold": 0.04,
                  "friction_correlation_distance": 0.025, "enable_stabilization": false}
}
```

（`default_joint_pos` 是長度 12、與 `joint_names` 同序的陣列，`obs_terms` 依序列出全部 7 項；上面只是格式示意。
`materials.robot` 是 Isaac Lab `physics_material` 事件給機器人所有連桿的摩擦，與地面以 multiply 組合。）
讀寫與檢查都在 `isaac/robot_dog_sim/policy_meta.py`（純 Python），匯出端和 Isaac Sim 端共用。

**關節順序依名稱對應**：Isaac Lab articulation 的順序不一定等於 ROS 的 FL/FR/RL/RR 順序（舊 8 自由度狗是
「所有大腿，再所有小腿」）。Isaac Sim 端一律用 `joint_names` 找索引，不假設順序。

## 5. Isaac Sim 執行端（`robot_dog_description/isaac/`）

### 5.1 檔案

```
isaac/
├── robot_dog_sim/
│   ├── __init__.py       空白（純 Python 模組要能用系統 python3 import）
│   ├── util.py           log、try_step、require_ros_jazzy（純 Python）
│   ├── robot_spec.py     站姿、馬達上限、感測器位置等常數（純 Python）
│   ├── kit.py            啟用擴充套件、開新 stage
│   ├── robot.py          匯入 URDF→USD、依名稱找連桿/關節、設定出生姿態與驅動模式
│   ├── sensors.py        IMU、RTX 2D LiDAR、RGB 相機（位置與 URDF 感測器 frame 相同）
│   ├── world.py          產生室內世界（或載入 --world 指定的 USD）
│   ├── ros_graph.py      OmniGraph 發佈器：clock、joint_states、imu、scan、camera、odom_gt、
│   │                     以及 ros2_control 模式用的 /isaac_joint_states、/isaac_joint_commands
│   ├── policy_meta.py    policy.json 讀寫與檢查（純 Python，匯出端共用）
│   ├── actuator.py       DC 馬達力矩公式（純 torch/numpy，可單元測試）
│   ├── observations.py   依 policy.json 組觀測（純函式，可單元測試）
│   └── policy_runner.py  載入策略、physics callback、站立保持、cmd_vel 逾時、跌倒偵測
├── robot_dog_isaac.py    現有 ros2_control 模式，改用上面的模組，指令列參數與行為不變
├── robot_dog_nav_sim.py  導航模式入口
├── policy_parity.py      M2 一致性測試（不接 ROS）
├── tools/rclpy_probe.py  M0：確認 rclpy 與 OmniGraph ROS 2 橋接能在同一程序共存
├── tests/                pytest
└── policies/<run>/       policy.pt、policy.json
```

執行方式（在 `~/IsaacLab` 的 uv 環境，先 source Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/IsaacLab && uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/robot_dog_nav_sim.py \
    --policy ~/dog_12dof_ws/robot_dog_description/isaac/policies/<run> [--headless] [--no-camera] [--hold] [--world x.usd]
```

`--policy` 預設為 `policies/` 下名稱排序最後的資料夾。

### 5.2 物理與控制迴圈

- 物理 dt、decimation、solver iterations、自碰撞、地面材質都**從 `policy.json` 讀取**。
- 匯入時關節驅動 stiffness/damping 設為 0，改為力矩控制；每個關節的 armature 設成 `actuator.armature`
  （PhysX joint armature，和 Isaac Lab 相同），否則抬腳時會數值發散。
- **physics step callback**（200 Hz）：
  - 每 `decimation` 步：讀 articulation 狀態 → `observations.py` 組 48 維觀測 → `policy.pt` 推論 →
    `q_target = default + action_scale × action`，記錄 `last_action`。
  - 每一步：`actuator.py` 依 Isaac Lab `DCMotor` 公式算力矩
    （`τ = kp(q_target − q) − kd·q̇`；`v = clip(q̇, ±vlim·(1 + effort_limit/sat))`；
    `τmax = min(sat·(1 − v/vlim), effort_limit)`、`τmin = max(sat·(−1 − v/vlim), −effort_limit)`；
    `τ` 截斷在 `[τmin, τmax]`。`sat` = saturation_effort、`vlim` = velocity_limit，與 Isaac Lab
    `DCMotor._clip_effort` 相同）→ 施加到關節。
- 觀測中的 `base_lin_vel`、`base_ang_vel` 用 articulation 根部真值轉到機身座標系，和 Isaac Lab 一樣。
- 渲染 50 Hz（每個策略步一幀）。模擬慢於即時只會讓 sim time 變慢，步態不受影響。

### 5.3 指令與保護

- `/cmd_vel`：rclpy 節點在腳本內訂閱，每個渲染幀 `spin_once(timeout_sec=0)`。策略步讀最新值與收到時間（sim time）。
  - 夾在 `command_ranges` 之內。
  - **超過 0.5 s 沒收到 → 指令視為 0。**
- 站立保持（**預設關**，`--hold` 開啟）：指令的 \|vx\|、\|vy\| < 0.02 且 \|wz\| < 0.05 持續 0.3 s，且機身已幾乎靜止
  （< 0.05 m/s、< 0.1 rad/s）→ `q_target = default`，不跑策略；指令恢復時 `last_action` 歸零。
  預設關閉的原因（計畫 2 實測）：剛停下時腳還沒回到髖下方，把關節固定在標準站姿會讓狗踉蹌
  （停止 1 s 後仍有 0.17 m/s、0.87 rad/s）；不保持時 v8 自己 1 s 內停穩（≤ 0.032 m/s），零指令 10 s 漂移 0.000 m。
- 跌倒偵測：機身高度 < 0.12 m 或傾斜 > 60° → 印出錯誤（含 sim time），每次跌倒只印一次。

### 5.4 感測器

| 感測器 | 掛載 | 設定 |
|---|---|---|
| IMU | `base_link`，(0, 0, 0.048) | 50 Hz 發佈 |
| LiDAR | `base_link`，(0.080, 0, 0.085) | RTX `Example_Rotary_2D`，最遠距離以實際設定為準，寫進 slam_toolbox/costmap 參數 |
| 相機 | `base_link`，(0.178, 0, 0.0575) | RGB 640×480，水平 FOV 1.204 rad，約 15 Hz；`--no-camera` 不建立 |

導航模式下 **LiDAR 建立失敗 → 程式結束**；IMU 或相機失敗只警告。

### 5.5 世界（`world.py`）

- 10 × 10 m 房間，四面外牆高 1.0 m、厚 0.1 m。
- 兩道內牆把房間分成三個區域，留 1.0 m 寬的通道。
- 4 個方塊（0.4～0.6 m）、3 個圓柱（直徑 0.3 m），高度都高於 LiDAR 掃描平面（約 0.34 m）。
- 地面與機器人的摩擦材質取自 `policy.json` 的 `materials`。出生點在原點，四周 1 m 內淨空。
- 整個場景用程式產生，不依賴網路或 Nucleus 素材，每次都相同。`--world` 可以改載其他 USD。

## 6. Nav2 與 ROS 端（`robot_dog_navigation`）

### 6.1 檔案

```
robot_dog_navigation/
├── package.xml, setup.py, setup.cfg, resource/
├── robot_dog_navigation/
│   ├── odom_footprint_node.py
│   ├── scan_ground_filter_node.py
│   ├── footprint_math.py       姿態分解（純函式）
│   └── ground_filter_math.py   打地距離幾何（純函式）
├── launch/nav.launch.py
├── config/nav2_params.yaml     Nav2 + slam_toolbox + amcl
├── maps/                       map_saver_cli 存的地圖
├── rviz/nav.rviz
├── scripts/nav_eval.py
└── test/                       pytest
```

### 6.2 節點

- **odom_footprint**：訂閱 `/odom_gt`。發佈 `/odom`（frame `odom`、child `base_footprint`，只含 x、y、yaw 與平面速度），
  TF `odom→base_footprint`（x、y、yaw），以及 TF `base_footprint→base_link`（z、roll、pitch）。
  移植自 `~/robotdog_ws/dog_gait/dog_gait/odom_footprint_node.py`。
- **scan_ground_filter**：訂閱 `/scan`。用掃描時間戳查 TF `odom→laser_link`（`odom` 是水平的，含機身傾斜），
  把每個回波轉到 `odom` 座標，**高度 < 0.05 m 的視為打到地面**，設為 `inf`，發佈 `/scan_filtered`。
  （原本寫的「打地距離 − 0.05 m」改成這個舊狗驗證過的作法：牆和障礙物不論機身怎麼傾斜都會保留。）
  姿態與高度只有 TF 這一個來源，不另外訂閱 `/imu`。移植自 `~/robotdog_ws/dog_gait/dog_gait/scan_ground_filter_node.py`。

### 6.3 定位與建圖

- `slam:=True`：slam_toolbox online async，mapping 模式，`base_frame: base_footprint`、`odom_frame: odom`、
  `scan_topic: /scan_filtered`，解析度 0.05 m。
- `slam:=False`：map_server（`map:=` 參數，預設 `maps/robot_dog_room.yaml`）+ amcl，
  `robot_model_type: nav2_amcl::OmniMotionModel`，初始位置設在原點，`scan_topic: /scan_filtered`。

### 6.4 Nav2

用 `nav2_bringup` 的 `navigation_launch.py`，參數檔從 Jazzy 預設複製後修改：

| 項目 | 設定 |
|---|---|
| planner | NavFn |
| controller | MPPI，`motion_model: Omni`，vx −0.3～0.5、vy ±0.3、wz ±0.8，20 Hz |
| velocity_smoother | 同上限，加速度 vx/vy 1.0 m/s²、wz 2.0 rad/s²，`deadband_velocity` 全 0 |
| footprint | 多邊形 `[[0.22, 0.16], [0.22, -0.16], [-0.22, -0.16], [-0.22, 0.16]]` |
| inflation | 0.35 m |
| costmap | obstacle layer 只用 `/scan_filtered` |
| collision_monitor | 保留預設停止區，來源改成 `/scan_filtered` |
| robot_base_frame | `base_footprint` |
| use_sim_time | true（所有節點） |
| transform_tolerance | 0.5 s |

### 6.5 Launch

```bash
ros2 launch robot_dog_navigation nav.launch.py slam:=True rviz:=true
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `slam` | `True` | SLAM 或地圖模式 |
| `map` | `maps/robot_dog_room.yaml` | 地圖模式用 |
| `params_file` | `config/nav2_params.yaml` | |
| `rviz` | `true` | |

啟動：robot_state_publisher（`robot_dog.urdf.xacro sim_mode:=none`）、odom_footprint、scan_ground_filter、
slam_toolbox 或 map_server + amcl、Nav2、RViz。Isaac Sim 另外在終端 1 啟動（第 5.1 節），因為它在 Isaac Lab 的 uv 環境執行。

## 7. 里程碑與驗收

前一個里程碑通過才進下一個。

| # | 內容 | 通過條件 |
|---|---|---|
| M0 | 環境確認 | 安裝 `ros-jazzy-joint-state-topic-hardware-interface`；`colcon build` `robot_dog_description` 成功（`robot_dog_navigation` 在 M3 才建）；Isaac Sim 程序內的 rclpy 能收到同程序 OmniGraph 發佈的 `/clock`；重構後的 `robot_dog_isaac.py` 列出各感測器 ok，`isaac.launch.py` + `stand_up.py` 能讓狗站起來 |
| M1 | RL 訓練 | 4.1 三步通過；正式訓練完成；`gait_eval.py` 全部通過（4.3）；匯出 `policy.pt` + `policy.json` |
| M2 | Isaac Sim 跑策略 | `policy_parity.py`：同一段 60 秒指令序列（前進、側移、轉向、停止各段）分別在 Isaac Lab（單一環境、無雜訊無推力）與 Isaac Sim（無 ROS、無感測器）執行，**每段平均機身速度差 < 0.05 m/s、角速度差 < 0.1 rad/s**，兩邊都沒跌倒 |
| M3 | ROS 2 橋接 | 3.1 表內話題頻率正確（`ros2 topic hz`）；`view_frames` 的 TF 樹與 3.2 相同；`teleop_twist_keyboard` 能讓狗前進、側移、轉彎；停送 `cmd_vel` 後 1 s 內停穩（0.5 s 逾時 + 0.5 s 煞停） |
| M4 | SLAM | teleop 繞房間一圈，地圖閉合、牆面沒有重影；`map_saver_cli` 存成 `maps/robot_dog_room` |
| M5 | Nav2 導航 | `nav_eval.py` 用 `nav2_simple_commander` 依序送 5 個固定目標（三個區域都有，至少一個要穿過內牆通道），SLAM 與 AMCL 兩模式**全部到達**（0.25 m、0.3 rad 內），單一目標 120 s（sim time）內完成，全程沒有跌倒訊息 |

### 實作分期

整份設計共用 `policy.json` 和話題介面，所以寫成一份 spec。實作計畫則分三份，依序撰寫與執行：

1. **計畫 1：M0 + M1**：環境確認、Isaac Sim 模組重構、RL 訓練、評估、匯出。
2. **計畫 2：M2 + M3**：`policy_runner`、一致性測試、導航模式的 ROS 2 橋接、`odom_footprint`、`scan_ground_filter`。
   計畫 1 留給計畫 2 的事項（2026-11-01 計畫 1 結案時記錄）：
   - `policy.json` 還沒記錄的物理設定，M2 前要從訓練紀錄的 `params/env.yaml` 補進去：rigid body 的
     `max_depenetration_velocity` 1.0、線/角阻尼 0、最大線/角速度 1000；PhysX scene 的 `solver_type` 1（TGS）、
     `bounce_threshold_velocity` 0.5、`friction_offset_threshold` 0.04、`friction_correlation_distance` 0.025、
     `enable_stabilization` false。
   - 訓練時 Isaac Lab 3.0 預設 `sim.use_newton_actuators: true`，DC 馬達是由 Newton 的 `ClampingDCMotor` 計算的；
     公式和 `isaaclab.actuators.DCMotor` 相同，第 8 節的單元測試比對對象不變，但一致性除錯時要知道這點。
   - `observations.py` 要檢查 `obs_terms` 的名稱與順序。匯出目前只檢查關節集合；要另外確認 `joint_names` 等於
     articulation 順序，否則 `joint_pos_rel` / `joint_vel_rel` 的順序可能和 `joint_names` 不同。
   - `robot_dog_isaac.py --effort` 模式沒有 armature；若用外部 PD 驅動，可能和訓練初期一樣數值發散。
3. **計畫 3：M4 + M5**：slam_toolbox、AMCL、Nav2 參數、RViz、`nav_eval`。

## 8. 測試

**單元測試（pytest，不開模擬器）**：
- `robot_dog_navigation/test/`：`footprint_math`（各種 roll/pitch/yaw 組合分解後再組回，與原四元數一致）、
  `ground_filter_math`（水平時沒有點被濾掉；已知 pitch 下打地距離與手算值一致）。
- `robot_dog_description/isaac/tests/`（在 Isaac Lab uv 環境執行）：`actuator.py` 與
  `isaaclab.actuators.DCMotor` 在隨機輸入下的力矩一致（誤差 < 1e-5）；`observations.py` 用合成狀態檢查順序、
  維度、縮放與 `policy.json` 一致。

**整合測試**：M2 的 `policy_parity.py`、M5 的 `nav_eval.py`、M1 的 `gait_eval.py`。

## 9. 錯誤處理

| 狀況 | 處理 |
|---|---|
| `policy.json` 的關節名稱在 articulation 找不到 | 拒絕啟動，印出兩邊的名稱清單 |
| `policy.json` 缺欄位或 `obs_terms` 總維度 ≠ 策略輸入 | 拒絕啟動，指出缺哪個欄位或兩個維度 |
| 導航模式 LiDAR 建立失敗 | 結束程式 |
| IMU／相機建立失敗 | 警告，繼續執行 |
| `/cmd_vel` 逾時 0.5 s | 指令歸零，策略自己停下站穩（`--hold` 時再固定站姿） |
| 跌倒 | 印錯誤一次；`nav_eval` 判定失敗 |
| 模擬慢於即時 | 全部 sim time；`transform_tolerance` 0.5 s |

## 10. 風險

| 風險 | 最早暴露 | 對策 |
|---|---|---|
| Isaac Sim 6.2 API 細節（URDF importer、RTX LiDAR 設定名稱、articulation 讀寫、physics callback） | M0、M2 | 以已安裝的 `isaacsim 6.2.0` 原始碼與 `isaacsim.robot.policy.examples` 為準，不憑記憶 |
| rclpy 和 Isaac Sim 內建 ROS 2 函式庫在同一個程序中衝突 | M0 | 備案：`/cmd_vel` 改用 OmniGraph `ROS2SubscribeTwist`，逾時改成「指令值 0.5 s 沒變化」 |
| 低速追蹤不夠準 | M1 | 回 M1 調獎勵或指令取樣，**不加**查表補丁 |
| Isaac Lab 與 Isaac Sim 的物理設定有落差 | M2 | 設定全部來自 `policy.json`；M2 不過就逐項比對，再進 M3 |
| 筆電同時跑 RTX LiDAR、相機、Nav2 太慢 | M3 | `--no-camera`、`--headless`；只影響速度，不影響正確性 |
| Isaac Lab 外部專案需要另一個 uv 環境（再裝一份 Isaac Sim） | 已處理 | 改用 `isaaclab/scripts/*.py` 直接在 `~/IsaacLab` 環境執行（4.1） |

## 11. 文件

更新三份中文 README：
- `robot_dog_description/README.md`：Isaac Sim 兩種模式、模組結構、策略檔。
- `robot_dog_description/isaaclab/README.md`：訓練、評估、匯出。
- `robot_dog_navigation/README.md`：啟動導航、存地圖、`nav_eval`、launch 參數、話題與 TF。
