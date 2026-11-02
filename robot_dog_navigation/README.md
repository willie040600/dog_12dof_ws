# robot_dog_navigation

robot_dog 在 Isaac Sim 導航模式下的 ROS 2 端：TF、雷射地面過濾、slam_toolbox / AMCL、Nav2、RViz，以及評估腳本 `nav_eval`。

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation
source install/setup.bash
```

## 導航

```bash
# 終端 1：Isaac Sim（見 robot_dog_description/README.md「Isaac Sim 6.2 導航模式」；狗從原點、面向 +x 出發）
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/robot_dog_nav_sim.py
# 終端 2：bridge + Nav2 + RViz（一條指令）
ros2 launch robot_dog_navigation nav.launch.py                 # SLAM：邊建圖邊導航
ros2 launch robot_dog_navigation nav.launch.py slam:=False     # AMCL：用 maps/robot_dog_room.yaml 定位
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `slam` | `True` | `True`：slam_toolbox；`False`：map_server + AMCL |
| `map` | `maps/robot_dog_room.yaml` | AMCL 模式的地圖 |
| `params_file` | `config/nav2_params.yaml` | |
| `rviz` | `true` | |

在 RViz 按工具列的 **Nav2 Goal**，在地圖上點一下並拖出方向，狗就會走過去。
AMCL 的初始位置設在原點；若 Isaac Sim 不是剛啟動（狗不在原點），先用 **2D Pose Estimate** 指定位置。

存圖（SLAM 模式繞完房間後）與檢查：
```bash
ros2 run nav2_map_server map_saver_cli -f robot_dog_navigation/maps/robot_dog_room   # 不要加 use_sim_time（會逾時）
ros2 run robot_dog_navigation check_map robot_dog_navigation/maps/robot_dog_room.yaml
```
`check_map` 把佔據格子和 `robot_dog_sim/room_layout.py` 的牆與障礙物比對：對齊率（0.10 m 內）≥ 0.95、外牆覆蓋率 ≥ 0.80 才通過。

評估（Isaac Sim 要重新啟動，讓狗回到原點）：
```bash
ros2 run robot_dog_navigation nav_eval --slam            # 配 slam:=True
ros2 run robot_dog_navigation nav_eval                   # 配 slam:=False
ros2 run robot_dog_navigation nav_eval --extra-goal=-3.3,1.0,0.0   # 多送一個到不了的目標（不計分）
```
依序送 `nav_goals.py` 的 5 個目標（三個區域，兩個要穿過內牆門口）；每個目標用 Isaac Sim 的真值 `/odom_gt`
判定：0.25 m、0.3 rad 內、120 s（sim time）內、沒有跌倒。每個目標印一行 `[NAV]`（含當下的定位誤差），
最後 `[NAV] PASS k/5`；5 個都通過才 exit 0。AMCL 模式會先檢查初始定位誤差（> 0.1 m 就重設一次初始位置）。

### 結果（v8 策略，RTX 5070 Ti 筆電，headless、`--no-camera`）

| 模式 | 結果 | 終點距離（m） | 定位誤差（m） |
|---|---|---|---|
| SLAM | PASS 5/5 | 0.09 / 0.09 / 0.09 / 0.03 / 0.14 | 0.01 / 0.04 / 0.01 / 0.06 / 0.06 |
| AMCL | PASS 5/5 | 0.06 / 0.12 / 0.19 / 0.18 / 0.13 | 0.04 / 0.09 / 0.12 / 0.13 / 0.09（初始 0.025） |

- 地圖 `maps/robot_dog_room`：2726 個佔據格，對齊率 0.999、外牆覆蓋率 1.000。
- 到不了的目標（箱子中心 −3.3, 1.0）：120 s 逾時判 FAIL，程式正常結束。Nav2 有時會在最近的可達點回報 SUCCEEDED，
  所以 Nav2 回報 SUCCEEDED 之後還要用真值檢查距離與角度；Nav2 回報失敗、被拒絕或逾時都直接判 FAIL。

## 節點與參數

`bridge.launch.py`（`nav.launch.py` 會一起啟動，全部 `use_sim_time`）：

| 節點 | 輸入 | 輸出 |
|---|---|---|
| robot_state_publisher | `/joint_states`、`robot_dog.urdf.xacro sim_mode:=none` | 腿與感測器 TF |
| `odom_footprint_node` | `/odom_gt`（Isaac Sim 真值，`odom→base_link`） | `/odom`（`odom→base_footprint`，平面）、TF `odom→base_footprint`、`base_footprint→base_link`（高度、roll、pitch） |
| `scan_ground_filter_node` | `/scan` + TF `odom→laser_link` | `/scan_filtered`：在 `odom` 中高度 < 0.05 m 的回波設為 `inf` |

TF 樹：`map → odom → base_footprint → base_link → {FL/FR/RL/RR 的 hip、thigh、calf，imu_link，laser_link，camera_link → camera_optical_link}`

- `base_footprint` 貼地、永遠水平；身體的起伏與傾斜都在 `base_footprint → base_link`，SLAM 與 Nav2 都以 `base_footprint` 為機器人座標。
- 走路時機身前傾幾度，2D LiDAR 就會在幾公尺外打到地板；`scan_ground_filter` 把這些點去掉，牆和障礙物不論怎麼傾斜都保留。
- `map → odom` 由 slam_toolbox 或 AMCL 發佈；LiDAR 量程 0.15–15 m。

`config/nav2_params.yaml` 由 `config/make_nav2_params.py` 從 Jazzy 的預設參數產生，**改參數請改腳本再重新產生**：
```bash
python3 robot_dog_navigation/config/make_nav2_params.py
```
主要改動：機器人座標 `base_footprint`、雷射 `/scan_filtered`；AMCL `OmniMotionModel`；MPPI `Omni`（vx −0.3～0.5、
vy ±0.3、wz ±0.8，加 TwirlingCritic）；goal checker 0.10 m / 0.25 rad；footprint 0.44 × 0.32 m；inflation 0.35 m。

測試（純 Python）：`cd robot_dog_navigation && python3 -m pytest test -q`
