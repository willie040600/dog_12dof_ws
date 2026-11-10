# robot_dog_navigation

robot_dog 在 Isaac Sim 導航模式下的 ROS 2 端：TF、雷射地面過濾、slam_toolbox / AMCL、Nav2、RViz，以及評估腳本 `nav_eval`。

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation
source install/setup.bash
```

## 導航

```bash
# 終端 1：Isaac Sim（MUJI 公寓；見 robot_dog_description/README.md「Isaac Sim 6.2 導航模式」；狗在客廳中央的原點、面向 +x 出發）
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/robot_dog_nav_sim.py
# 終端 2：bridge + Nav2（一條指令；加 rviz:=true 會一起開 RViz）
ros2 launch robot_dog_navigation nav.launch.py                 # SLAM：邊建圖邊導航
ros2 launch robot_dog_navigation nav.launch.py slam:=False     # AMCL：用 maps/apartment.yaml 定位
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `slam` | `True` | `True`：slam_toolbox；`False`：map_server + AMCL |
| `map` | `maps/apartment.yaml` | AMCL 模式的地圖 |
| `params_file` | `config/nav2_params.yaml` | |
| `rviz` | `false` | `true`：一起開 RViz |

用 `rviz:=true` 開 RViz 後，按工具列的 **Nav2 Goal**，在地圖上點一下並拖出方向，狗就會走過去。
AMCL 的初始位置設在原點；若 Isaac Sim 不是剛啟動（狗不在原點），先用 **2D Pose Estimate** 指定位置。
浴室（東南角）門口只有 0.8 m、裡面最寬 0.40 m，和膨脹半徑差不多，Nav2 多半規劃不進去；淋浴盤（5 cm）比 LiDAR 平面低，
地圖上沒有，硬送進去可能被絆到。

存圖（SLAM 模式繞完公寓後）與檢查：
```bash
ros2 run nav2_map_server map_saver_cli -f robot_dog_navigation/maps/apartment --ros-args -p save_map_timeout:=10.0
ros2 run robot_dog_navigation check_map robot_dog_navigation/maps/apartment.yaml
```
（不要加 `use_sim_time`；預設只等 `/map` 2 s，模擬慢時會存檔失敗，所以加長到 10 s。）

`check_map` 和 `maps/truth/` 的真值平面圖比對（由公寓的 mesh 切出來，見 robot_dog_description/README.md）：
- 對齊率：地圖的佔據格中，離 LiDAR 平面真值 0.10 m 內的比例，≥ 0.95；
- 覆蓋率：「狗走得到的空間看得見的真值表面」中，0.15 m 內有地圖佔據格的比例，≥ 0.80（牆的背面、家具內部不算）。

評估（Isaac Sim 要重新啟動，讓狗回到原點）：
```bash
ros2 run robot_dog_navigation nav_eval --slam            # 配 slam:=True
ros2 run robot_dog_navigation nav_eval                   # 配 slam:=False
ros2 run robot_dog_navigation nav_eval --extra-goal=0.4,2.7,0.0   # 多送一個到不了的目標：主臥床的中央（不計分）
```
依序送 `nav_goals.py` 的 5 個目標：客廳西側 → 主臥 → 次臥 → 餐廚東側 → 回到起點（兩個臥室門各穿過兩次）；
每個目標在身體高度範圍的真值上淨空 ≥ 0.5 m。每個目標用 Isaac Sim 的真值 `/odom_gt`
判定：0.25 m、0.3 rad 內、120 s（sim time）內、沒有跌倒。每個目標印一行 `[NAV]`（含當下的定位誤差與 sideways），
最後 `[NAV] PASS k/5`；5 個都通過才 exit 0。AMCL 模式會先檢查初始定位誤差（> 0.1 m 就重設一次初始位置）。

### 結果（公寓，v8 策略，RTX 5070 Ti 筆電，headless、`--no-camera`，即時率 0.61）

| 模式 | 結果 | 終點距離（m） | 定位誤差（m） | 用時（s） | sideways |
|---|---|---|---|---|---|
| SLAM | PASS 5/5 | 0.10 / 0.07 / 0.04 / 0.09 / 0.03 | 0.01 / 0.02 / 0.02 / 0.02 / 0.03 | 12.7 / 13.4 / 51.7 / 9.5 / 14.8 | 6% |
| AMCL | PASS 5/5 | 0.03 / 0.06 / 0.08 / 0.09 / 0.04 | 0.07 / 0.06 / 0.08 / 0.02 / 0.03（初始 0.022） | 11.6 / 14.1 / 33.8 / 9.5 / 14.8 | 4% |

- 地圖 `maps/apartment`（四扇門的門板拿掉後重新建的）：2347 個佔據格，對齊率 0.997、覆蓋率 0.945
  （有門板時是 1878 格、0.998 / 0.825：開著的門板擋住了牆）。
- 到不了的目標（主臥床的中央）：Nav2 約 40 s 後回報 FAILED，判 FAIL，程式正常結束。Nav2 有時會在最近的可達點回報
  SUCCEEDED，所以 Nav2 回報 SUCCEEDED 之後還要用真值檢查距離與角度；Nav2 回報失敗、被拒絕或逾時都直接判 FAIL。
- **SLAM 模式的已知問題**：從主臥去次臥時，兩間臥室之間的牆被床擋住、LiDAR 還沒看到，Nav2 會規劃穿過那片未知區域
  往東走；狗走到東側門框前看到牆，路線翻轉成穿過門口，MPPI 在門框前來回擺，脫困要 10 s 到超過 120 s
  （5 輪中 2 輪逾時）。AMCL 模式用完整地圖，沒有這個問題。

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
主要改動：機器人座標 `base_footprint`、雷射 `/scan_filtered`；AMCL `OmniMotionModel`，`alpha1`～`alpha5` 0.05
（`/odom` 是模擬的真值；預設 0.2 時 AMCL 在主臥偏了 0.32～0.36 m）；MPPI `Omni`（vx −0.1～0.5、vy ±0.1、wz ±0.8，
`vy_std` 0.05）；goal checker 0.10 m / 0.25 rad；footprint 0.44 × 0.32 m；inflation 0.35 m、`cost_scaling_factor` 10
（MPPI 建議的陡峭衰減；公寓的門口寬 0.8 m，每一點離門框都在 0.40 m 內）。

**頭朝前走**：狗雖然能側移，導航時要轉身面向路徑、頭朝前走，不要斜著走。`nav_eval` 每個目標印出 `sideways`：
移動時間中，行進方向偏離機頭 20° 以上的比例（0.2 s 平均，步態的左右晃動不算；倒退也算）。
下表是在計畫 3 的方塊房間比較的（公寓裡目前的設定是 4～6%）：

| MPPI 設定 | sideways | 巡航（≥ 0.3 m/s）偏角中位數 | 卡住 |
|---|---|---|---|
| `Omni` vy ±0.3 + TwirlingCritic（舊） | 87% | — | 無 |
| `DiffDrive`（不能側移） | 1% | 0° | 6 次有 3 次：衝過目標停在旁邊 0.2–0.3 m，要側移才到得了，等約 40 s 後 Nav2 原地轉圈才脫困 |
| `Omni` vy ±0.1、`vy_std` 0.2 | 13% | 8° | 無 |
| **`Omni` vy ±0.1、`vy_std` 0.05（目前）** | **3%** | **0–1°** | **無** |

所以保留一點側移（0.1 m/s）給到達目標前的微調；拿掉 TwirlingCritic（它讓「斜走」比「轉身」便宜）；
`vy_std` 調小，否則取樣的 vy 常貼在上限，巡航時一路斜 8°。

測試（純 Python）：`cd robot_dog_navigation && python3 -m pytest test -q`
