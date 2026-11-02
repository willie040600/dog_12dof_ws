# robot_dog_navigation

robot_dog 在 Isaac Sim 導航模式下的 ROS 2 端。目前（計畫 2）提供 TF 與雷射地面過濾；Nav2、slam_toolbox、AMCL 在計畫 3 加入。

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation
source install/setup.bash
# 先啟動 Isaac Sim：robot_dog_description/isaac/robot_dog_nav_sim.py（見 robot_dog_description/README.md）
ros2 launch robot_dog_navigation bridge.launch.py
```

`bridge.launch.py` 啟動（全部 `use_sim_time`）：

| 節點 | 輸入 | 輸出 |
|---|---|---|
| robot_state_publisher | `/joint_states`、`robot_dog.urdf.xacro sim_mode:=none` | 腿與感測器 TF |
| `odom_footprint_node` | `/odom_gt`（Isaac Sim 真值，`odom→base_link`） | `/odom`（`odom→base_footprint`，平面）、TF `odom→base_footprint`、`base_footprint→base_link`（高度、roll、pitch） |
| `scan_ground_filter_node` | `/scan` + TF `odom→laser_link` | `/scan_filtered`：在 `odom` 中高度 < 0.05 m 的回波設為 `inf` |

TF 樹：`odom → base_footprint → base_link → {FL/FR/RL/RR 的 hip、thigh、calf，imu_link，laser_link，camera_link → camera_optical_link}`

- `base_footprint` 貼地、永遠水平；身體的起伏與傾斜都在 `base_footprint → base_link`，SLAM 與 Nav2 都以 `base_footprint` 為機器人座標。
- 走路時機身前傾幾度，2D LiDAR 就會在幾公尺外打到地板；`scan_ground_filter` 把這些點去掉，牆和障礙物不論怎麼傾斜都保留。

測試（純 Python）：`cd robot_dog_navigation && python3 -m pytest test -q`
