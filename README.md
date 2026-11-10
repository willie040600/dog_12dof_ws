# dog_12dof_ws

12 自由度機器狗的模擬工作區：在 Isaac Sim 裡用訓練好的行走策略（v8）走路，再用 ROS 2 Nav2 在 MUJI 一層樓公寓裡導航。

目前只在模擬裡跑。馬達參數還是佔位值，不能直接推論實機表現，見 [robot_dog_description/README.md](robot_dog_description/README.md)「需要換成實際數值的地方」。

## 需要什麼

下載 repo 後不能直接啟動，要先裝好 ROS 2 和 Isaac Lab。場景、地圖、機器人模型和 v8 策略都在 repo 裡，不用另外下載。

| 項目 | 驗證過的環境 |
|---|---|
| 系統 | Ubuntu 24.04 |
| 顯卡 | NVIDIA RTX 5070 Ti 筆電版（12 GB），驅動 615.71.09 |
| ROS | ROS 2 Jazzy |
| Isaac Lab | commit `6bb654809`（含 Isaac Sim 6.2） |
| uv | 0.13.6 |
| 磁碟 | Isaac Lab 的 Python 環境約 27 GB |

Isaac Lab 請用上面這個 commit，較新的版本沒有測過。

以下指令假設兩個資料夾放在 `~/dog_12dof_ws` 和 `~/IsaacLab`，其他 README 也用這兩個路徑。

## 安裝

### 1. ROS 2 Jazzy

照[官方說明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)安裝 `ros-jazzy-desktop` 和 `ros-dev-tools`。

### 2. 這個專案

```bash
git clone https://github.com/willie040600/dog_12dof_ws.git ~/dog_12dof_ws
```

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths robot_dog_description robot_dog_navigation robot_dog_agent --ignore-src -y
```

`rosdep` 也會裝 Gazebo 模式要用的套件，導航用不到它們。

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation robot_dog_agent
```

### 3. Isaac Lab

先[安裝 uv](https://docs.astral.sh/uv/getting-started/installation/)，再取得指定版本：

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
```

```bash
git -C ~/IsaacLab checkout 6bb654809
```

第一次 `uv run --extra isaacsim` 會自動下載環境，需要一段時間。也可以先裝好：

```bash
cd ~/IsaacLab && uv sync --extra isaacsim --locked
```

下面的 `OMNI_KIT_ACCEPT_EULA=yes` 代表你接受 Isaac Sim 的授權條款（EULA），請先確認可以接受。

## 啟動導航

開兩個終端機。

終端 1，Isaac Sim：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/robot_dog_nav_sim.py
```

等機器狗出現後，在終端 2 啟動 Nav2 和 RViz：

```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 launch robot_dog_navigation nav.launch.py slam:=False
```

`slam:=False` 用 repo 裡的地圖定位；不加就是邊走邊建圖。在 RViz 按 **Nav2 Goal**，再點地圖上的目標。

## 用中文指揮機器狗

Nav2 啟動後，可以在第三個終端機用自然語言下指令（需要 Google AI Studio 的免費 Gemini API key）：

```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 run robot_dog_agent chat
```

```
你> 先去廚房，再回起點
```

申請 key、能說的話、地點表和限制見 [robot_dog_agent/README.md](robot_dog_agent/README.md)。

## 確認裝好了

Nav2 啟動後，開第三個終端機，讓機器狗自動走完五個目標：

```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 run robot_dog_navigation nav_eval
```

最後應該看到 `[NAV] PASS 5/5`。2026-11-07 從全新 clone 以 `--headless --no-camera`、`rviz:=false` 跑過，結果是 5/5。

不需要模擬器的快速測試：

```bash
cd ~/dog_12dof_ws/robot_dog_description/isaac && python3 -m pytest tests -q
```

```bash
cd ~/dog_12dof_ws/robot_dog_navigation && python3 -m pytest -q
```

```bash
cd ~/dog_12dof_ws/robot_dog_agent && python3 -m pytest test -q
```

## 常見問題

| 狀況 | 怎麼處理 |
|---|---|
| Isaac Sim 一啟動就結束 | 終端 1 要先 `source /opt/ros/jazzy/setup.bash` |
| 跑得比真實時間慢 | 正常，ROS 端都用模擬時間。可以加 `--headless --no-camera` 省資源，驗證環境的即時率約 0.61 |
| 沒有畫面的機器 | 模擬加 `--headless --no-camera`，Nav2 加 `rviz:=false` |
| 想用鍵盤遙控 | 另外裝 `ros-jazzy-teleop-twist-keyboard`，步驟見 [robot_dog_description/README.md](robot_dog_description/README.md) |

## 資料夾

| 路徑 | 內容 | 說明 |
|---|---|---|
| `robot_dog_description/` | 機器人模型、Isaac Sim 腳本、公寓場景、策略 | [README](robot_dog_description/README.md) |
| `robot_dog_description/isaaclab/` | 用 Isaac Lab 訓練行走策略 | [README](robot_dog_description/isaaclab/README.md) |
| `robot_dog_navigation/` | Nav2、SLAM / AMCL、地圖、評估腳本 | [README](robot_dog_navigation/README.md) |
| `robot_dog_agent/` | 用中文下指令的 Gemini agent | [README](robot_dog_agent/README.md) |
| `docs/superpowers/` | 設計規格與實作計畫 | |

## 授權

這個 repo 目前沒有授權條款。
