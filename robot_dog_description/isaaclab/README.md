# robot_dog 強化學習訓練（Isaac Lab 3.0 + Isaac Sim 6.2，Ubuntu）

這個資料夾就是訓練機器狗需要的全部檔案，不需要 ROS 2 或 Fusion。

```
isaaclab/
├── setup_robot_dog_lab.sh   # 一鍵建立 Isaac Lab 專案並裝入任務
└── robot_dog_lab/
    ├── assets/robot_dog.py                  # ROBOT_DOG_CFG（URDF、馬達模型、站姿）
    ├── data/robot_dog/{urdf,meshes}/        # robot_dog_rl.urdf + STL
    └── tasks/locomotion/velocity/           # 任務 RobotDog-Velocity-Flat + RSL-RL PPO 設定
```

## 步驟

前提：訓練機已安裝 Isaac Lab 3.0（原始碼在 `~/IsaacLab`）、`uv`、RTX 顯卡驅動。

```bash
unzip robot_dog_description.zip
cd robot_dog_description/isaaclab
bash setup_robot_dog_lab.sh ~/IsaacLab ~/projects
```

腳本會：用 Isaac Lab 範本在 `~/projects/robot_dog_lab` 建專案 → 複製本資料夾的檔案 →
確認 `RobotDog-Velocity-Flat` 有註冊。之後：

```bash
cd ~/projects/robot_dog_lab
# 先用少量環境試跑 5 個 iteration，確認 URDF 轉換、接觸感測器都正常
uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task RobotDog-Velocity-Flat --num_envs 64 --max_iterations 5
# 正式訓練（預設 4096 個環境、1000 iterations）
uv run --extra isaacsim isaaclab train --rl_library rsl_rl --task RobotDog-Velocity-Flat
# 回放最新的模型
uv run --extra isaacsim isaaclab play --rl_library rsl_rl --task RobotDog-Velocity-Flat --checkpoint latest --viz kit
```

- 顯示卡記憶體不夠時降低 `--num_envs`（例如 2048）。
- 改用 Newton 物理：在指令最後加 `physics=newton_mjwarp`。
- 更新機器狗設計後重新執行 `setup_robot_dog_lab.sh`，只會覆蓋 robot_dog 的檔案。

## 訓練出來的策略

- 觀測：機身線速度、角速度、重力方向、速度指令、12 個關節角度與速度、上一步動作。
- 動作：`關節目標 = 站姿 + 0.25 × action`，關節順序與 ROS 2 控制器相同
  （FL、FR、RL、RR 各 hip / thigh / calf），所以可以接到 ROS 2 的 `joint_group_position_controller`。
- 馬達（25 N·m、30 rad/s）與質量都是暫定值，換成實際規格後再訓練，sim2real 會比較可靠。
