# robot_dog 強化學習訓練（Isaac Lab 3.0 + Isaac Sim 6.2，Ubuntu）

這個資料夾就是訓練機器狗需要的全部檔案，不需要 ROS 2 或 Fusion。

```
isaaclab/
├── scripts/
│   ├── train.py                             # 註冊任務後呼叫 Isaac Lab 的 train CLI
│   └── play.py                              # 回放
├── robot_dog_lab/
│   ├── assets/robot_dog.py                  # ROBOT_DOG_CFG（URDF、馬達模型、站姿）
│   ├── data/robot_dog/{urdf,meshes}/        # robot_dog_rl.urdf + STL
│   └── tasks/locomotion/velocity/           # 任務 RobotDog-Velocity-Flat + RSL-RL PPO 設定
└── tests/                                   # pytest
```

## 前提

Isaac Lab 3.0 原始碼在 `~/IsaacLab`，已用 `uv` 建好環境（內含 Isaac Sim 6.2 pip 版），RTX 顯卡驅動。
所有指令都在 `~/IsaacLab` 底下、用它的 uv 環境執行，**不需要**另外建立 Isaac Lab 外部專案。

```bash
cd ~/IsaacLab
export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
S=~/dog_12dof_ws/robot_dog_description/isaaclab/scripts
```

Isaac Lab 3.0 沒有 `--headless`：預設就不開畫面，要看畫面才加 `--viz kit`。
`train.py`、`play.py` 會自動補上 `--rl_library rsl_rl`。

## 訓練

```bash
# 先用少量環境試跑 5 個 iteration，確認 URDF 轉換、接觸感測器都正常（第一次會先把 URDF 轉成 USD）
uv run --extra isaacsim python $S/train.py --task RobotDog-Velocity-Flat --num_envs 64 --max_iterations 5 --run_name smoke
# 正式訓練（預設 4096 個環境、1000 iterations）
uv run --extra isaacsim python $S/train.py --task RobotDog-Velocity-Flat --run_name v1
# 從上一版接著訓練 1000 iterations
uv run --extra isaacsim python $S/train.py --task RobotDog-Velocity-Flat --checkpoint <run 資料夾>/model_<N>.pt --max_iterations 1000 --run_name v2
```

紀錄與 checkpoint 在 `~/IsaacLab/logs/rsl_rl/robot_dog_flat/<時間>_<run_name>/`。
顯示卡記憶體不夠時降低 `--num_envs`（例如 2048）。

## 回放

```bash
uv run --extra isaacsim python $S/play.py --task RobotDog-Velocity-Flat --checkpoint <run 資料夾>/model_<N>.pt --viz kit
```

## 任務內容

- 觀測（48 維）：機身線速度、角速度、重力方向、速度指令 (vx, vy, wz)、12 個關節角度與速度、上一步動作。
- 動作：`關節目標 = 站姿 + 0.25 × action`。
- 為了配合 Nav2，和 Isaac Lab 預設的速度追蹤任務相比改了：
  - `heading_command=False`：wz 直接取樣（Nav2 直接送 wz）
  - 零指令環境 10%：學會站著不動
  - 速度追蹤獎勵 `std` 0.25：Nav2 常送 0.05～0.2 的小指令，低速也要追準
- 馬達（25 N·m、30 rad/s）與質量都是暫定值，換成實際規格後再訓練。

## 測試

```bash
uv run --extra isaacsim --with pytest python -m pytest ~/dog_12dof_ws/robot_dog_description/isaaclab/tests -q
```
