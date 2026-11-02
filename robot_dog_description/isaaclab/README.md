# robot_dog 強化學習訓練（Isaac Lab 3.0 + Isaac Sim 6.2，Ubuntu）

這個資料夾就是訓練機器狗需要的全部檔案，不需要 ROS 2 或 Fusion。

```
isaaclab/
├── scripts/
│   ├── train.py                             # 註冊任務後呼叫 Isaac Lab 的 train CLI
│   ├── play.py                              # 回放
│   ├── stand_test.py                        # 零動作站立測試
│   ├── export_policy.py                     # 匯出 policy.pt + policy.json
│   └── gait_eval.py                         # 依 spec 4.3 評估
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
# 第 1 階段：從頭學會走（追蹤獎勵 std 覆寫成 0.5；4096 個環境、1000 iterations，約 13 分鐘）
uv run --extra isaacsim python $S/train.py --task RobotDog-Velocity-Flat --run_name stage1 \
    env.rewards.track_lin_vel_xy_exp.params.std=0.5 env.rewards.track_ang_vel_z_exp.params.std=0.5
# 第 2 階段：從第 1 階段接著訓練 1000 iterations，用預設 std 0.25 把低速追準
uv run --extra isaacsim python $S/train.py --task RobotDog-Velocity-Flat --run_name stage2 \
    --checkpoint <第 1 階段的 run 資料夾>/model_999.pt --max_iterations 1000
```

為什麼分兩階段：std 0.25 時，誤差 0.4 m/s 的獎勵幾乎是 0，從頭訓練容易學成「站著不動」；
std 0.5 學得會走，但忽略 0.1 m/s 的指令只少 4% 獎勵，所以 ±0.1 的指令狗會直接站著。

紀錄與 checkpoint 在 `~/IsaacLab/logs/rsl_rl/robot_dog_flat/<時間>_<run_name>/`。
顯示卡記憶體不夠時降低 `--num_envs`（例如 2048）。

## 回放

```bash
uv run --extra isaacsim python $S/play.py --task RobotDog-Velocity-Flat --checkpoint <run 資料夾>/model_<N>.pt --viz kit
```

## 匯出（給 Isaac Sim 導航模式用）

```bash
uv run --extra isaacsim python $S/export_policy.py --run_dir ~/IsaacLab/logs/rsl_rl/robot_dog_flat/<run 資料夾>
```

- 輸出到 `robot_dog_description/isaac/policies/<run 資料夾>/`：`policy.pt`（TorchScript）+ `policy.json`
  （關節順序、站姿、觀測項目、物理、馬達、摩擦材質，全部從實際的環境設定讀出，格式見 spec 第 4.4 節）。
- 預設用編號最大的 `model_<N>.pt`；`--checkpoint model_500.pt` 指定其他的；`--out <資料夾>` 改輸出位置。
- 輸出資料夾已有檔案時會拒絕（避免覆蓋已提交的策略），要覆蓋加 `--force`。
- 會檢查：動作項剛好涵蓋 12 個關節各一次、TorchScript 與訓練時的推論輸出相同（誤差 < 1e-5）、`policy.json` 通過格式檢查。

## 評估

```bash
uv run --extra isaacsim python $S/gait_eval.py --policy_dir ~/dog_12dof_ws/robot_dog_description/isaac/policies/<run 資料夾>
```

17 個情境同時跑（每個一個環境、指令固定、沒有推力與雜訊）：暖身 2 秒後量 5 秒平均速度。全部通過時 exit 0。
誤差除了小於下表門檻，也要小於指令大小的一半（站著不動不能通過 ±0.1 的指令）。

| 項目 | 指令 | 通過 |
|---|---|---|
| 前後 | vx ∈ {±0.1, ±0.3, ±0.5} | \|誤差\| < 0.1 m/s |
| 側移 | vy ∈ {±0.1, ±0.3} | \|誤差\| < 0.1 m/s |
| 轉向 | wz ∈ {±0.2, ±0.5, ±0.8} | \|誤差\| < 0.15 rad/s |
| 站立 | 0 指令 10 秒 | 水平漂移 < 0.1 m |
| 全程 | — | 沒有跌倒 |

站立測試（零動作 5 秒，16 隻機身高度都在 0.20～0.28 m、沒有倒下）：

```bash
uv run --extra isaacsim python $S/stand_test.py
```

## 目前採用的策略

`robot_dog_description/isaac/policies/2026-11-02_12-15-26_v8/`（`model_3200.pt`），`gait_eval` **17/17**：
（評估用標準機器人：不隨機化機身質量與質心，seed 42；機身高度每個情境都在 0.222–0.231 m）

| 情境 | 指令 | 實際 |
|---|---|---|
| 前後 | ±0.1 / ±0.3 / ±0.5 m/s | +0.101 −0.085 / +0.307 −0.308 / +0.506 −0.510 |
| 側移 | ±0.1 / ±0.3 m/s | +0.101 −0.077 / +0.320 −0.301 |
| 轉向 | ±0.2 / ±0.5 / ±0.8 rad/s | +0.191 −0.114 / +0.499 −0.495 / +0.809 −0.804 |
| 站立 | 0，10 秒 | 漂移 0.009 m |
| 跌倒 | — | 0 |

最弱的是慢速右轉（−0.2 → −0.114，誤差 0.086，門檻 0.1）。Isaac Sim 一致性測試（`policy_parity.py`）10 段都在 0.02 m/s 內。

實際的訓練過程（每一版都從上一版接著訓練 1000 iterations，每次只改一件事）：

| 版本 | 改了什麼 | 結果 |
|---|---|---|
| v4 | 從頭訓練；armature 0.01、std 0.5、`feet_air_time` 門檻 0.2 s | 會走，但 ±0.1 m/s 站著不動 |
| v5 | std 0.25 | 0.3～0.5 更準，死區還在 |
| v6 | `feet_air_time_any_motion`（抬腳獎勵也看小指令與轉向） | 死區還在 |
| v7 | 25% 單一軸小指令（`RobotDogVelocityCommand`） | 速度 17/17，但**趴著爬**（機身約 0.09 m），加了高度判準後 0/17 |
| v8 | `base_height_l2`（目標 0.23 m，權重 −50），從 v7/`model_2450` 接著訓練 | 17/17，站高約 0.226 m；21 個 checkpoint 中 16 個 ≥ 15/17，取 `model_3200` |

用目前的設定從頭重現：照上面「訓練」一節的兩階段指令（第 1 階段 std 0.5）。這條路徑本身還沒有完整跑過一次，
重訓後請用 `gait_eval` 確認。挑同一個 run 裡最好的 checkpoint：

```bash
uv run --extra isaacsim python $S/rank_checkpoints.py --run_dir ~/IsaacLab/logs/rsl_rl/robot_dog_flat/<run>
uv run --extra isaacsim python $S/export_policy.py --run_dir <同上> --checkpoint <BEST 那一行的 model_N.pt>
```

換了策略之後，重做 Isaac Sim 一致性測試（Isaac Lab 參考 → Isaac Sim 比對）：

```bash
P=~/dog_12dof_ws/robot_dog_description/isaac/policies/<run>
uv run --extra isaacsim python $S/parity_reference.py --policy_dir $P --out $P/parity_reference.json
uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/policy_parity.py --policy $P --reference $P/parity_reference.json --headless
```

策略的關節順序（`policy.json` 的 `joint_names`）是 **4 個 hip → 4 個 thigh → 4 個 calf**（FL、FR、RL、RR），
不是 ROS 控制器的「每條腿 hip/thigh/calf」順序，所以 Isaac Sim / ROS 端一律依名稱對應。

## 任務內容

- 觀測（48 維）：機身線速度、角速度、重力方向、速度指令 (vx, vy, wz)、12 個關節角度與速度、上一步動作。
- 動作：`關節目標 = 站姿 + 0.25 × action`。
- 為了配合 Nav2，和 Isaac Lab 預設的速度追蹤任務相比改了：
  - `heading_command=False`：wz 直接取樣（Nav2 直接送 wz）
  - 零指令環境 10%：學會站著不動
  - 速度追蹤獎勵 `std` 0.25（第 1 階段覆寫成 0.5）：Nav2 常送 0.05～0.2 的小指令，低速也要追準
  - `base_height_l2`（目標 0.23 m，權重 −50）：沒有它時策略會壓低機身趴著爬（v7）；`gait_eval` 也要求平均高度 ≥ 0.18 m
  - `feet_air_time` 門檻 0.5 → 0.2 s：這隻狗每步騰空不到 0.5 s，預設值會讓每一步都扣分，策略學成站著不動
  - `feet_air_time` 改成 `feet_air_time_any_motion`（`velocity/mdp.py`）：`|cmd_xy| > 0.05` 或 `|wz| > 0.05` 就給抬腳獎勵；
    Isaac Lab 原版只看 `|cmd_xy| > 0.1`，0.1 m/s 和原地轉向的指令因此學成站著不動
  - 指令取樣 `RobotDogVelocityCommand`（`velocity/mdp.py`）：25% 的環境改給單一軸的小指令（vx/vy 0.05～0.2 m/s 或 wz 0.1～0.3 rad/s）；
    只用均勻取樣時，(0.1, 0, 0) 這種指令只會在站立環境附近出現，策略會把它當成「站著」
- 馬達加了 armature 0.01 kg·m²（減速馬達的轉子等效慣量）。沒有它時，腿很輕、dt 0.005 s 下的 PD 會數值發散，
  腳一離地關節速度就爆掉。
- 馬達（25 N·m、30 rad/s）與質量都是暫定值，換成實際規格後再訓練。

## 測試

```bash
uv run --extra isaacsim --with pytest python -m pytest ~/dog_12dof_ws/robot_dog_description/isaaclab/tests -q
```
