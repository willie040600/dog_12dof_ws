# 機器狗對話視窗（含相機畫面）— 設計

日期：2026-11-11
前一份 spec：`2026-11-10-agent-nav-chat-design.md`。本文件在 `robot_dog_agent` 裡新增一個桌面視窗，對話流程
（Gemini → 工具 → Nav2）完全沿用，不改 RL 策略、Isaac Sim 腳本、Nav2 參數、地圖。

## 1. 目標與範圍

把終端機對話做成一個視窗，裡面同時看得到機器狗的相機畫面。

使用者的決定：
- 形式是**桌面視窗**（PyQt5）。
- 視窗裡有：**相機畫面、對話、停止按鈕、狀態列、八個地點的快捷按鈕**。不做小地圖。

我的決定：
- 終端機版 `chat` 保留（`--no-robot`、`agent_eval` 都還在用它的元件）；視窗是新指令 `gui`。
- `gui` 也有 `--no-robot`（假機器人、沒有相機、不需要 ROS）。
- 不安裝新套件：PyQt5 5.15、numpy 已隨 ROS 裝好。

### 成功條件

1. 新增的測試全部通過，不需要模擬器、網路；視窗測試用 Qt 的 offscreen 模式，不會跳出視窗。原有 68 個測試仍通過。
2. 開著相機的模擬器實測（第 7 節）五項全過。
3. 啟動失敗（沒有 key、Nav2 沒開、沒有定位）時，視窗或終端機有看得懂的訊息，關閉視窗後行程會結束。
4. 關閉視窗、按停止、在啟動它的終端機按 Ctrl+C，都不會留下還在跑的 Nav2 目標。
5. README 說明怎麼啟動、每個元件做什麼、限制。

### 非目標

- 小地圖、在畫面上點選目標、錄影、用相機畫面回答問題（視覺問答是之後的 spec）。
- 指令排隊：一次只執行一個指令。
- 遠端或手機使用。

## 2. 視窗

```
┌─ 機器狗 ────────────────────────────────────────────────────┐
│ ┌──────────────────────────┐ ┌────────────────────────────┐ │
│ │        相機畫面          │ │ 你：去廚房                  │ │
│ │  （保持 4:3，隨視窗縮放） │ │   [工具] go_to_place … → reached │
│ │                          │ │ 狗：我已經到達廚房了。      │ │
│ └──────────────────────────┘ │                            │ │
│ [起點][客廳西側][主臥室][次臥室] └────────────────────────────┘ │
│ [餐桌旁][廚房][玄關][浴室門口]  [ 輸入指令…          ][送出] │
│ [        停 止        ]                                     │
├─────────────────────────────────────────────────────────────┤
│ 位置 (2.48, -0.61) 朝向 -90°　客餐廚　｜　導航中，剩 1.2 m   │
└─────────────────────────────────────────────────────────────┘
```

| 元件 | 行為 |
|---|---|
| 相機畫面 | 每 0.1 秒取最新一張影像顯示。超過 3 秒沒有新影像（或從未收到）時改顯示「沒有相機畫面（Isaac Sim 是不是加了 `--no-camera`？）」；`--no-robot` 時顯示「--no-robot：沒有相機」。 |
| 對話區 | 唯讀。每行一則：`你：…`、`  [工具] …`（和終端機版同樣的格式）、`狗：…`、`[錯誤] …`、Gemini 重試的提示。自動捲到最底。 |
| 輸入框 + 送出 | Enter 或按「送出」送出指令並清空輸入框。空白不送。指令執行中、以及還沒就緒時，輸入框、送出、地點按鈕都鎖住。 |
| 地點按鈕 | 八個，文字是地點名稱在「（」之前的部分。按下等於送出「去」加上這個文字，例如「去廚房」，同樣經過 Gemini。 |
| 停止 | 只在指令執行中可以按。見第 4 節。 |
| 狀態列 | 每 0.5 秒更新。左半：`位置 (x, y) 朝向 θ°　區域`，拿不到位置時是「位置未知」。右半是狀態：「等 Nav2 啟動…」「待命」「處理中…」「導航中，剩 1.2 m」「啟動失敗」。區域用中文：客餐廚、主臥室、次臥室、浴室。 |

啟動：`ros2 run robot_dog_agent gui [--model <id>] [--no-robot]`。

- 沒有可用的 `GEMINI_API_KEY`：和 `chat` 一樣在終端機印出說明，結束碼 2，不開視窗。
- 視窗馬上出現，狀態是「等 Nav2 啟動…」；相機畫面在 Nav2 就緒之前就可以顯示。
- Nav2 就緒且有定位後變成「待命」，對話區印出一行使用提示。沒有定位（60 秒）則對話區印出原因，狀態「啟動失敗」，
  輸入維持鎖住。
- 關閉視窗、或在啟動它的終端機按 Ctrl+C：取消進行中的導航，收掉 ROS 節點，結束。

## 3. 架構

```
robot_dog_agent/robot_dog_agent/
├── session.py      Session：一次一個指令，在背景執行緒跑 Agent，往外送事件（純 Python）
├── frames.py       to_rgb：把 sensor_msgs/Image 的內容轉成 RGB888 位元組（純 Python + numpy）
├── camera_feed.py  CameraFeed：訂閱 /camera/image_raw，只留最新一張（rclpy）
├── gui.py          MainWindow 和 main()（PyQt5）
├── robot.py        機器人介面加 cancel() / clear_cancel()；FakeRobot 跟著實作
├── nav2_robot.py   Nav2Robot 實作 cancel() / clear_cancel()；rclpy 已初始化時不重複初始化；pose(wait=0) 不等待
└── chat.py         build_agent 多一個 on_progress 參數
```

執行緒：

| 執行緒 | 做什麼 |
|---|---|
| Qt 主執行緒 | 視窗。兩個計時器：0.1 秒取影像、0.5 秒更新狀態列。 |
| 指令執行緒 | 啟動時建立 `Nav2Robot`（會等 Nav2）；之後每個指令一條，跑 `Agent.ask`（Gemini 請求和導航都在這裡等）。 |
| 相機執行緒 | 自己的節點和 executor，收影像、存最新一張。 |
| TF 執行緒 | `Nav2Robot` 原有的。 |

跨執行緒只有三條路：`Session` 的事件經由 Qt signal（排隊）送到主執行緒；主執行緒用計時器**拉**最新影像和位置
（`CameraFeed.latest()`、`robot.pose(wait=0)`，都有鎖或本來就可從別的執行緒呼叫）；主執行緒呼叫 `robot.cancel()`
（只設一個 `threading.Event`）。Qt 元件只在主執行緒碰。

## 4. Session 與停止

`Session(on_event, run=<開執行緒>)`：
- `start(build)`：在背景執行 `build() -> (agent, robot)`。成功 → 狀態 `idle`；丟 `RuntimeError` → 事件 `error`、狀態 `failed`。
- `submit(text) -> bool`：空白、或狀態不是 `idle` 時回 `False`，什麼都不做。否則狀態 `busy`、送 `user` 事件、
  `robot.clear_cancel()`，在背景跑 `agent.ask(text)`，結束後送 `reply`（或 `error`）並回到 `idle`。
- `stop()`：狀態是 `busy` 時呼叫 `robot.cancel()`。
- `shutdown()`：`robot.cancel()`，等指令執行緒結束（最多 10 秒），`robot.close()`。
- `state`：`starting`、`idle`、`busy`、`failed`。`remaining`：導航中最近一次回報的剩餘距離，工具呼叫結束時清掉。
- 事件 `on_event(kind, text)`，`kind` 是 `user`、`tool`、`reply`、`error`、`notice`（Gemini 重試提示）、`state`。
  錯誤的處理和 `run_chat` 相同：`GeminiError` 印訊息，其他例外只印類別名稱。

機器人介面的取消：
- `cancel()`：可從任何執行緒呼叫。有導航在進行 → 取消它，`go_to` 回 `canceled`。沒有導航在進行 → 記住，下一次
  `go_to` 不送目標、直接回 `canceled`（用在「正在等 Gemini 回覆時按了停止」）。
- `clear_cancel()`：忘掉還沒用到的取消。每個指令開始時呼叫，上一個指令留下的取消不會影響下一個。
- `canceled` 之後，`Agent` 原本就會跳過同一輪剩下的工具呼叫，並把結果送回 Gemini。
- 終端機版的 Ctrl+C（`KeyboardInterrupt`）行為不變。

## 5. 相機

- `CameraFeed`：節點 `robot_dog_agent_camera`（`use_sim_time`），以 sensor data QoS 訂閱 `/camera/image_raw`。
  回呼只做一件事：把 `(width, height, RGB 位元組, 收到的牆上時間)` 存成最新一張。`latest()` 回傳它或 `None`。
- `to_rgb(encoding, width, height, step, data) -> bytes | None`：支援 `rgb8`（模擬器送的）、`bgr8`、`rgba8`、`bgra8`、
  `mono8`；處理每列有填充（`step` 大於 `width × 通道數`）的情況；格式不支援或資料長度不夠回 `None`。
- 視窗把 RGB 位元組包成 `QImage(Format_RGB888)`，縮放時保持比例。
- 模擬器的影像是 640×480、約每秒 9 張（牆上時間）；視窗每秒取 10 次，不會積壓。

## 6. 測試（不需要模擬器）

- **frames**：五種格式的像素值；有列填充的影像；不支援的格式；資料太短。
- **robot**：`FakeRobot.cancel()` 後下一次 `go_to` 回 `canceled` 且沒有記錄目標；`clear_cancel()` 後恢復正常；
  取消只生效一次。
- **session**（用同步的 `run`，不開執行緒）：空白和忙碌時 `submit` 回 `False`；就緒前不能送；事件順序
  `state busy → user → tool → reply → state idle`；`GeminiError` 和其他例外都變成 `error` 事件並回到 `idle`；
  每個指令開始時清掉取消；`stop()` 只在 `busy` 時呼叫 `cancel()`；進度更新 `remaining`、工具結束時清掉；
  `start` 失敗 → `failed`；`shutdown` 取消並關閉機器人。
- **gui**（Qt offscreen，假的 Session）：八個地點按鈕各送出對應的文字；Enter 和「送出」都會送出並清空；
  空白不送；狀態 `busy` 時輸入、送出、地點按鈕鎖住而停止可按，`idle` 時相反，`starting`、`failed` 時全部鎖住；
  各種事件在對話區的文字；有影像時顯示、超過 3 秒變成提示文字；狀態列的各種文字；關閉視窗會呼叫 `shutdown`。
- **chat**：`build_agent` 的 `on_progress` 參數。

## 7. 實機檢查

模擬器要開相機（`--headless`，不加 `--no-camera`），定位模式。用一支驅動腳本以 offscreen 模式跑真正的視窗元件，
用 `/odom_gt` 判定：

| # | 動作 | 通過條件 |
|---|---|---|
| 1 | 啟動後等待 | 30 秒內相機畫面出現；狀態變成「待命」 |
| 2 | 按「主臥室」 | 回到「待命」時真值位置在主臥室 0.25 m 內；途中截的兩張相機畫面不同（畫面有在更新） |
| 3 | 按「次臥室」，導航開始 5 秒後按「停止」 | 對話區出現 `canceled`；10 秒內回到「待命」；之後 3 秒內真值位置移動 < 0.1 m |
| 4 | 待命時讀狀態列 | 顯示的位置和真值相差 < 0.15 m，區域正確 |
| 5 | 關閉視窗 | 行程在 15 秒內結束；Nav2 沒有進行中的目標 |

另外存一張視窗截圖給使用者看。`--no-robot` 模式也開一次，送一句話確認有回覆。

## 8. 風險

- **rclpy 和 Qt 的執行緒**：`BasicNavigator` 在指令執行緒裡 spin，相機另有 executor。只要不從主執行緒呼叫會 spin
  的函式就沒問題；狀態列用的 `pose(wait=0)` 只查 TF buffer。
- **開相機後模擬變慢**：即時率約從 0.6 降到 0.5（之前量過）。
- **offscreen 模式的中文字**：截圖若出現方框是字型問題，不影響真的視窗；實機檢查不以截圖的文字為判定依據。
- **Gemini 額度**：地點按鈕也經過 Gemini，每按一次兩次請求。實機檢查約用 6 次。

## 9. 文件

- `robot_dog_agent/README.md`：加「視窗版」一節（啟動、元件說明、停止、相機沒畫面時怎麼辦、`--no-robot`）。
- 根目錄 `README.md`：「用中文指揮機器狗」一節加上 `gui` 指令。
