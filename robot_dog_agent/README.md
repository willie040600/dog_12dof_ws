# robot_dog_agent

在終端機用中文指揮模擬裡的機器狗。你打「去主臥室」，Gemini 決定要呼叫哪個工具，工具再把目標送給 Nav2。
行走仍由 RL 策略負責，路徑和避障仍由 Nav2 負責；這個套件只是最上層的「聽懂人話」。

設計見 [docs/superpowers/specs/2026-11-10-agent-nav-chat-design.md](../docs/superpowers/specs/2026-11-10-agent-nav-chat-design.md)。

## 申請和設定 API key

用的是 Google AI Studio 的免費 Gemini API。

1. 到 <https://aistudio.google.com/> 申請 API key。
2. 在 `~/.bashrc` 加一行 `export GEMINI_API_KEY=你的key`。
3. 開一個新的終端機。

注意：

- key 不要寫進 repo 裡的任何檔案。程式只從環境變數讀取。
- 免費額度送出的內容（你打的字、工具的結果）會被 Google 用來改進產品。這裡只有模擬的座標和指令。
- 免費額度有每分鐘和每日的請求上限，各模型不同，數字在 AI Studio 的專案頁面看。一個指令用 2～3 次請求。
  實測（2026-11-10）`gemini-3.6-flash` 每天只有 20 次，大約八個指令就用完，所以預設用 `gemini-3.5-flash-lite`。
  額度用完時程式會告訴你要等多久。

## 編譯

```bash
cd ~/dog_12dof_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation robot_dog_agent
```

## 啟動

先照[根目錄 README](../README.md)「啟動導航」開好 Isaac Sim（終端 1）和 Nav2（終端 2）。定位模式（`slam:=False`）
和建圖模式都可以。然後開第三個終端機：

```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 run robot_dog_agent chat
```

看到 `你>` 就可以打字。對話隨時可以啟動，不需要狗在起點。

如果一直停在「等 Nav2 啟動…」並重複印出 `bt_navigator/get_state service not available, waiting...`，
代表 `nav.launch.py` 還沒開好；按 Ctrl+C 可以取消。

```
你> 去廚房
  剩 2.6 m
  剩 1.4 m
  [工具] go_to_place {"place": "kitchen"} → reached
狗> 我已經到達廚房了。
```

- **Ctrl+C**：導航中按，會取消這次導航，狗停下來，回到 `你>`。在 `你>` 按則結束程式。
- **`exit`**：結束程式。

## 能說的話

| 類型 | 例子 |
|---|---|
| 去有名字的地點 | 去主臥室、到玄關、回起點 |
| 連續幾步 | 先去廚房，再回起點 |
| 相對移動 | 往前走半公尺、後退半公尺、向左轉 90 度、往右橫移 30 公分 |
| 指定座標 | 去座標 (1.0, 0.0) |
| 問位置 | 你現在在哪裡 |

走不到的目標（在牆裡、家具上、離障礙物不到 0.30 m）會直接被拒絕並說明原因，狗不會動。公寓不大，
「往前走一公尺」常常會因為目標點太靠近家具而被拒絕，這時改說短一點的距離。

原地轉向用 Nav2 的 `spin`，實測會多轉 2～5°（說 90° 轉了 92～95°）。

走到地點後還會再用 `spin` 把朝向修正到位：Nav2 自己的朝向容許值是 0.25 rad，一進入就停，實測最多差到 0.37 rad；
修正後剩 0.04～0.06 rad。

### 地點

座標是地圖座標（公尺）：狗出生在原點、面向 +x，+y 在牠左手邊。

| 名稱 | id | x, y | 到達後面向 |
|---|---|---|---|
| 起點（客廳中央，沙發和電視櫃之間） | `start` | 0.0, 0.0 | 0° |
| 客廳西側 | `living_room` | −2.5, −0.3 | 180° |
| 主臥室 | `master_bedroom` | −1.5, 3.0 | 90° |
| 次臥室 | `second_bedroom` | 3.0, 2.2 | 0° |
| 餐桌旁（餐桌東側） | `dining_table` | 3.6, 0.3 | 180°（面向餐桌） |
| 廚房（流理台前） | `kitchen` | 2.5, −0.6 | −90°（面向流理台） |
| 玄關 | `entrance` | −2.0, −0.9 | −90°（面向大門） |
| 浴室門口（門外） | `bathroom_door` | 3.6, −0.5 | 0°（面向浴室） |

要加地點：改 [places.py](robot_dog_agent/places.py) 的 `PLACES`，再跑測試（會檢查走得到、淨空夠、不和別的地點太近）。

## 選項

| 選項 | 用途 |
|---|---|
| `--no-robot` | 不需要 ROS 和模擬器，用一隻「瞬間到達」的假狗。用來確認 key 可用、試對話 |
| `--model <id>` | 換 Gemini 模型，預設 `gemini-3.5-flash-lite` |

```bash
ros2 run robot_dog_agent chat --no-robot
```

## 驗收（`agent_eval`）

重新啟動 Isaac Sim 和 Nav2（定位模式，狗在起點），然後：

```bash
cd ~/dog_12dof_ws && source install/setup.bash
ros2 run robot_dog_agent agent_eval
```

它依序說六句話，用模擬器的真值位置判定，最後印 `[AGENT] PASS k/6`：

| # | 句子 | 通過條件 |
|---|---|---|
| 1 | 去主臥室 | 停在主臥室的 0.25 m、0.3 rad 內 |
| 2 | 先去廚房，再回起點 | 第一段停在廚房 0.25 m 內，最後停在起點 0.25 m、0.3 rad 內 |
| 3 | 往前走半公尺 | 往前 0.5 ± 0.25 m，橫向偏移 ≤ 0.25 m |
| 4 | 向左轉 90 度 | 轉 90° ± 17°，位置移動 ≤ 0.25 m |
| 5 | 去座標 (0.4, 2.7) | 床的中央：工具必須回 `blocked`，沒有送出目標，狗不動 |
| 6 | 你現在在哪裡 | 有查位置（`get_pose`）、有回答，沒有送出目標，狗不動 |

2026-11-10 用 `gemini-3.5-flash-lite`、定位模式、`--headless --no-camera` 連續跑兩輪，都是 6/6，沒有跌倒：

| # | 第一輪 | 第二輪 |
|---|---|---|
| 1 | 離主臥室 0.01 m、0.01 rad | 0.04 m、0.02 rad |
| 2 | 離起點 0.05 m、0.04 rad | 0.05 m、0.06 rad |
| 3 | 往前 0.45 m，橫向 0.04 m | 往前 0.44 m，橫向 0.05 m |
| 4 | 轉了 92°，移動 0.02 m | 轉了 93°，移動 0.02 m |
| 5 | 工具回 `blocked`，沒有送出目標，沒有移動 | 同左 |
| 6 | 查了位置並回答「在起點附近」，沒有移動 | 同左 |

一輪用 12 次請求（六句各兩次）。

不需要模擬器和網路的測試：

```bash
cd ~/dog_12dof_ws/robot_dog_agent && python3 -m pytest test -q
```

## 已知限制

- **只認得上表的地點**，不認得家具。說「去冰箱旁邊」會回答不知道在哪裡。
- **浴室裡面沒有地點**（淋浴盤有 5 cm 高低差）。
- **目標檢查用的是公寓的真值平面圖**，不是 SLAM 建出來的地圖，所以只適用這個場景。
- **出錯後會忘掉之前的對話**：一句話做到一半失敗（被限流到重試用完、等回覆時按 Ctrl+C）時，下一句會從新的對話開始。
- **模擬比真實時間慢**（即時率約 0.5～0.6），走一段路要等的時間大約是模擬時間的兩倍。
- **建圖模式**下主臥門口偶爾會卡住，這是導航本身的已知問題，見 [robot_dog_navigation/README.md](../robot_dog_navigation/README.md)。

## 檔案

| 檔案 | 內容 | 需要 ROS |
|---|---|---|
| `places.py` | 地點表、相對移動的幾何、目標檢查 | 否 |
| `tools.py` | 給 Gemini 的四個工具和說明文字，以及執行工具的 `Toolbox` | 否 |
| `agent.py` | 對話迴圈：送出、執行工具呼叫、回傳結果 | 否 |
| `gemini.py` | Gemini Interactions API 客戶端（`requests`） | 否 |
| `robot.py` | 機器人介面和假機器人 | 否 |
| `nav2_robot.py` | 真的機器人：Nav2 目標、TF 位置、取消 | 是 |
| `chat.py` | 終端機對話 | `--no-robot` 時否 |
| `eval_cases.py`、`agent_eval.py` | 六句話的判定和執行 | 後者是 |
