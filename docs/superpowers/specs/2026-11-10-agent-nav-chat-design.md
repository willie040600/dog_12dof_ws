# 用自然語言指揮機器狗（Gemini agent）— 設計

日期：2026-11-10
前面的 spec：`2026-11-01-robot-dog-isaac-nav2-design.md`、`2026-11-03-apartment-world-design.md`。
本文件只新增一個上層套件，RL 策略、Isaac Sim 腳本、Nav2 參數、地圖都不改。

## 1. 目標與範圍

在終端機用中文對機器狗下指令，例如「去主臥室」「先去廚房，再回起點」「往前走一公尺」，由 Gemini 決定要呼叫
哪個工具，工具再把目標送給 Nav2。

使用者的決定：
- LLM 用 **Google AI Studio 的免費 Gemini API**。
- 第一版**只做導航**，不用相機。
- 介面是**終端機對話**。
- 聽得懂**有名字的地點**，以及**相對移動和座標**。

### 成功條件

1. 新套件的純 Python 測試全部通過，不需要網路、ROS、模擬器。
2. 定位模式（AMCL）下 `agent_eval` 連續兩次都 `[AGENT] PASS 6/6`（第 7 節），全程沒有跌倒。
3. 沒有設定 API key、key 無效、被限流、沒有網路時，程式印出看得懂的訊息，不會留下還在跑的 Nav2 目標。
4. API key 只從環境變數讀取，不出現在 repo、log 或錯誤訊息裡。
5. README 說明怎麼申請 key、怎麼啟動、能說哪些話、免費額度的限制。

### 非目標

- 相機與視覺問答、語音、巡邏報告、語意地圖（之後各自一份 spec）。
- 程式重開後還記得之前的對話。
- 真實機器狗；一次指揮多隻狗；回覆逐字串流。
- 建圖模式可以用，但不列入驗收：主臥門口偶爾卡住是已知問題。

## 2. Gemini API（2026-11-10 查官方文件）

| 項目 | 內容 |
|---|---|
| 介面 | **Interactions API**（2026 年 6 月起正式版，官方建議新專案使用；`generateContent` 已列為舊版） |
| 端點 | `POST https://generativelanguage.googleapis.com/v1beta/interactions` |
| 認證 | 標頭 `x-goog-api-key`，值來自環境變數 `GEMINI_API_KEY` |
| 請求 | `model`、`input`、`system_instruction`、`tools`、`previous_interaction_id` |
| 工具宣告 | `{"type": "function", "name", "description", "parameters": JSON Schema}` |
| 工具呼叫 | 回應的 `steps` 裡 `type: "function_call"` 的項目，帶 `name`、`arguments`、`id` |
| 回傳結果 | `input` 放 `{"type": "function_result", "name", "call_id", "result": [{"type": "text", "text"}]}` |
| 對話延續 | `previous_interaction_id`；`system_instruction` 和 `tools` 每次請求都要重送 |
| 保存 | 伺服器保存互動紀錄，免費額度保留 1 天 |
| 免費額度 | `gemini-3.8-flash`、`gemini-3.6-flash`、`gemini-3.5-flash-lite` 等都有；送出的內容會被 Google 用來改進產品 |
| 速率上限 | 文件不列數字，只能在 AI Studio 的專案頁面看；超過時回 `429` |

- **預設模型**：`gemini-3.6-flash`（文件標為 Stable）。用 `--model` 可以換。
- **不裝 SDK，直接用 `requests` 呼叫 REST**。Ubuntu 24.04 的系統 Python 擋 `pip install`，ROS 節點又只能用
  系統 Python；`python3-requests` 已經裝好，而且只有一個端點。
- 文件沒有列出回應 JSON 的完整欄位，所以先用真的 key 打了六次 API，把請求和回應存在
  `robot_dog_agent/test/data/`，客戶端照這些範例檔寫、也用它們測試。實測（2026-11-10，`gemini-3.6-flash`）：
  - 上表的請求欄位全部被接受；`system_instruction` 是字串。
  - 回應有 `id`、`status`（`completed` 或 `requires_action`）、`steps`、`usage`。文字回覆在
    `type: "model_output"` 的 step 的 `content[].text`；另外有 `type: "thought"` 的 step（只有簽章），忽略即可。
  - 「先去廚房，再回起點」在同一個回應裡給了兩個 `function_call`。
  - 不存在的 `previous_interaction_id` 回 `400`，訊息只有「Request contains an invalid argument.」，
    和其他參數錯誤分不出來。
  - 無效的 key 也是回 `400`（不是 `401`），回應內容是只有一個元素的 list，訊息含「API key not valid」。

## 3. 架構

新套件 `robot_dog_agent`（ament_python），和 `robot_dog_navigation` 並列：

```
robot_dog_agent/
├── package.xml  setup.py  setup.cfg  resource/
├── robot_dog_agent/
│   ├── places.py     地點表、相對移動的幾何、目標檢查（純 Python）
│   ├── tools.py      給 Gemini 的工具宣告，以及執行工具的 Toolbox（純 Python）
│   ├── gemini.py     Interactions API 客戶端（requests）
│   ├── agent.py      對話迴圈：送出、執行工具呼叫、回傳結果，直到拿到文字回覆（純 Python）
│   ├── robot.py      機器人介面（pose、go_to）和 FakeRobot：不需要 ROS 的替身（純 Python）
│   ├── nav2_robot.py Nav2Robot：Nav2 目標、目前位置、取消（rclpy）
│   ├── chat.py       終端機對話的入口
│   ├── eval_cases.py 第 7 節六句話的判定（純 Python）
│   └── agent_eval.py 第 7 節的實機檢查
└── test/
```

依賴方向：`chat` → `agent`、`gemini`、`tools` → `places`、`robot`。`agent`、`tools`、`places`、`robot`、
`eval_cases` 不匯入 rclpy 和 requests，測試時換成假的 LLM 和 `FakeRobot`。

資料流程：

```
你打字 → agent → Gemini（system_instruction + tools）
                   ↓ function_call
                 Toolbox → 目標檢查 → Robot → Nav2 NavigateToPose
                   ↓ 結果（到達／失敗＋原因＋最後位置）
                 Gemini → 文字回覆 → 印在終端機
```

## 4. 地點與幾何（`places.py`）

座標是 map frame，等於世界座標（公寓座標 − (3.5, 1.8)）；狗出生在原點，面向 +x，+y 在牠左手邊。

| id | 名稱 | x, y, 朝向 | 淨空 |
|---|---|---|---|
| `start` | 起點（客廳中央，沙發和電視櫃之間） | 0.0, 0.0, 0° | 0.70 |
| `living_room` | 客廳西側 | −2.5, −0.3, 180° | 0.85 |
| `master_bedroom` | 主臥室 | −1.5, 3.0, 90° | 0.95 |
| `second_bedroom` | 次臥室 | 3.0, 2.2, 0° | 0.85 |
| `dining_table` | 餐桌旁（餐桌東側，面向餐桌） | 3.6, 0.3, 180° | 0.65 |
| `kitchen` | 廚房（流理台前，面向流理台） | 2.5, −0.6, −90° | 0.45 |
| `entrance` | 玄關（面向大門） | −2.0, −0.9, −90° | 0.82 |
| `bathroom_door` | 浴室門口（門外，面向浴室） | 3.6, −0.5, 0° | 0.55 |

前四個和 `nav_goals.GOALS` 的座標相同；後四個是新的，已用真值地圖 `apartment_body` 確認走得到，淨空如表
（流理台在 x 1.35～3.75、y −1.65～−1.07，由 USD 讀出）。浴室裡面不放地點（淋浴盤 5 cm）。地點兩兩相距至少 0.78 m。

函式：
- `find_place(id)`：查表，查不到回 `None`。
- `relative_target(pose, forward, left, turn_left_deg)`：目前位置加上機身座標的位移，朝向加上轉角。
- `check_target(truth, x, y)`：回 `None` 表示可以去，否則回原因。依序檢查：不在地圖範圍內、在障礙物上、
  和起點不連通（例如床的中央）、淨空小於 `MIN_CLEARANCE = 0.30` m（機身對角線的一半是 0.27 m）。
  用 `robot_dog_navigation.nav_goals` 的 `load_truth`、`clearance`、`reachable`。
- `area_of(x, y)`：客餐廚、主臥、次臥、浴室（用公寓座標的分界：y 3.0、x 5.0、浴室 x ≥ 7.8 且 y ≤ 2.1）。
- `nearest_place(x, y)`：最近的地點和距離。

目標檢查用的是真值地圖，不是 SLAM 地圖：場景固定是這間公寓，而且這樣檢查不依賴 Nav2 是否已啟動。

## 5. 工具與對話迴圈

### 5.1 工具（`tools.py`）

| 工具 | 參數 | 做什麼 |
|---|---|---|
| `get_pose` | 無 | 回傳 x、y、朝向（度）、所在區域、最近的地點和距離。狗不動。 |
| `go_to_place` | `place`（八個 id 的 enum） | 去表上的地點，用表上的朝向。 |
| `go_to_pose` | `x`、`y`，可選 `yaw_deg` | 去指定座標。沒給朝向時，面向行進方向（從目前位置指向目標）；目標離目前位置不到 0.1 m 時維持目前朝向。 |
| `move_relative` | `forward_m`、`left_m`、`turn_left_deg`，都可省略，預設 0 | 相對目前位置移動或原地轉向。位移各自限制在 ±5 m。 |

沒有 `list_places`：地點表直接寫在 system instruction 裡，省一次請求。
沒有 `stop`：導航工具會等 Nav2 結束才回傳，中途停止靠 Ctrl+C（5.3）。

三個導航工具的流程相同：算出目標 → `check_target` → `Robot.go_to` → 回傳結果。
回傳的結果是一個 dict，轉成 JSON 文字交給 Gemini：

```
{"status": "ok" | "reached" | "blocked" | "failed" | "timeout" | "canceled" | "skipped" | "error",
 "reason": "...",                      # ok、reached 以外都有
 "x": 2.48, "y": -0.61, "yaw_deg": -88, # 結束時的位置（blocked、skipped、error 是目前位置）
 "seconds": 21.4}                       # 模擬時間
```

- `ok`：只有 `get_pose` 會回，另外帶 `area`、`nearest_place`、`distance_to_place`，沒有 `seconds`。
- `blocked`：`check_target` 不通過，沒有送給 Nav2。
- `failed`：Nav2 拒絕目標，或回報 FAILED。`timeout`：超過 120 秒模擬時間，已取消。
- `error`：參數缺少、不是數字、地點 id 不存在、工具名稱不存在。Toolbox 不丟例外，一律回 dict。

原地轉向（`move_relative` 只給轉角）送的是「位置不變、朝向改變」的 NavigateToPose 目標。

### 5.2 對話迴圈（`agent.py`）

`Agent.ask(text)`：

1. 把文字送給 Gemini（帶 `previous_interaction_id`，第一句沒有）。
2. 回應裡沒有 `function_call`：回傳文字，結束。
3. 有的話，**依序**執行每個呼叫（同一個回應裡可能有好幾個，例如「先去廚房，再回起點」）。
   其中一個的結果不是 `ok` 或 `reached` 時，後面的不執行，結果填 `skipped`。
4. 把全部結果送回 Gemini，回到第 2 步。
5. 一個指令最多 8 輪；超過就停下來，告訴使用者這個指令用了太多步驟。
6. 回應既沒有文字也沒有工具呼叫時，回一句固定的訊息請使用者再說一次。
7. 中途丟出任何例外（重試用完、等 Gemini 時按 Ctrl+C），或超過 8 輪，都**重新開始對話**（不再帶
   `previous_interaction_id`）：伺服器上的對話停在沒有結果的工具呼叫，接不下去。代價是忘掉之前的對話。

system instruction 的內容：
- 角色：你透過工具控制模擬公寓裡的一隻四足機器狗。
- 座標系、單位（公尺、度、左轉為正）、第 4 節的地點表。
- 規則：有名字的地點一律用 `go_to_place`，不要自己猜座標；工具回 `blocked` 時告訴使用者原因，不要自己
  改去附近的點；不需要動的問題不要呼叫導航工具；不知道位置的東西（例如「冰箱」）直接說不知道。
- 一律用繁體中文、簡短回覆。

### 5.3 終端機對話（`chat.py`）

```
ros2 run robot_dog_agent chat [--model gemini-3.6-flash] [--no-robot]
```

- 啟動時檢查 `GEMINI_API_KEY`；沒有就印出申請和設定方法後結束（不啟動 ROS）。
- 等 Nav2 啟動完成、而且拿得到 map→base_footprint（最多等 60 秒），印出地點清單，開始讀輸入。定位模式和
  建圖模式都一樣，不需要旗標。**不設定 AMCL 的初始位置**：`BasicNavigator.waitUntilNav2Active(localizer='amcl')`
  會往原點發一個初始位置，狗不在原點時啟動對話會讓 AMCL 定位跑掉（實測：下一個目標規劃失敗），所以不用它。
- 導航中每 2 秒印一次剩餘距離（Nav2 還沒有路徑時回報 0，不印）。輸出立即送出，接管線時也看得到進度。
- **導航中按 Ctrl+C**：取消 Nav2 目標，工具結果是 `canceled`，後面的呼叫 `skipped`，結果照常送回 Gemini
  （否則伺服器上的對話會停在沒有結果的工具呼叫），印出它的回覆，回到輸入提示。
- **輸入提示下按 Ctrl+C、Ctrl+D，或輸入 `exit`**：結束程式。
- `--no-robot`：用 `FakeRobot`（目標直接瞬間到達），不需要 ROS 和模擬器。用來確認 key 可用、試對話。
- rclpy 以 `SignalHandlerOptions.NO` 初始化，Ctrl+C 才會變成 Python 的 `KeyboardInterrupt`，而不是把
  rclpy 關掉。TF 用獨立的節點和執行緒（`nav_eval` 的做法：`BasicNavigator` 會自己 spin）。

### 5.4 Gemini 客戶端的錯誤處理（`gemini.py`）

| 情況 | 處理 |
|---|---|
| `429` | 印「被限流，等 N 秒」，依 `Retry-After` 等待，沒有就等 10、20、40 秒；最多重試 3 次 |
| `5xx`、連線逾時（60 秒）、連不上 | 同上的等待與重試 |
| 其他 `4xx` | 不重試；丟出例外，訊息取自回應內容；訊息含「API key」時提示檢查 `GEMINI_API_KEY` |
| 帶著 `previous_interaction_id` 送文字時回 `400` | 當成對話已過期（實測分不出來）：印「之前的對話已過期，重新開始」，不帶 id 重送一次；還是失敗就丟出例外 |
| 帶著 `previous_interaction_id` 送工具結果時回 `400` | 丟出例外（結果對應的工具呼叫已經不在了，沒辦法重送到新對話） |
| `Retry-After` 超過 120 秒 | 不等，丟出例外並說明要等多久（多半是每日額度用完） |
| 重試用完 | 丟出例外 |

`chat` 接到例外時印出訊息、放棄這一句，回到輸入提示；這時狗一定是停著的（工具呼叫之間沒有進行中的目標）。
key 只放在標頭，不放在網址；例外訊息和 log 不含標頭。

## 6. 測試（純 Python，`robot_dog_agent/test`）

- **places**：八個地點都走得到、淨空 ≥ 0.45 m、兩兩相距 ≥ 0.7 m；`relative_target` 的方向（朝向 90° 時
  前進 1 m 是 +y）；`check_target` 拒絕床中央 (0.4, 2.7)、牆上的點、地圖外的點、緊貼牆的點，接受起點；
  `area_of`、`nearest_place`。
- **tools**：宣告的 enum 和地點表一致；`go_to_place` 把表上的座標交給機器人；`blocked` 時機器人沒有被呼叫；
  `go_to_pose` 沒給朝向時面向行進方向；`move_relative` 從目前位置起算；各種壞參數回 `error`。
- **agent**（假的 LLM 照劇本回應）：只有文字；一次呼叫；同一回應兩個呼叫依序執行；第一個失敗時第二個
  `skipped`；超過 8 輪；取消。
- **gemini**（假的 HTTP）：請求內容（模型、工具、system instruction、`previous_interaction_id`、key 在標頭）；
  解析實際錄下的回應範例；`429` 後成功；`401` 的訊息不含 key；過期的 id 重新開始。

## 7. 實機檢查（`agent_eval`）

```
ros2 run robot_dog_agent agent_eval [--model ...]
```

在定位模式下，用真的 Gemini 和真的 Nav2 依序送出六句話，用 `/odom_gt`（真值）判定，每句印一行 `[AGENT]`，
最後印 `[AGENT] PASS k/6`，全過才回傳 0。任何時候機身高度低於 0.12 m 算跌倒，該句失敗。

| # | 句子 | 通過條件 |
|---|---|---|
| 1 | 去主臥室 | 停在 `master_bedroom` 的 0.25 m、0.3 rad 內 |
| 2 | 先去廚房，再回起點 | 第一個導航工具結束時在 `kitchen` 的 0.25 m 內；最後停在 `start` 的 0.25 m、0.3 rad 內 |
| 3 | 往前走一公尺 | 沿原本朝向的位移 1.0 ± 0.25 m，橫向偏移 ≤ 0.25 m |
| 4 | 向左轉 90 度 | 朝向改變 90° ± 17°（0.3 rad），位置移動 ≤ 0.25 m |
| 5 | 去座標 (0.4, 2.7) | 沒有送出 Nav2 目標，位置移動 ≤ 0.1 m（這是床的中央） |
| 6 | 你現在在哪裡 | 沒有送出 Nav2 目標，位置移動 ≤ 0.1 m，回覆不是空的 |

一輪大約用 15 次請求。

## 8. 風險

- **回應格式**：文件沒有完整的回應 JSON。先打一次真的 API 再寫客戶端（第 2 節）。
- **原地轉向**：Nav2 對「位置不變、只改朝向」的目標是否會轉，要實測（第 7 節第 4 句）。不轉的話改用
  behavior server 的 `spin`。
- **模型的工具使用**：`gemini-3.6-flash` 用中文呼叫工具的穩定度要實測。不穩時先改 system instruction 和
  工具說明；再不行換 `gemini-3.8-flash`。
- **免費額度**：上限只能在 AI Studio 看到，而且會變。一個指令用 2～3 次請求；`agent_eval` 一輪約 15 次。
  被限流時等待重試，不會讓狗停在半路（等待發生在兩次工具呼叫之間，狗是停著的）。
- **即時率約 0.5**：一段路的實際等待時間約是模擬時間的兩倍；逾時用模擬時間計算，不受影響。

## 9. 文件

- `robot_dog_agent/README.md`：申請 key、設定環境變數、啟動順序、能說的話、地點表、Ctrl+C、免費額度與
  資料使用的提醒、`--no-robot`。
- 根目錄 `README.md`：加一節指向它。
