# 手機遙控 app：Android 前端 + 電腦端伺服器 — 設計

日期：2026-11-19
前幾份 spec：`2026-11-11-agent-gui-design.md`（視窗）、`2026-11-17-agent-gui-map-design.md`（地圖）、
`2026-11-18-object-memory-design.md`（物件記憶、巡邏）。本文件把同一套操作搬到 Android 手機上。
不改 RL 策略、Isaac Sim 腳本、Nav2 參數、地圖檔；**不改現有的 `gui.py`**。

## 1. 為什麼、給誰用

使用者在家裡拿著手機叫狗：一邊走一邊看狗的相機、看地圖上狗在哪、打字或點按鈕下指令。手機和電腦在**同一個 Wi-Fi**；
不需要從外面連回家。電腦照樣跑 Isaac Sim、Nav2；手機只是遙控畫面。

使用者的決定：
- **Android app 只當前端**：狗、Nav2、Gemini、物件記憶都留在電腦上；app 只顯示、只送指令，不做判斷。Gemini 的 API key 不離開電腦。
- **電腦端是獨立程式** `ros2 run robot_dog_agent phone`，不開視窗。它和 `gui` **二選一**（兩個程式會搶著控制 Nav2）。
  物件記憶是同一個檔案，兩邊都讀得到；對話不共用。
- **傳輸用方案 1**：Python 內建的 HTTP 伺服器；狀態用輪詢、對話事件用長輪詢、相機用 MJPEG（第 4 節）。不用 WebSocket、不用 rosbridge。
- 只在家用 Wi-Fi。
- 手機上的位置和路線要即時（0.2 秒一次，比電腦視窗的 0.5 秒快）。

### 目標

手機上能做的事和電腦視窗相同：

1. 看相機畫面。
2. 看地圖：狗的位置和朝向、Nav2 的路線、點選的目標、記住的物件、八個地點。
3. 點地圖叫狗過去（不經過 Gemini）。
4. 打字下指令、看對話（你／狗／工具紀錄／錯誤）。
5. 八個地點按鈕、巡邏按鈕、停止按鈕、狀態列（位置與區域、待命／處理中…／剩幾公尺）。

### 成功條件

1. Python 測試全部通過（原有 219 個加上新的；不需要模擬器、網路、手機）。
2. Android 的 JVM 單元測試全部通過（`./gradlew test`，不需要手機）。
3. 模擬器實測（第 8 節）全部通過，並截圖給使用者看。
4. API key 不出現在任何回應、紀錄、app、提交裡。
5. 沒有配對碼（或配對碼錯）的請求一律拒絕，狗不動。

### 非目標

- 從家外面連（VPN、Tailscale、port forwarding）、HTTPS。
- iPhone、平板版面、橫向畫面（app 鎖直向）。
- 地圖縮放、平移。
- 手機和電腦視窗同時操作同一隻狗（`phone` 和 `gui` 二選一）。
- 推播通知、背景執行（app 不在前景時就不連線）。
- 語音輸入。
- 上架 Play 商店；只做 debug APK，用 Android Studio 或 `adb install` 裝。

### 反目標（做到了也算失敗）

- 同一個 Wi-Fi 上的其他裝置，不知道配對碼也能叫狗、用掉 Gemini 額度。
- 狗停不下來：手機上的停止按鈕、伺服器終端機的 Ctrl+C 都一定要停得下來。
  （我的決定：手機斷線或關掉 app **不會**讓狗自己停，正在做的指令會做完；這和關掉電腦視窗不同。
  理由：家用 Wi-Fi 偶爾斷一下很常見，斷一下就把狗停掉反而難用。）
- 改了電腦視窗 `gui` 的行為。

## 2. 架構

```
電腦（同一台：Isaac Sim + nav.launch.py 照舊）
  ros2 run robot_dog_agent phone
    ├─ Session（現有）     ← 指令、點地圖、巡邏、停止；一次一件事
    ├─ Nav2Robot（現有）   ← 位置
    ├─ CameraFeed（現有）  ← 相機（彩色）；物件記憶用的深度照舊
    ├─ MapFeed（現有）     ← /map、/plan
    ├─ ObjectMemory（現有）← 記住的物件
    └─ HTTP 伺服器（新）   0.0.0.0:8765，每個請求一條執行緒（ThreadingHTTPServer）
             ↑ 家用 Wi-Fi，HTTP + 配對碼
Android app（新）：Kotlin + Jetpack Compose
```

`phone` 的組裝內容和 `gui.create` 相同（rclpy 初始化、CameraFeed、MapFeed、ObjectMemory、看照片用的 GeminiClient、Eyes、
`chat.build_agent`），只是把視窗換成伺服器。因為 `gui.py` 不改、也不能把 PyQt5 拉進來，這段組裝在 `phone.py` 裡另寫一份，不 import `gui`。
`--no-robot` 照樣可用（FakeRobot、repo 裡的地圖、沒有相機），方便不開模擬器時測 app。

## 3. 配對碼

- 第一次啟動時產生 6 位數字，存在 `~/.config/robot_dog/phone_code`（權限 600）；之後每次啟動沿用同一組，手機只要輸入一次。
  `--new-code` 換一組新的（舊手機就連不上了）。
- 啟動時在終端機印出：`手機連線：http://192.168.0.154:8765　配對碼 123456`（IP 取電腦對外的那張網卡；抓不到就印 0.0.0.0 並提示）。
- 除了 `/api/ping`，每個請求都要帶 header `Authorization: Bearer <配對碼>`，錯了回 401。
- 防猜：配對碼錯一次，回應前等 1 秒；同一個 IP 一分鐘內錯 10 次，之後一分鐘全部回 429。
- 配對碼不是 API key，可以印在終端機；API key 照舊只從環境變數讀，不印、不寫檔、不送給手機。

## 4. 伺服器的介面

全部是 JSON（UTF-8），除了地圖（PNG）和相機（MJPEG）。

| 方法 路徑 | 做什麼 | 回應 |
|---|---|---|
| `GET /api/ping` | 不用配對碼；app 用來確認「這個位址有伺服器」 | `{"app": "robot_dog_phone", "version": 1}` |
| `GET /api/hello` | 連上後拿一次不會變的東西 | `{"boot": "<這次啟動的編號>", "places": [{"id","name","short","x","y"}], "can_survey": bool, "camera": bool}` |
| `GET /api/state?path=<版本>` | 每 0.2 秒一次 | 見下 |
| `GET /api/map.png` | 地圖底圖：灰階 PNG，已上下翻轉（y 朝上），1 像素 = 1 格 | PNG；沒有地圖時 404 |
| `GET /api/events?after=<n>` | 長輪詢：有比 n 新的事件就立刻回，否則最多等 25 秒回空的 | `{"boot", "events": [{"id","kind","text"}], "next": n}` |
| `GET /api/camera.mjpg` | 相機串流，`multipart/x-mixed-replace` | 約 10 張/秒 JPEG |
| `POST /api/command` `{"text"}` | 和視窗打字送出相同（`Session.submit`） | `{"ok": bool}`；不是待命時 `ok=false` |
| `POST /api/go_to` `{"x","y"}` | 點地圖（`Session.go_to`） | 同上 |
| `POST /api/survey` | 巡邏（`Session.survey`）；沒有眼睛時伺服器自己先擋下、回 `ok=false`（`Session.survey` 本身不會擋） | 同上 |
| `POST /api/stop` | 停止（`Session.stop`） | `{"ok": true}` |

`/api/state` 的內容：

```json
{
  "state": "idle | starting | busy | failed",
  "status": "待命",                      // Session.status_text()
  "pose": [x, y, yaw] | null,
  "pose_text": "位置 (0.12, -0.30) 朝向 0°　客餐廚",
  "path_version": 7,
  "path": [[x, y], ...] | null,          // 只在版本和請求的 ?path= 不同時給；[] 表示沒有路線
  "target": [x, y] | null,
  "objects": [{"name", "x", "y"}],
  "map": {"version", "resolution", "origin": [x, y], "width", "height"} | null
}
```

規則（和電腦視窗相同，但由伺服器算，app 不判斷）：
- **路線**只給「這個指令開始之後」收到的 `/plan`；指令結束就變成 `[]`。路線點最多 300 個（多了就平均抽樣）。
- **目標 ✕**：`/api/go_to` 成功後就是點的位置；指令結束後再保留 2 秒；下一個指令一開始就清掉。文字指令沒有 ✕。
- **地圖版本**：`/map` 換新時加一；app 看到版本變了才重抓 `map.png`。

事件：伺服器把 `Session.on_event` 的每一筆（`user`、`reply`、`log`、`error`、`state`）編號存起來，最多保留最近 300 筆。
app 一開就用 `after=0` 拿回目前保留的全部，所以中途打開 app 也看得到前面的對話。多支手機同時連都看到一樣的東西；
指令一次只會有一個執行（第二個 `ok=false`）。

`boot`：伺服器每次啟動時產生的隨機編號，出現在 `/api/hello` 和 `/api/events`。事件編號每次啟動都從頭開始，所以 app 看到
`boot` 變了，就清掉手機上的對話、從 `after=0` 重新拿（我的決定：重開的伺服器是一段新的對話，和電腦視窗重開一樣）。

並行（我的決定）：HTTP 請求各在自己的執行緒，而 `Session.submit`、`go_to`、`survey` 的「是不是待命」檢查沒有鎖
（視窗只有一條執行緒，所以以前不需要）。伺服器用一把自己的鎖包住這三個呼叫，確保兩支手機同時按、或連點兩下時只會有一個指令執行。
`ObjectMemory.items` 會在巡邏寫入時被讀，伺服器讀它時遇到「迭代中被改」就重試一次，再失敗就沿用上一次的清單。

相機：伺服器把最新的彩色畫面（640×480）壓成 JPEG（品質 70），每 0.1 秒送一張；畫面超過 3 秒沒更新就不送新的，
app 顯示「沒有相機畫面」。沒有人在看時不壓縮。

結束：在終端機按 Ctrl+C → 和關掉視窗相同（取消導航、等背景工作、關機器人、關 feeds）；長輪詢和 MJPEG 連線一併斷開。

## 5. Android app

### 5.1 畫面（直向）

```
┌──────────────────────────────┐
│ ● 待命　　　　　　　　　　　 ⚙ │  狀態列：圓點顏色 = 連線／狀態；⚙ 回設定頁
│ 位置 (0.12,-0.30) 0° 客餐廚   │
├──────────────────────────────┤
│ ┌──────────────────────────┐ │
│ │                          │ │  大畫面：地圖或相機（預設地圖）
│ │   地圖：牆、地點、▲ 狗、   │ │
│ │   綠色路線、✕ 目標、物件  │ │
│ │                  ┌──────┐│ │  右下角小窗：另一個（相機）；點一下互換
│ │                  │ 相機 ││ │
│ └──────────────────┴──────┘┘ │
├──────────────────────────────┤
│ 你：去廚房                    │  對話區（可捲動，自動捲到最新）
│   [工具] go_to_place …        │
│ 狗：到廚房了。                │
├──────────────────────────────┤
│ [起點][客廳西側][主臥室][次臥 →│  地點按鈕：一列，可左右滑；最後一顆是「巡邏」
│ [ 輸入指令，例如：去廚房 ][送出]│
│ [            停 止           ]│  紅色大按鈕；只有「處理中…」時能按
└──────────────────────────────┘
```

- 地圖保持原本的長寬比（約 5:3）置中；畫法、顏色和電腦視窗的 `MapWidget` 相同（灰底黑牆、棕色地點、橘色物件方塊、綠色路線、紅色 ✕、藍色三角形）。
- **點地圖**（我的決定，觸控容易誤觸）：待命時點一下，先放一個灰色 ✕ 並在地圖下緣出現「去這裡 (1.20, 0.50)　[去][取消]」；
  按「去」才送 `/api/go_to`。不是待命時點了沒反應。
- 待命以外的時候，地點按鈕、巡邏、輸入框、送出都鎖住；停止只有「處理中…」能按。和電腦視窗相同。
- 對話前綴和電腦視窗相同：`你：`、`狗：`、`[錯誤] `；工具紀錄原樣顯示。
- 鍵盤打開時，對話區縮小、輸入列跟著上移（`imePadding`）。
- app 在前景時螢幕不關（我的決定：叫狗時常常盯著看）。

### 5.2 設定頁

第一次開 app、或按 ⚙ 時出現：

- 電腦位址（預設 `192.168.0.154:8765`）、配對碼（6 位數字）。
- 「連線」：先打 `/api/ping` 確認是這個伺服器，再用配對碼打 `/api/hello`；成功才存起來（SharedPreferences）並回主畫面。
- 失敗訊息分開講：連不上（位址錯、電腦沒開 `phone`、防火牆）、不是這個伺服器、配對碼錯。

### 5.3 連線狀態

- 狀態列圓點：綠＝待命、橘＝處理中／啟動中、紅＝啟動失敗或連不上電腦。
- 連不上時狀態列顯示「連不上電腦，重試中…」，每 2 秒重試；畫面保留最後的樣子但按鈕全鎖。恢復後自動接上：
  同一次啟動（`boot` 沒變）就從上次的事件編號繼續；伺服器重開過（`boot` 變了）就清掉對話、從頭拿。
- app 退到背景時停止輪詢和相機；回前景時重新連。

### 5.4 技術選擇

- Kotlin + Jetpack Compose，單一 Activity。minSdk 26（Android 8.0）；compileSdk / targetSdk 用已安裝的最新版（API 37，若 Android Gradle Plugin 還不支援就用它支援的最新版）。
- 只用 AndroidX、Compose、kotlinx-coroutines；網路用內建的 `HttpURLConnection`，JSON 用內建的 `org.json`，app 本身不加第三方套件。
  例外（我的決定）：Android 內建的 `org.json` 在 JVM 單元測試裡只是空殼，所以**只有測試**加 `testImplementation("org.json:json")`，
  讓 `Model.kt` 的解析能在 `./gradlew test` 裡真的被測到；它不會進 APK。
- MJPEG 自己解析（讀 multipart 邊界，`BitmapFactory.decodeByteArray`）。
- 允許明文 HTTP（Android 9 以後預設禁止）：用 `network_security_config` 開放；只連使用者填的位址。
- 專案放在 repo 的 `robot_dog_phone/`，加 `COLCON_IGNORE`（colcon 不碰它）。Gradle wrapper 一併提交；`build/`、`.gradle/`、`local.properties` 進 `.gitignore`。
- 第一次建置時 Gradle 會從 Google / Maven Central 下載 Android Gradle Plugin 和函式庫（數百 MB），這是 Android 建置的正常過程。

## 6. 程式怎麼分

### 電腦端（`robot_dog_agent`）

| 檔案 | 內容 |
|---|---|
| `phone_state.py`（新，純 Python） | `EventLog`：編號、最多 300 筆、`wait_after(n, timeout)`（Condition）；`MapTracker`：路線新鮮度、目標 ✕ 保留 2 秒、路線版本、抽樣；`state_json(...)`、`grid_png(grid)`（灰階＋翻轉，重用 `map_view.grid_to_gray`）。全部不碰網路、不碰 ROS，方便測試 |
| `phone_code.py`（新） | 讀／產生／換配對碼檔；`Guard`：比對配對碼、錯誤計數與 429 |
| `phone_server.py`（新） | `ThreadingHTTPServer` 和路由；MJPEG 迴圈；只依賴 Session 介面和幾個「拿最新資料」的函式，測試時全部可以換成假的 |
| `phone.py`（新） | `main`：參數（`--port 8765`、`--model`、`--vision-model`、`--no-robot`、`--new-code`）、組裝（內容同 `gui.create`，另寫一份、不 import `gui`）、印連線資訊、Ctrl+C 收尾 |
| `setup.py` | 新增 console script `phone = robot_dog_agent.phone:main` |

不改 `gui.py`、`session.py`、`map_widget.py`。依賴不變（`cv2` 已在用來壓 JPEG／PNG）。

### 手機端（`robot_dog_phone/`）

| 檔案（Kotlin） | 內容 |
|---|---|
| `Api.kt` | 每個介面一個函式；配對碼 header；錯誤分類（連不上／401／429／其他） |
| `Model.kt` | `State`、`Event`、`Place`、`MapInfo`；從 JSON 解析（純 Kotlin，可做單元測試） |
| `MapGeometry.kt` | 地圖座標 ↔ 畫面像素（等比縮放置中、y 朝上），等同 `map_view.MapView`；純 Kotlin，可測 |
| `Mjpeg.kt` | 從串流切出一張張 JPEG；純 Kotlin（只吐 bytes），可測 |
| `RobotViewModel.kt` | 輪詢 `/state`、長輪詢 `/events`、相機、重試；把畫面要的東西做成 Compose 狀態 |
| `MainScreen.kt`、`MapView.kt`、`SettingsScreen.kt` | 畫面 |

## 7. 網路設定（使用者自己做）

這台電腦開著 ufw 防火牆，手機連進來前要開放 8765 port（只開給家裡的網段）。這是系統安全設定，由使用者自己執行：

```bash
sudo ufw allow from 192.168.0.0/24 to any port 8765 proto tcp
```

README 會寫這一行。模擬器連 `10.0.2.2:8765` 走本機，不受防火牆影響，開發和實測用模擬器即可。

## 8. 驗證

**Python 自動測試**（真的在 127.0.0.1 的隨機 port 起伺服器，用 `urllib` 打；機器人用 FakeRobot 或假 Session）
- 配對碼：沒帶、錯的 → 401；錯 10 次 → 429；`/api/ping` 不用；配對碼檔的產生、沿用、`--new-code`、權限 600。
- `/api/state`：欄位齊全；路線只在版本不同時給；指令結束路線清空；✕ 保留 2 秒、下一個指令清掉。
- `/api/events`：`after=0` 拿回全部；長輪詢在新事件時立刻回、逾時回空；超過 300 筆丟最舊的。
- `/api/command`、`/api/go_to`、`/api/survey`、`/api/stop`：轉給 Session 的正確方法；忙碌時 `ok=false`；沒有眼睛時巡邏 `ok=false`；
  兩個執行緒同時送指令只會有一個成功。
- `boot`：每次啟動不同；`/api/hello` 和 `/api/events` 一致。
- `/api/map.png`：大小和翻轉正確；沒有地圖時 404。`/api/camera.mjpg`：第一張是合法 JPEG；沒有相機時不送。
- 任何回應都不含 API key（測試用一組假 key 檢查）。

**Android 單元測試**（JVM，`./gradlew test`）
- `MapGeometry`：來回換算一致；留白處判為範圍外；y 朝上。
- `Model`：解析一份由 Python 測試產生的 `/api/state`、`/api/events` 樣本（放在 test resources，兩邊用同一份）。
- `Mjpeg`：從一段錄好的串流切出正確張數。

**模擬器實測**（開相機、定位模式；Android 模擬器 `Medium_Phone_API_37.0`，app 連 `10.0.2.2:8765`；以 `/odom_gt` 判定）

| # | 動作 | 通過條件 |
|---|---|---|
| 1 | 開 app，輸入配對碼 | 5 秒內出現地圖、狗、相機畫面；地圖上的狗和真值相差 < 0.15 m |
| 2 | 按「廚房」 | 對話出現 `你：去廚房`、工具紀錄、`狗：` 回覆；途中地圖上有綠色路線；到達時真值在廚房地點 0.3 m 內 |
| 3 | 點地圖次臥室附近的空地，按「去」 | 到達時真值在點選位置 0.25 m 內；整段沒有 Gemini 請求 |
| 4 | 點遠處，路線出現 5 秒後按停止 | 出現 `canceled`；回到「待命」；之後 3 秒移動 < 0.1 m |
| 5 | 打錯的配對碼 | 設定頁顯示「配對碼錯誤」；伺服器沒有收到任何指令 |
| 6 | 關掉 `phone`（Ctrl+C）再開 | app 顯示「連不上電腦，重試中…」，伺服器回來後自動接上，對話從頭顯示 |

另外截兩張模擬器畫面（導航途中的地圖、相機大畫面）給使用者看。用使用者自己的手機實測不在驗收範圍內（要先開防火牆），
README 寫怎麼裝 APK、怎麼連。

## 9. 留給實作者的決定

- 顏色、字型大小、按鈕高度、小窗大小：清楚好看即可，截圖確認。
- 地點名稱在小螢幕上重疊時，可以只畫圓點、名稱縮小或省略。
- 長輪詢逾時、MJPEG 張數等數字在合理範圍內可調。
- Gradle / AGP / Compose BOM 的版本：用 Android Studio 新專案範本的預設，能用已安裝的 SDK 建置即可。

## 10. 結果（2026-11-19）

- Python 測試 265 個全部通過（原有 219 個加上新增的 46 個）；Android JVM 單元測試 20 個全部通過。
- 模擬器實測（第 8 節，`--headless`、開相機、定位模式；Android 模擬器 Medium Phone API 37 連 `10.0.2.2:8765`）6/6：
  1. 4.0 秒內出現地圖、狗、相機畫面；地圖上的狗和真值相差 0.020 m。
  2. 按「廚房」：途中收到並畫出路線；到達後真值離廚房地點 0.06 m；`狗：我已經走到廚房了。`
  3. 點 (2.6, 2.0) 按「去」：到達後真值離點的位置 0.05 m；沒有 `你：`／`狗：`（沒有 Gemini 請求）。
  4. 點 (-2.5, -0.3)，路線出現 5 秒後按停止：`canceled`，回到「待命」，之後 3 秒移動 0.010 m。
  5. 打錯配對碼：設定頁顯示「配對碼錯誤」；伺服器沒有收到任何指令；改回正確的配對碼後接回。
  6. 關掉 `phone` 再開：app 顯示「連不上電腦，重試中…」，伺服器回來後自動接上，對話從頭顯示。
- 另外用 `--no-robot` 伺服器在模擬器上試過：設定頁、點地圖確認、鍵盤打開時輸入列在鍵盤上方、地點按鈕經 Gemini 走到廚房。

和設計不同的地方：

- **Android 17 的區域網路權限**：API 37 起，app 沒有 `ACCESS_LOCAL_NETWORK` 權限就連不到私人位址（模擬器上 shell 連得到、
  app 連不到，一律逾時）。app 宣告這個權限並在啟動時詢問；被拒絕時設定頁說明要去哪裡打開。
- **伺服器重開時的事件編號**：伺服器收到比自己最後一筆還大的編號（重開前的編號）時立刻回答，app 看到 `boot` 變了就丟掉這次的回答、
  從 0 重新拿。原本的設計會讓 app 等滿 25 秒，之後還一直用舊編號問。
- **設定頁多一個「取消」**（已經有配對碼時），按 ⚙ 進去之後可以不改設定直接回主畫面。
- **小窗裡的地圖不畫文字**，也不會畫出小窗外。
- **`nav2_robot.py` 修了兩個原本就有的問題**（實測時發現，`gui` 也會遇到）：
  - 導航的計時改用一直在更新的 TF 節點的模擬時鐘。BasicNavigator 的節點只在等東西時才更新時鐘，所以每次啟動後的第一個指令
    會拿到過時的時間（還沒收到 `/clock` 時是 0）；模擬器跑超過 120 秒以後，第一個指令會在 0.1 秒內被判成逾時並取消。
  - 啟動時等 Nav2 改成自己問 `bt_navigator/get_state`，每次最多等 2 秒、沒回答就再問。BasicNavigator 的等法不設時限，
    伺服器重開時第一個回答在 DDS 還沒配對好時遺失，就會永遠停在「等 Nav2 啟動…」。
- `phone` 的終端機也會印出對話（`你：`、`狗：`、工具紀錄），和終端機版相同；port 被占用時印提示並結束。
