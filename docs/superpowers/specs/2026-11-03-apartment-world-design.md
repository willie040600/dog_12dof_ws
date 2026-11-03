# 導航環境換成 MUJI 公寓 — 設計

日期：2026-11-03
前一份 spec：`2026-11-01-robot-dog-isaac-nav2-design.md`（本文件取代其中第 5.5 節「世界」，並改寫 M4 / M5 的場地；
其餘架構、話題、TF、Nav2 參數不變）。

## 1. 目標與範圍

把 Isaac Sim 導航模式的場景從程式產生的 10 × 10 m 方塊房間，換成使用者提供的 `MUJI_Apartment_3D.usd`，
並在公寓裡重新達成 M4（建圖）與 M5（Nav2 導航）。

使用者的決定：
- 舊的方塊房間**整個移除**（程式、地圖、測試），公寓是唯一的導航場景。
- 範圍是**完整導航**：載入（含碰撞）、新的 SLAM 地圖、地圖檢查、新的 5 個目標點，SLAM 與 AMCL 兩模式都 5/5。
- 檔案放哪裡由我決定。
- 三個設計選擇都照推薦（下面第 3～5 節）。

### 成功條件

1. 純 Python 測試全部通過（`robot_dog_description/isaac/tests`、`robot_dog_navigation/test`）。
2. `maps/apartment.{yaml,pgm}` 已提交，`check_map` 通過（對齊率 ≥ 0.95、覆蓋率 ≥ 0.80，見 4.3）。
3. `nav_eval --slam` 與 `nav_eval`（AMCL）都 `[NAV] PASS 5/5`，全程沒有跌倒；平均 sideways 在 10% 以下
   （方塊房間目前約 3%）。
4. 到不了的目標（床的中央）被判 FAIL，`nav_eval` 正常結束。
5. README 與 spec 更新：公寓的使用方式、結果、即時率。

### 非目標

- 浴室不放目標（見 6.2）；門板固定在開啟的位置，不能開關；不做動態物件。
- 不修改 `MUJI_Apartment_3D.usd` 本身。

## 2. 公寓檔（實測）

| 項目 | 值 |
|---|---|
| 座標 | Z 朝上、公尺；`defaultPrim` `/World`；範圍 x 0～10、y 0～6、z −0.1～2.5 |
| 群組 | `Walls`、`Floors`、`Openings`（門窗框、玻璃）、`Furniture`、`Kitchen`、`Bath`、`Decor`、`Details`、`Ceiling`、`_materials` |
| 規模 | 271 個 Mesh、約 4.5 萬個三角形、22 個材質 |
| 物理 / 燈光 | **都沒有**（沒有 CollisionAPI、沒有任何 Light） |
| 地板 | 四塊地板頂面都在 z = 0，房間之間沒有高低差；唯一凸起是淋浴盤 5 cm |
| 天花板 | z 2.4～2.5 m |
| 門口 | 玄關 0.9 m（南牆 x 1.0～1.9）、主臥 0.9 m（x 1.6～2.5, y 3）、次臥 0.9 m（x 6.5～7.4, y 3）、浴室 0.8 m（x 7.8, y 0.6～1.4）；門板固定成開啟 90° |
| 貼圖 | 27 張 jpg（44 MB），以相對路徑 `textures/...` 引用，全部存在、沒有多餘 |

格局（公寓座標）：北半部是主臥（x 0～5）與次臥（x 5～10），南半部是客餐廚（LDK），浴室在東南角
（x 7.8～10, y 0～2.1）。

## 3. 檔案位置與載入

### 3.1 檔案

```
robot_dog_description/isaac/worlds/
├── MUJI_Apartment_3D.usd
└── textures/            27 張貼圖（USD 以相對路徑引用，必須和 USD 放在一起）
```

`isaac/` 已由 CMake 安裝到 share，`worlds/` 跟著安裝。檔案從工作區根目錄搬過來，以 git 一般檔案提交
（目前沒有遠端；之後要推到 GitHub 時可以再考慮 Git LFS）。

### 3.2 `robot_dog_sim/apartment.py`（純 Python，常數單一來源）

| 名稱 | 值 | 用途 |
|---|---|---|
| `USD` | `worlds/MUJI_Apartment_3D.usd` 的絕對路徑 | 載入 |
| `SPAWN_XY` | `(3.5, 1.8)` | 公寓座標中的出生點（客廳中央、沙發與電視櫃之間）；載入時整個公寓平移 `−SPAWN_XY` |
| `LIDAR_HEIGHT` | 0.31 m | 站姿機身高 0.226 m + `laser_joint` 0.085 m |
| `BODY_BAND` | 0.03～0.35 m | 會擋到狗身體的高度範圍 |
| `HIDDEN` | `('Ceiling_Group',)` | 載入時隱藏 |

### 3.3 `world.apartment_world(stage, meta)`

1. `apply_physics_settings(stage, meta)`（和訓練相同的 dt、重力、PhysX 設定）。
2. `/World/scene` 以 reference 載入 `USD`，加平移 `(−SPAWN_XY, 0)`。**出生點因此是世界原點**：
   狗在原點、面向 +x 出生；`odom`、SLAM 地圖原點、AMCL 初始位置（原點）、`nav_eval` 全部沿用，不用改。
3. 對每個可見的 Mesh：套 `CollisionAPI` 與 `MeshCollisionAPI(approximation='none')`（靜態三角網格），
   並綁定 `policy_ground(stage, meta)` 的摩擦材質（和訓練相同的摩擦與 combine mode）。全部以 stage 上的 override
   寫入，USD 檔不變。
4. `HIDDEN` 的群組設為 invisible，且不加碰撞。
5. `add_lights(stage)`（現有的日光 + 天空光；天花板隱藏後光線照得進屋內）。

`robot_dog_nav_sim.py`：預設呼叫 `apartment_world`；`--world <USD>` 載入其他場景時走同一套流程
（同樣加碰撞與材質，但平移為 0）。`world.room_world` 與 `room_layout.py` 刪除。

## 4. 真值平面圖與地圖檢查

### 4.1 `isaac/tools/make_floorplan.py`

用 Isaac Lab 的 Python 執行（需要 `pxr`）。讀 USD，略過 `HIDDEN` 與不可見的 Mesh，把三角形和水平面求交，
轉成 0.05 m 的格子圖，座標已平移到世界座標（出生點 = 原點）：

| 輸出 | 切法 | 用途 |
|---|---|---|
| `apartment_lidar` | 在 `LIDAR_HEIGHT` 切一刀 | `map_check` 的真值 |
| `apartment_body` | `BODY_BAND` 內每 4 cm 切一刀後取聯集 | 目標點與出生點的淨空檢查 |

格式和 `map_saver` 相同（`.pgm` + `.yaml`，trinary，佔據 = 0、空 = 254），存到
`robot_dog_navigation/maps/truth/` 並提交。公寓檔有改就重跑一次：

```bash
cd ~/IsaacLab && uv run --extra isaacsim python \
    ~/dog_12dof_ws/robot_dog_description/isaac/tools/make_floorplan.py
```

### 4.2 `nav_goals` 讀真值（不再 import robot_dog_description）

`nav_goals.load_truth(name)` 用 `map_check.load_map` 讀 `maps/truth/<name>.yaml`，從 repo 或 package share 找。
`clearance(grid, x, y)`：(x, y) 到最近佔據格的距離。`reachable(grid, x, y)`：從原點出發、經過空格能不能走到
(x, y)。這取代原本以檔案路徑找 `room_layout.py` 的做法（最終審查第 6 點）。

### 4.3 `map_check`

- **對齊率**：SLAM 地圖的佔據格中，離真值（`apartment_lidar`）佔據格 0.10 m 以內的比例，門檻 ≥ 0.95（不變）。
- **覆蓋率**：「看得見的真值表面」中，0.15 m 內有 SLAM 佔據格的比例，門檻 ≥ 0.80。
  看得見的真值表面 = `apartment_lidar` 的佔據格中，0.10 m 內有「從原點走得到的空格」（依 `apartment_body`）的那些。
  牆的背面、家具內部、浴室裡被門板擋住的角落都不算。
- 方塊房間專用的 `outer_wall_coverage` 刪除。
- 門檻 0.80 若在實際繞完一圈後因為被家具遮住的表面而達不到，**回報實測值並和使用者討論**，不自行降低。

## 5. 目標點與 `nav_eval`

### 5.1 目標點（世界座標 = 公寓座標 − (3.5, 1.8)）

| # | 名稱 | 公寓座標 | 世界座標 (x, y, yaw) | 淨空（`apartment_body`） |
|---|---|---|---|---|
| 1 | living room west | (1.0, 1.5) | (−2.5, −0.3, π) | 0.85 m |
| 2 | master bedroom | (2.0, 4.8) | (−1.5, 3.0, π/2) | 0.94 m（經過主臥門） |
| 3 | second bedroom | (6.5, 4.0) | (3.0, 2.2, 0) | 0.87 m（經過次臥門） |
| 4 | dining and kitchen east | (7.0, 1.5) | (3.5, −0.3, −π/2) | 0.75 m |
| 5 | back to the start | (3.5, 1.8) | (0.0, 0.0, 0) | 0.70 m |

1→2→3 會先進主臥、再回客廳、再進次臥，兩個臥室門都要穿過兩次。

### 5.2 判定與到不了的目標

判定不變：Nav2 SUCCEEDED、真值距離 ≤ 0.25 m、yaw ≤ 0.3 rad、120 s（sim time）內、沒有跌倒。
README 的到不了目標範例改成主臥床的中央：`--extra-goal=0.4,2.7,0.0`（公寓 (3.9, 4.5)）。

### 5.3 測試（純 Python）

- 5 個目標與出生點在 `apartment_body` 上淨空 ≥ 0.5 m，且從原點走得到。
- 目標涵蓋主臥、次臥、客餐廚三個區域（以公寓座標判斷）。
- 床中央的到不了目標在 `apartment_body` 上走不到或淨空 < 0.16 m。
- `map_check`：用合成的小格子圖測對齊率與覆蓋率（取代原本以方塊房間為準的測試）。

## 6. 風險

### 6.1 即時率

碰撞從 13 個方塊變成約 4.5 萬個三角形，貼圖也變多，即時率可能比 0.58（方塊房間、`--no-camera`）低。
所有 ROS 節點用 sim time，所以只影響牆鐘時間。實測後記到 README；若低於 0.3 再討論（例如只對牆、地板、
家具加碰撞，略過裝飾小物）。

### 6.2 LiDAR 看不到的東西

- 淋浴盤（5 cm）比 LiDAR 平面低，地圖上沒有；浴室最寬處淨空只有 0.40 m、門口 0.35 m，和膨脹半徑 0.35 m 差不多，
  所以浴室不放目標。用 RViz 手動把狗送進浴室可能會被淋浴盤絆到，README 註明。
- 餐桌與椅子在 LiDAR 平面只剩桌腳、椅腳（間距約 0.4 m），膨脹會把縫補起來；實測時特別看這一區有沒有
  路徑穿過椅子底下。
- 床、沙發、櫃子在 LiDAR 平面有側面，地圖上會是實心外框。

### 6.3 門口

0.9 m 的門口中心淨空 0.40 m，比膨脹半徑 0.35 m 稍大，Nav2 過得去但代價高。MPPI 參數不改；
若實測卡在門口，依序檢查：門板與門框的 costmap、`inflation_radius`、`cost_scaling_factor`，每次只改一項。

## 7. 要改的檔案

| 檔案 | 動作 |
|---|---|
| `isaac/worlds/MUJI_Apartment_3D.usd`、`isaac/worlds/textures/` | 從工作區根目錄搬入 |
| `isaac/robot_dog_sim/apartment.py` | 新增（常數） |
| `isaac/robot_dog_sim/world.py` | 新增 `apartment_world`、共用的加碰撞函式；刪除 `room_world` 與 `room_layout` import |
| `isaac/robot_dog_sim/room_layout.py`、`isaac/tests/test_room_layout.py` | 刪除 |
| `isaac/robot_dog_nav_sim.py` | 預設載入公寓；`--world` 走同一套流程 |
| `isaac/tools/make_floorplan.py` | 新增 |
| `robot_dog_navigation/maps/truth/apartment_{lidar,body}.{yaml,pgm}` | 新增（工具產生） |
| `robot_dog_navigation/maps/apartment.{yaml,pgm}` | 新增（SLAM 繞完存的地圖）；刪除 `robot_dog_room.*` |
| `robot_dog_navigation/robot_dog_navigation/nav_goals.py` | 新目標點、`load_truth`、`clearance`、`reachable`；刪除 `room_layout()` |
| `robot_dog_navigation/robot_dog_navigation/map_check.py`、`check_map.py` | 改成和真值格子圖比 |
| `robot_dog_navigation/launch/nav.launch.py` | 預設地圖改成 `maps/apartment.yaml` |
| `robot_dog_navigation/setup.py` | 安裝 `maps/truth/` |
| 測試、兩份 README、前一份 spec 第 5.5 / 6.3 / 6.5 / 7 節 | 更新 |

## 8. 結果（2026-11-03，計畫 4 結案時記錄）

v8 策略，RTX 5070 Ti 筆電，headless、`--no-camera`。即時率：`--no-camera` 0.61、有相機 0.53（方塊房間 0.58 / 0.53），
碰撞從方塊換成約 4.5 萬個三角形沒有變慢。

| 項目 | 結果 |
|---|---|
| 載入 | 270 個 mesh 加碰撞；狗在原點站穩（機身 0.238 m），`/scan` 全部打到東西 |
| 真值 | `apartment_lidar` 2618 格、`apartment_body` 3201 格（200 × 120） |
| M4 地圖 | 1878 格，對齊率 0.998、覆蓋率 0.825；分區和真值的平均偏移 1～2 cm |
| M5 SLAM | PASS 5/5（終點 0.03～0.10 m，sideways 6%）；見下面「已知問題」 |
| M5 AMCL | PASS 5/5 連續兩輪（終點 0.03～0.10 m、定位誤差 ≤ 0.08 m、初始 0.022 m、sideways 4%），沒有「Failed to make progress」 |
| 到不了的目標 | 主臥床中央：Nav2 約 40 s 回報 FAILED，判 FAIL，程式正常結束 |

實作中發現、spec 沒預料到的四件事（每件都改一項、重跑驗證）：

1. **地毯的邊緣是台階**：客廳地毯 8 mm 高、有碰撞，平地訓練的策略爬不上去，而它比 LiDAR 與兩個真值切面都低，
   Nav2 看不到。貼在地上、頂部 ≤ 2 cm 的 mesh（兩塊地毯、立燈底座）不加碰撞（`apartment.FLAT_TOP`）。
2. **原地轉向的死區**：站著時策略不理會 |wz| < 約 0.15 rad/s（指令 0.03～0.13 實際 0.01），Nav2 修最後一點朝向時
   正好送約 0.1 rad/s，狗會停在目標上約 37 s。`CommandBuffer` 在原地轉向時把 0.03～0.2 提高到 0.2。
3. **AMCL 運動雜訊**：`/odom` 是模擬真值，預設 `alpha` 0.2 讓 AMCL 在主臥偏 0.32～0.36 m（地圖本身只偏 1～2 cm），
   兩輪都因此失敗；改 0.05 後 ≤ 0.08 m。
4. **膨脹衰減**：門口每一點離門框都在 0.40 m 內，`cost_scaling_factor` 從 3 改成 10（MPPI 建議值）。它不是下面
   門口問題的解法，但之後的 AMCL 與 SLAM 輪次都沒有問題，所以保留。

### 已知問題：SLAM 模式在主臥門口

從主臥去次臥時，兩間臥室之間的牆被床擋住、LiDAR 還沒看到，NavFn（`allow_unknown: true`）規劃穿過那片未知區域往東；
狗走到東側門框前看到牆，路線翻轉成穿過門口，MPPI 在門框前來回擺（記錄的 `/plan` 證實）。5 輪 SLAM 中 2 輪在這裡
逾時，其餘 10～40 s 脫困。AMCL 模式用完整地圖，沒有這個問題。可能的改法（未做）：SLAM 模式的全域規劃器不穿過
未知區域（第一次進次臥時就會規劃不出路）、把 SLAM 繞行改成先進次臥再進主臥、或讓 BT 在路線大幅翻轉時先原地轉向。
