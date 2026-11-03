# 計畫 4：導航環境換成 MUJI 公寓 實作計畫

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 Isaac Sim 導航模式的場景換成 `MUJI_Apartment_3D.usd`（含碰撞、燈光），移除方塊房間，並在公寓裡重新達成 M4（SLAM 地圖通過檢查）與 M5（SLAM、AMCL 兩模式 `nav_eval` 5/5）。

**Architecture:** Isaac 端新增 `apartment.py`（常數）與 `world.apartment_world`：以 reference 載入公寓、平移讓客廳中央落在原點、對每個 mesh 加靜態三角網格碰撞與訓練摩擦材質、隱藏天花板。新增 `floorplan.py`（純 numpy，把三角形切成格子圖）與 `tools/make_floorplan.py`（用 `pxr` 讀 USD），產生並提交兩張真值格子圖。ROS 端 `nav_goals` / `map_check` 改成讀真值格子圖，不再依賴 `room_layout.py`。

**Tech Stack:** Isaac Sim 6.2（`pxr`：Usd、UsdGeom、UsdPhysics）、numpy、scipy（`ndimage`）、ROS 2 Jazzy、Nav2、slam_toolbox、nav2_amcl、pytest

**Spec:** `docs/superpowers/specs/2026-11-03-apartment-world-design.md`（前一份 `2026-11-01-robot-dog-isaac-nav2-design.md` 的第 5.5 節由它取代）

## Global Constraints

- 工作區 `~/dog_12dof_ws`（git，`main`）。`DESC` = `~/dog_12dof_ws/robot_dog_description`，`NAV` = `~/dog_12dof_ws/robot_dog_navigation`。
- Isaac Sim：`source /opt/ros/jazzy/setup.bash && cd ~/IsaacLab && OMNI_KIT_ACCEPT_EULA=yes uv run --extra isaacsim python $DESC/isaac/robot_dog_nav_sim.py --headless --no-camera`。同一時間只跑一個 Isaac 程序。
- ROS 端：`source /opt/ros/jazzy/setup.bash && cd ~/dog_12dof_ws && source install/setup.bash`；改了套件後 `colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation`。
- 公寓常數只有一個來源 `DESC/isaac/robot_dog_sim/apartment.py`：`SPAWN_XY = (3.5, 1.8)`、`LIDAR_HEIGHT = 0.31`、`BODY_BAND = (0.03, 0.35)`、`HIDDEN = ('Ceiling_Group',)`。世界座標 = 公寓座標 − `SPAWN_XY`。
- 真值格子圖解析度 0.05 m，格式和 `map_saver` 相同（P5 pgm，trinary：佔據 0、空 254；yaml 的 `origin` 是世界座標）。
- 判定與門檻：目標 0.25 m / 0.3 rad / 120 s（sim time）；`check_map` 對齊率 ≥ 0.95、覆蓋率 ≥ 0.80（達不到時回報實測值給使用者，不自行降低）；平均 sideways ≤ 10%。
- `MUJI_Apartment_3D.usd` 本身不修改；物理與隱藏都寫在 stage 的 override。
- Nav2 參數（MPPI、inflation 0.35、footprint）不改；若門口卡住，依 spec 6.3 一次只改一項。
- 背景程序以 PID 停止，不要用會比對到自己指令列的 `pkill -f`；每次 `nav_eval` 前都重新啟動 Isaac Sim（狗回到原點）。
- `.vscode/settings.json` 是使用者自己 stage 的，不要帶進任何 commit（`git add` 只列本計畫的檔案）。
- 程式註解英文、文件繁體中文；commit 結尾 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。

## Review Focus

1. **出生點卡在幾何裡**：平移方向寫反或 `SPAWN_XY` 落在家具上，狗一出生就和 mesh 重疊而彈飛或跌倒。期待狗在原點站穩（Task 1 smoke：5 s 後機身高度 ≥ 0.2 m、水平位移 < 0.05 m；Task 3 `test_spawn_is_clear_and_reachable`）。
2. **貼圖路徑斷掉**：USD 用相對路徑 `textures/...`，搬檔或安裝到 share 後若少了資料夾，畫面全灰但不報錯。期待每個引用的貼圖都在 USD 旁邊（Task 1 `test_every_texture_next_to_usd`，直接掃 usdc 內的字串，不需要 `pxr`）。
3. **真值和模擬沒對齊**：真值格子圖的 origin 若沒有減掉 `SPAWN_XY`，`map_check` 全部不對齊、目標點淨空也是錯的。期待 yaml origin = −`SPAWN_XY`，且 SLAM 地圖對齊率 ≥ 0.95（Task 2 `test_truth_yaml_origin_is_minus_spawn`；Task 4 `check_map`）。
4. **隱藏的天花板仍有碰撞，或不可見的 mesh 擋路**：期待 `HIDDEN` 底下與不可見的 mesh 都不加碰撞，也不進真值（Task 1 `test_wants_collision`，Task 2 工具用同一個判斷）。
5. **卡在門口或椅腳之間**：0.9 m 門口淨空只比膨脹半徑大 0.05 m，餐桌區只有椅腳。期待 `nav_eval` 全程沒有「Failed to make progress」，沒有恢復行為（Task 5 檢查 launch log 計數為 0；若不為 0，用軌跡找出位置再依 spec 6.3 處理）。

---

### Task 1：公寓檔就位與載入（碰撞、材質、隱藏天花板）

**Files:**
- Move: `~/dog_12dof_ws/MUJI_Apartment_3D.usd` → `DESC/isaac/worlds/MUJI_Apartment_3D.usd`；`~/dog_12dof_ws/textures/` → `DESC/isaac/worlds/textures/`（`git mv` 不適用，未追蹤；用 `mv` 後 `git add`）
- Create: `DESC/isaac/robot_dog_sim/apartment.py`
- Modify: `DESC/isaac/robot_dog_sim/world.py`（新增 `add_static_collision`、`apartment_world`；刪除 `room_world`、`static_cylinder` 與 `room_layout` import）
- Modify: `DESC/isaac/robot_dog_nav_sim.py`（預設 `world.apartment_world`；`--world` 走 `world.usd_world`）
- Test: `DESC/isaac/tests/test_apartment.py`

**Interfaces:**
- Produces（`apartment.py`，純 Python）：
  - `USD: str`：`worlds/MUJI_Apartment_3D.usd` 的絕對路徑（相對於 `apartment.py` 所在目錄的上一層）
  - `SPAWN_XY`、`LIDAR_HEIGHT`、`BODY_BAND`、`HIDDEN`（值見 Global Constraints）
  - `wants_collision(path: str, visible: bool) -> bool`：`visible` 為假或路徑含 `HIDDEN` 任一名稱（以 `/` 分隔的完整段落比對）時為假
  - `texture_paths(usd_path: str) -> set[str]`：掃 usdc 位元組內 `rb'textures/[\w./-]+\.(?:jpg|png)'`，回傳相對路徑集合
- Produces（`world.py`）：
  - `add_static_collision(stage, root_path: str, material) -> int`：對 `root_path` 底下每個 Mesh，若 `wants_collision`，套 `UsdPhysics.CollisionAPI`、`UsdPhysics.MeshCollisionAPI`（`approximation` = `'none'`）、綁定 physics 材質；`HIDDEN` 群組設 `invisible`；回傳加了碰撞的 mesh 數
  - `usd_world(stage, meta, usd_path: str, offset_xy=(0.0, 0.0))`：`apply_physics_settings` → `/World/scene` reference `usd_path`、平移 `(-offset_xy[0], -offset_xy[1], 0)` → `add_static_collision(stage, '/World/scene', policy_ground(stage, meta))` → `add_lights`
  - `apartment_world(stage, meta)`：`usd_world(stage, meta, apartment.USD, apartment.SPAWN_XY)`

- [ ] **Step 1：寫失敗的測試** `DESC/isaac/tests/test_apartment.py`

```python
import os
from robot_dog_sim import apartment


def test_usd_and_constants():
    assert os.path.isfile(apartment.USD) and apartment.USD.endswith('worlds/MUJI_Apartment_3D.usd')
    assert apartment.SPAWN_XY == (3.5, 1.8) and apartment.LIDAR_HEIGHT == 0.31
    assert apartment.BODY_BAND == (0.03, 0.35) and apartment.HIDDEN == ('Ceiling_Group',)


def test_every_texture_next_to_usd():
    paths = apartment.texture_paths(apartment.USD)
    assert len(paths) == 27
    base = os.path.dirname(apartment.USD)
    assert [p for p in paths if not os.path.isfile(os.path.join(base, p))] == []


def test_wants_collision():
    assert apartment.wants_collision('/World/scene/Walls_Group/Wall_01/Wall_01', True)
    assert not apartment.wants_collision('/World/scene/Ceiling_Group/Ceiling/Ceiling', True)
    assert not apartment.wants_collision('/World/scene/Decor_Group/Lamp/Lamp', False)
    assert apartment.wants_collision('/World/scene/Furniture_Group/Ceiling_Group_lookalike/Mesh', True)
```

- [ ] **Step 2：確認失敗**：`cd $DESC/isaac && python3 -m pytest tests/test_apartment.py -q` → `ModuleNotFoundError: robot_dog_sim.apartment`
- [ ] **Step 3：搬檔**：`mkdir -p $DESC/isaac/worlds && mv ~/dog_12dof_ws/MUJI_Apartment_3D.usd ~/dog_12dof_ws/textures $DESC/isaac/worlds/`
- [ ] **Step 4：實作 `apartment.py`**，測試通過：`python3 -m pytest tests -q`（`test_room_layout.py` 仍在，Task 3 才刪）
- [ ] **Step 5：實作 `world.py` 與 `robot_dog_nav_sim.py` 的修改**；`--world` 的說明改成「載入其他 USD（同樣加碰撞與材質，不平移）」。
- [ ] **Step 6：smoke test**：nav sim `--headless --no-camera --max-frames 250`（5 s sim），另一個終端記錄 `/odom_gt`。期待：log 有 `[robot_dog] LiDAR -> /scan  ok`、加了碰撞的 mesh 數 > 200 且印出；最後一則 `/odom_gt` 的 z ≥ 0.2 m、|x|、|y| < 0.05 m；`/scan` 有限值的比例 > 0.9（四周都有牆）。nav sim 要印出 `[robot_dog] apartment: N collision meshes`。
- [ ] **Step 7：量即時率**：headless 有相機與 `--no-camera` 各跑 30 s 牆鐘時間，讀 `/clock` 前後差，算 sim / wall。低於 0.3 時停下來回報（spec 6.1）。
- [ ] **Step 8：Commit**：`git add $DESC/isaac/worlds $DESC/isaac/robot_dog_sim/apartment.py $DESC/isaac/robot_dog_sim/world.py $DESC/isaac/robot_dog_nav_sim.py $DESC/isaac/tests/test_apartment.py`，`git commit -m "Load the MUJI apartment as the navigation world (collision, friction, no ceiling)"`

---

### Task 2：真值格子圖（`floorplan.py` + `make_floorplan.py`）

**Files:**
- Create: `DESC/isaac/robot_dog_sim/floorplan.py`（純 numpy）
- Create: `DESC/isaac/tools/make_floorplan.py`（`pxr`；Isaac 的 Python 執行）
- Create: `NAV/maps/truth/apartment_lidar.{yaml,pgm}`、`NAV/maps/truth/apartment_body.{yaml,pgm}`（工具產生）
- Test: `DESC/isaac/tests/test_floorplan.py`

**Interfaces:**
- Consumes：`apartment.USD`、`SPAWN_XY`、`LIDAR_HEIGHT`、`BODY_BAND`、`wants_collision`（Task 1）
- Produces（`floorplan.py`）：
  - `slice_grid(tris, z: float, bounds, res=0.05) -> np.ndarray[bool]`：`tris` 形狀 (N, 3, 3)、世界座標；`bounds` = `(xmin, ymin, xmax, ymax)`；格子 `[row, col]`，row 0 = 最小 y；三角形和平面 z 的交線段經過的格子為真
  - `band_grid(tris, z0, z1, bounds, step=0.04, res=0.05) -> np.ndarray[bool]`：`z0` 到 `z1`（含）每 `step` 切一刀取聯集
  - `save_map(grid, res, origin_xy, path_no_ext: str)`：寫 `.pgm`（列順序反轉，最上列 = 最大 y）與 `.yaml`（`image`、`mode: trinary`、`resolution`、`origin: [x, y, 0]`、`negate: 0`、`occupied_thresh: 0.65`、`free_thresh: 0.196`）
- Produces（檔案）：`NAV/maps/truth/apartment_lidar.yaml`、`apartment_body.yaml`，origin = `[-3.5, -1.8, 0]`，範圍公寓 x 0～10、y 0～6（200 × 120 格）

- [ ] **Step 1：寫失敗的測試**（合成的 1 × 1 × 1 m 立方體，12 個三角形，放在世界座標 (0.5, 0.5, 0.5)）

```python
def test_slice_cube_outline():           # 在 z=0.3 切：外框 4 條邊上的格子為真，中央為假
def test_slice_above_cube_is_empty():    # z=1.5 → 全部為假
def test_band_includes_low_obstacle():   # 0.05 m 高的薄板：band_grid(0.03, 0.35) 有、slice_grid(0.31) 沒有
def test_save_map_roundtrip(tmp_path):   # save_map 後用相同規則讀回（P5 標頭、列反轉、origin），佔據格位置一致
def test_truth_yaml_origin_is_minus_spawn():  # NAV/maps/truth/*.yaml 的 origin == [-3.5, -1.8, 0]、resolution 0.05
```

（最後一個測試在 Step 5 產生檔案前會失敗；其餘在 Step 3 前失敗。）

- [ ] **Step 2：確認失敗**：`python3 -m pytest tests/test_floorplan.py -q`
- [ ] **Step 3：實作 `floorplan.py`**（向量化：先篩出 z 範圍跨過平面的三角形，再對每條邊求交點；線段以 res/2 步長取樣落格）
- [ ] **Step 4：實作 `make_floorplan.py`**：用 `pxr` 走過 `/World` 下每個 Mesh，`wants_collision(path, visible)` 為假就略過；點座標乘上 world transform，減掉 `SPAWN_XY`；三角化多邊形面；`bounds` = 公寓範圍 − `SPAWN_XY`；輸出兩張圖到 `NAV/maps/truth/`，印出各自的佔據格數。
- [ ] **Step 5：產生並檢查**：`cd ~/IsaacLab && uv run --extra isaacsim python $DESC/isaac/tools/make_floorplan.py`（或 `~/IsaacLab/.venv/bin/python`，只需要 `pxr`）。打開兩張 pgm 對照公寓的俯視圖：牆、三個門口、床、沙發、餐桌椅腳都在；地毯不在；`apartment_body` 有淋浴盤、`apartment_lidar` 沒有。全部測試通過。
- [ ] **Step 6：Commit**：`git commit -m "Truth floor plans of the apartment (LiDAR plane and body band) from its meshes"`

---

### Task 3：ROS 端改用真值（目標點、`map_check`），移除方塊房間

**Files:**
- Modify: `NAV/robot_dog_navigation/nav_goals.py`（新 `GOALS`；`load_truth`、`clearance`、`reachable`；刪除 `room_layout()`）
- Modify: `NAV/robot_dog_navigation/map_check.py`、`NAV/robot_dog_navigation/check_map.py`
- Modify: `NAV/setup.py`（安裝 `maps/truth/*`）、`NAV/package.xml`（`<exec_depend>python3-scipy</exec_depend>`）
- Delete: `DESC/isaac/robot_dog_sim/room_layout.py`、`DESC/isaac/tests/test_room_layout.py`
- Test: `NAV/test/test_nav_goals.py`、`NAV/test/test_map_check.py`（改寫方塊房間相關的測試）

**Interfaces:**
- Consumes：`NAV/maps/truth/apartment_{lidar,body}.yaml`（Task 2）；`map_check.load_map(yaml_path) -> (grid int8, resolution, (ox, oy))`（現有）
- Produces（`nav_goals.py`）：
  - `GOALS`：spec 5.1 的 5 個 `(name, x, y, yaw)`，世界座標：
    `('living room west', -2.5, -0.3, math.pi)`、`('master bedroom', -1.5, 3.0, math.pi / 2)`、`('second bedroom', 3.0, 2.2, 0.0)`、`('dining and kitchen east', 3.5, -0.3, -math.pi / 2)`、`('back to the start', 0.0, 0.0, 0.0)`
  - `load_truth(name: str) -> (grid, res, origin)`：在 repo 的 `NAV/maps/truth/` 或 package share 的 `maps/truth/` 找 `<name>.yaml`
  - `clearance(truth, x, y) -> float`：(x, y) 到最近佔據格中心的距離（m）；佔據格上為 0
  - `reachable(truth, x, y) -> bool`：從原點所在格出發、經 4 鄰接的空格（`grid == 0`）能否到 (x, y) 所在格（`scipy.ndimage.label`）
- Produces（`map_check.py`）：
  - `check_map(points, truth_lidar, truth_body, tol=0.10, near=0.15, visible=0.10) -> {'aligned': float, 'coverage': float}`（定義見 spec 4.3；`truth_*` 是 `load_map` 的三元組）
- Produces（`check_map.py` CLI）：印 `[MAP] N occupied cells, aligned A (>= 0.95), coverage C (>= 0.80): PASS|FAIL`

- [ ] **Step 1：改寫測試（先失敗）**

```python
# test_nav_goals.py
def test_goals_are_clear_and_reachable():     # 每個目標 clearance(body) >= 0.5 且 reachable(body)
def test_spawn_is_clear_and_reachable():      # (0, 0)：clearance >= 0.5
def test_goals_cover_three_areas():           # 公寓座標 = 世界座標 − 真值 yaml 的 origin（不另存 SPAWN_XY）：y > 3 且 x < 5 → 主臥；y > 3 且 x >= 5 → 次臥；y < 3 → 客餐廚；三區都有
def test_bed_centre_is_unreachable():         # (0.4, 2.7) 不 reachable 或 clearance < 0.16
# 保留：judge、pose_error、sideways_share 的測試

# test_map_check.py（合成 2 m × 2 m 的方形房間真值，0.05 m；中間一個 0.4 m 方塊）
def test_truth_itself_passes():               # 以真值 lidar 的佔據格當地圖 → aligned > 0.99、coverage > 0.95
def test_shifted_map_is_not_aligned():        # 平移 0.3 m → aligned < 0.5
def test_missing_wall_lowers_coverage():      # 拿掉北牆 → 0.6 < coverage < 0.9
def test_hidden_surfaces_do_not_count():      # 方塊內部被填滿的格子不在地圖上也不影響 coverage
# 保留：test_load_map_values_and_row_order
```

- [ ] **Step 2：確認失敗**：`cd $NAV && python3 -m pytest test -q`
- [ ] **Step 3：實作 `nav_goals.py`、`map_check.py`、`check_map.py`、`setup.py`、`package.xml`**；刪除 `room_layout.py`、`test_room_layout.py`。`grep -rn room_layout $DESC $NAV --include=*.py` 沒有結果。
- [ ] **Step 4：全部測試通過**：`cd $DESC/isaac && python3 -m pytest tests -q`；`cd $NAV && python3 -m pytest test -q`
- [ ] **Step 5：Commit**：`git commit -m "nav_goals and map_check read the apartment's truth floor plans; remove the box room"`

---

### Task 4：M4 公寓建圖

**Files:**
- Modify: `NAV/launch/nav.launch.py`（`map` 預設 `maps/apartment.yaml`；docstring）
- Create: `NAV/maps/apartment.{yaml,pgm}`（`map_saver_cli` 產生）
- Delete: `NAV/maps/robot_dog_room.{yaml,pgm}`

**Interfaces:**
- Consumes：`apartment_world`（Task 1）、`GOALS`（Task 3）、`check_map` CLI（Task 3）

- [ ] **Step 1：build**：`colcon build --symlink-install --packages-select robot_dog_description robot_dog_navigation`
- [ ] **Step 2：SLAM 繞一圈**：nav sim（重新啟動）+ `nav.launch.py slam:=True rviz:=false` + `ros2 run robot_dog_navigation nav_eval --slam`；同時用軌跡記錄（`/odom_gt` 位置、速度、`/cmd_vel`，5 Hz）。期待 `[NAV] PASS 5/5`（這一輪也是 M5 的 SLAM 結果）、平均 sideways ≤ 10%、launch log 中「Failed to make progress」0 次。
- [ ] **Step 3：存圖**：同一個 stack 還在時 `ros2 run nav2_map_server map_saver_cli -f $NAV/maps/apartment`（不要加 `use_sim_time`）。
- [ ] **Step 4：檢查**：`ros2 run robot_dog_navigation check_map $NAV/maps/apartment.yaml` → PASS。覆蓋率 < 0.80 時：列出沒被覆蓋的真值表面位置（例如被床擋住的牆角），回報給使用者，不改門檻。
- [ ] **Step 5：刪除 `robot_dog_room.*`**，改 `nav.launch.py`，重新 build。
- [ ] **Step 6：Commit**：`git commit -m "M4 in the apartment: SLAM map passes check_map"`

---

### Task 5：M5 公寓導航與文件

**Files:**
- Modify: `NAV/README.md`、`DESC/README.md`（導航模式一節：公寓、即時率、檔案位置、`make_floorplan`）
- Modify: `docs/superpowers/specs/2026-11-01-robot-dog-isaac-nav2-design.md`（第 5.5 節指向新 spec；6.3 / 6.5 / 7 的地圖名稱與場地；「實作分期」加計畫 4 結果）
- Modify: `docs/superpowers/specs/2026-11-03-apartment-world-design.md`（加「結果」一節）

- [ ] **Step 1：AMCL 模式**：nav sim（重新啟動）+ `nav.launch.py slam:=False rviz:=false` + `nav_eval --extra-goal=0.4,2.7,0.0`。期待初始定位誤差 < 0.1 m、`[NAV] PASS 5/5`、extra goal 判 FAIL 且程式正常結束、「Failed to make progress」0 次、平均 sideways ≤ 10%。
- [ ] **Step 2：不通過時**：用軌跡找出停住的位置，依序檢查 (1) 門口：門板與門框在 costmap 上的寬度、`inflation_radius`、`cost_scaling_factor`；(2) 餐桌椅腳；(3) 定位誤差（每個目標的 localization error）。每次只改一項、重跑；三輪仍不過就停下來回報數據。
- [ ] **Step 3：再跑一次 SLAM 模式**（若 Task 4 之後改過參數）確認仍 5/5。
- [ ] **Step 4：文件**：README 的使用說明、地圖與 `check_map` 新定義、結果表（兩模式的終點距離、定位誤差、sideways、即時率）、浴室不放目標與淋浴盤的提醒、到不了的目標範例。
- [ ] **Step 5：Commit**：`git commit -m "M5 in the apartment: Nav2 reaches 5/5 goals in SLAM and AMCL modes"`

---

## 計畫 4 完成條件（= spec 第 1 節成功條件）

- `python3 -m pytest $DESC/isaac/tests $NAV/test -q` 全部通過；`grep -rn room_layout` 在程式碼中沒有結果。
- `NAV/maps/apartment.*` 與 `NAV/maps/truth/*` 已提交，`check_map` PASS。
- `nav_eval --slam` 與 `nav_eval`（AMCL）都 `[NAV] PASS 5/5`，沒有跌倒、沒有「Failed to make progress」，平均 sideways ≤ 10%；床中央的目標判 FAIL 且程式正常結束。
- README 與兩份 spec 更新，即時率已記錄。
