# auto

比賽自動化程式：照簡章跑「帶位 → 固定餐 → 加點餐」三趟，一次可以連跑兩個回合。

有兩個版本，流程和操作方式完全一樣：

| | 單檔版 `run_mission.py` | 套件版 `ros2 run auto mission` |
|---|---|---|
| 檔案 | 只有一個 py 檔，不用 `colcon build` | `auto/` 底下多個模組 + `config/mission.yaml` |
| 7 個點放哪 | 寫在檔案最上面的「設定」區 | 讀 `car_waypoints` 的 `waypoints.yaml` |
| 依賴 | 只要 ROS 和 `nav2_simple_commander` | 另外依賴 `car_waypoints` 套件 |

## 單檔版（run_mission.py）

打開 [run_mission.py](run_mission.py)，最上面的「設定」區填 7 個點，每個點是 `(x, y, yaw)`：x、y 單位公尺，yaw 單位**度**。
還沒量的點留 `None`，程式會拒絕啟動並告訴你缺哪個。

```python
START = (-0.034, 0.004, 0.8)     # 出發區
PREP = (-0.004, -0.563, 0.1)     # 備餐區
TABLES = {1: (1.203, 0.634, -0.3), 2: None, 3: None, 4: None, 5: None}
STOP_SECONDS = 3.0               # 到桌後靜止秒數
```

量座標：用 `ros2 run car_waypoints wp_record` 把車開到定位後存一個名稱，再輸入 `list`，
它會列出 `x=... y=... yaw=...°`，照抄進去就好（yaw 已經是度）。

不必 7 個點都填：只用得到填好的點，沒填的（`None`）程式會明確說「還沒設定座標」，不會出發。

執行（開機順序見下一節）：

```bash
source /opt/ros/humble/setup.bash
python3 ~/mecanum_ros2/src/auto/run_mission.py
```

啟動後先列出「可用的點」和「還沒設定座標的點」，然後問你要去哪些點：

| 輸入 | 做什麼 |
|---|---|
| `start t1 prep` | **自訂路線**：依序前往這幾個點。每個點出發前都停下來等你按 Enter，跑完回到這個提問，可以再輸入下一條路線。點名稱不分大小寫、可以重複（例如 `t3 prep t3`） |
| 直接按 Enter | 跑比賽流程（下面「操作流程」那三趟，兩個回合）。選好桌號後，**會先提示「按 Enter 先回 start」**，車子回到 `start` 才開始第一趟（剛跑完自訂路線時車子可能不在 start）。只能選已經填好座標的桌子；另一區的桌子都沒填時，不跑第二回合 |
| `q` | 結束 |

自訂路線打錯名字、或點沒有座標，會當場說明並重問，車子不會動。
導航失敗或 Ctrl+C 取消時，一樣用 `r`、`s`、`q` 選擇。

兩個版本不要同時開。以下「開機順序」「操作流程」「導航失敗時」對兩個版本都適用；「設定」「程式結構」是套件版的說明。

## 套件版
7 個點（出發區、備餐區、1~5 號桌）用 `car_waypoints` 的 `wp_record` 記錄。

## 開機順序

和 `wp_go` 一樣：先開馬達 + 雷達、再開 Nav2 並完成定位，最後才執行這個程式。

```bash
ros2 launch car_bringup odombase2_lidar.launch.py lidar_z:=0.23   # 終端 1
ros2 launch car_navigation2 navigation2.launch.py                  # 終端 2，RViz 設好初始位置
ros2 run auto mission                                              # 終端 3
```

## 先記好 7 個點

```bash
ros2 run car_waypoints wp_record
```

名稱預設是 `start`、`prep`、`t1`、`t2`、`t3`、`t4`、`t5`。程式啟動時會檢查，缺哪個就列出來然後結束，不會動車子。

## 操作流程

啟動後先問第一回合抽到幾桌，然後：

| 趟次 | 程式做的事 | 你要做的事 |
|---|---|---|
| 1 帶位 | 等 Enter → 開到指定桌 → 靜止 3 秒 | 裁判給點餐卡後，輸入點餐內容（可略過）→ 車子回 `prep` |
| 2 固定餐 | 等 Enter → 開到指定桌 → 靜止 3 秒 | 備餐區放好餐點後按 Enter；到桌後輸入加點項目（可略過）→ 車子回 `prep` |
| 3 加點餐 | 等 Enter → 開到指定桌 → 靜止 3 秒 | 備餐區放好加點餐點後按 Enter；到桌後按 Enter（不須加點卡）→ 車子回 `start` |

- 第一回合結束後問第二回合桌號：只能抽另一區（前區 1、4／後區 2、3、5），抽同區會被擋下。直接按 Enter 就結束，不跑第二回合。
- **每次出發都要你按 Enter**，簡章不允許用等待時間當出發指令。
- 沒有鏡頭，所以點餐卡、加點卡由你手動輸入。
- 任何提示輸入 `q`（或 Ctrl+D）結束，車子停在原地。

## 導航失敗時

某一步沒成功（Nav2 失敗、被 Ctrl+C 取消、尚未定位）會停下來問：

- `r` 重試同一步
- `s` 跳過（你已經手動把車放到位，之後的流程照走，包括靜止 3 秒）
- `q` 結束

每一段都會印出花的秒數；超過簡章的單趟上限（90 秒，有障礙物 120 秒）會標警告。

## 設定

`config/mission.yaml`，每個欄位都可以省略。改完直接重跑，不用重新 build：

```yaml
waypoint_file: ~/mecanum_ros2/src/car_waypoints/config/waypoints.yaml
stop_seconds: 3.0            # 到桌後靜止的秒數
points:
  start: start               # 7 個點在點位檔裡的名稱，可以換成自己的
  prep: prep
  tables: [t1, t2, t3, t4, t5]
```

打錯欄位名稱、桌子不是 5 個、兩個點同名，都會明確報錯，不會悄悄忽略。
換設定檔：`ros2 run auto mission --ros-args -p mission_config:=/path/to/mission.yaml`

## 程式結構

- `auto/flow.py`：三趟流程，導航、輸入、輸出、等待都是注入的，不依賴 ROS，換成鏡頭掃 QR 時只要換輸入來源。
- `auto/settings.py`：讀 `mission.yaml`、檢查點位是否齊全。
- `auto/mission.py`：接上 `car_waypoints` 的導航節點。

## 測試

```bash
cd ~/mecanum_ros2/src/auto && python3 -m pytest
```

`test_mission_ros.py` 用假的 TF 和假的 NavigateToPose server，只在本機（domain 89），不會影響實車上跑的節點。

## 送餐 GUI 與加點 QR 掃描

在 Jetson 的圖形桌面開啟終端，先依照上面的開機順序啟動車子和 Nav2，並啟動 USB 相機，確認影像發布到 `/image_raw`。首次使用或修改掃描節點後編譯：

```bash
cd ~/mecanum_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select zbar_opencv
source install/setup.bash
ros2 run zbar_opencv zbar_opencv
```

另一個桌面終端啟動介面：

```bash
source /opt/ros/humble/setup.bash
python3 ~/mecanum_ros2/src/auto/mission_gui.py
```

介面沿用 `run_mission.py` 的座標及比賽流程。以介面按鈕選擇桌號及確認每次出發；在第二趟到桌停穩後，將加點卡 QR Code 對準相機，收到 `/zbar_opencv_code` 的非空文字後自動填入加點餐點並繼續回備餐區。QR 內容原文及即時相機畫面顯示在介面，也可以按「略過加點」。請確認掃描內容與餐點相符。介面關閉需在等待操作時確認，導航中請先等導航完成。

GUI 需要 Jetson 桌面與 Python tkinter，SSH 沒有圖形顯示時請在 Jetson 本機桌面啟動。不要同時啟動其他送餐任務。
