# Ros2_tirt

ROS 2 Humble 麥克納姆輪送餐專案，2026-11-06 實機來源快照。包含 10 個 ROS 套件、最新地圖／點位、Cartographer 建圖入口、終端機及淺色數字鍵 GUI、A/B 必經路線、相機／QR、底盤／EKF／光達與車身遮蔽。

## 安裝與編譯

目標為 Ubuntu 22.04 arm64、ROS 2 Humble。內附 tf2 0.25.24 核心庫為 arm64，來源與版權見 `tf2_0.25.24/COPYRIGHT`；不適用於 x86。導航啟動僅在子程序使用此修正庫，不覆蓋系統安裝。

```bash
git clone https://github.com/keys-lll/Ros2_tirt.git "$HOME/Ros2_tirt"
cd "$HOME/Ros2_tirt"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
bash scripts/build.sh
```

需預先安裝 colcon、rosdep 並完成 rosdep 初始化。Ubuntu 桌面視窗排列另外使用 `gnome-terminal`、`python3-gi`、`gir1.2-wnck-3.0`；建圖另外需要 `ros-humble-cartographer-ros`。請使用英文工作區路徑，Humble rosidl 在中文父目錄可能產生介面路徑錯誤。

## 一條指令：導航＋GUI

先停止其他導航與任務，將車放在記錄的 start 位置及朝向。在儲存庫根目錄執行：

```bash
bash diagnostics/start_competition.sh gui
```

導航建立本次日誌且 Nav2 lifecycle 就緒後，才啟動 GUI。預設不開 RViz；GUI 置中、按鈕直排在右側。X11 桌面會自動把焦點與滑鼠移至 GUI。GUI 與終端機任務共用互斥鎖，避免同時控制車輛。

| 數字鍵 | 操作 |
|---|---|
| 1～5 | 選擇目前可用桌號 |
| 6 | 確認／繼續／手動放行／重試；任務結束後重新開始 |
| 7 | 一般導航失敗時，確認已手動到位；A/B 失敗不提供跳過 |
| 8 | 結束第二回合 |
| 0 | 中止；任務結束後關閉 GUI |

數字列與數字小鍵盤皆可使用，所有按鈕均標示對應鍵位。相機隨任務程式啟動，持續到程式關閉；GUI 畫面相對收到的影像旋轉 180°。只有點餐／加點提示期間接受新 QR，GUI 按 6 可手動放行。

## 終端機與分開啟動

```bash
# 一條指令啟動導航＋終端機任務
bash diagnostics/start_competition.sh

# 或在兩個終端機分開執行
bash scripts/run.sh nav
bash scripts/run.sh gui      # GUI
# bash scripts/run.sh mission  # 終端機任務，與 GUI 擇一
```

終端機仍用桌號、Enter 繼續及 q 中止。`scripts/run.sh` 與名稱含 10hz 的 diagnostics 腳本都是啟動／記錄入口，不會強制覆寫 controller 頻率；以當前 YAML 為準。

## 比賽流程

導航啟動以 start 初始化定位；任務選桌後直接出發，不新增「先回 start」。

- 進備餐區：目前位置 → b → a → prep。
- 離開備餐區：prep → a → b → 目的地。
- A/B 使用專用位置判定（3 cm、角度 pi），不要求車頭對正、不等人工輸入；逐點導航切換仍可能短暫減速。各段仍由 Nav2 避障規劃，未強制直線穿越。
- 第一趟桌邊 `none` 仍去 prep；第二趟桌邊 `none` 返回 start 後立即重新選桌，程式保留。
- prep 要人工確認才出發（GUI 6／終端機 Enter），沒有 6 秒自動出發。
- 加點提示直接放行表示不加點，回 start；加點餐送完的最後桌邊步驟只需確認回 start。
- t4 外側 35 cm 預備點已移除；表內 `t4` 為直接停靠點。

## 建圖與存點

```bash
# 終端機 1：Cartographer，預設開建圖 RViz（競賽導航預設不開）
bash scripts/mapping.sh

# 終端機 2：將車開到位置後輸入名稱存點
bash diagnostics/record_mapping_points.sh
```

| 點名 | 位置 |
|---|---|
| start | 起始點 |
| prep | 備餐區停靠點 |
| t1～t5 | 各桌停靠點，逐一記錄 |
| a / A | 備餐區前方，統一存為 a |
| b / B | 5 號桌旁，統一存為 b |

A/B 記錄時不必刻意對車頭；檔案仍保留當時 yaw，任務的 A/B 專用控制器不要求到點方向。輸入 `list` 檢查，重輸同名點會覆寫，`q` 離開。點位在 `src/car_waypoints/config/waypoints.yaml`，單位 m/rad；存點與導航必須使用同一地圖座標系。建圖提供 map→base_link TF 後才能存點。

儲存地圖（會更新本儲存庫 maps 下的地圖）：

```bash
source scripts/env.sh
ros2 run nav2_map_server map_saver_cli -f "$TIRT_WORKSPACE/maps/mecanum_map"
```

## 當前參數快照

| 參數 | 值 |
|---|---|
| 平移速度上限 | 0.25 m/s |
| 角速度上限 | 1.4 rad/s |
| controller／local costmap 頻率 | 20.0 / 10.0 Hz |
| GoalAlign／PathAlign／RotateToGoal | 24.0 / 32.0 / 32.0 |
| ObstacleFootprint.scale | 0.16 |
| 全域膨脹半徑／衰減 | 0.2 m / 0.5 |
| 局部膨脹半徑／衰減 | 0.35 m / 1.0 |
| 一般 goal checker XY／yaw | 0.05 m / 3.14 rad |
| prep goal checker XY／yaw | 0.05 m / 3.14 rad |
| prep DWB XY | 0.05 m |

以上依實機檔案整理，沒有在打包時另行調參。prep DWB 與 goal checker 的 XY 容許值仍不同，為目前來源現況。車身 footprint 為長 25 × 寬 32 cm；`self_filter.yaml` 包含完整車身遮蔽與既有支架遮蔽。額外 Collision Monitor 未啟動。

## 地圖與 Keepout

目前主地圖：`maps/mecanum_map.yaml`，解析度 0.05 m，原點 `[-7.18, -11.5, 0]`。

Keepout：`maps/keepout.yaml`，原點 `[-1.79, -4.84, 0]`，**預設停用**。舊遮罩與現有地圖的原點不同，未重新對齊前不要直接啟用。`src/mecanum_navigation/maps/1031i_keepout.*` 為先前繪圖底稿，並非目前預設遮罩。

## 檔案與紀錄

- `src/`：10 套件實機來源。
- `maps/`：當前主地圖與停用的 Keepout 遮罩。
- `mapping/`：TIRT Cartographer 建圖啟動檔、Lua 設定與 RViz 設定。
- `diagnostics/`：導航／GUI／終端機整合啟動與視窗配置、存點入口。
- `scripts/`：工作區環境、編譯、分開啟動、建圖入口。
- `test_logs/`：執行時才建立，含參數快照、rosbag 和各程序日誌，不納入 Git。

`source scripts/env.sh` 會將 `TIRT_WORKSPACE` 指向此儲存庫，來源、地圖、點位與修正庫不依賴原機器的 `/home/leo` 路徑。USB serial-by-id、相機 `/dev/video0` 保留原機設定，換機需確認裝置。

## 驗證範圍

本次更新完成 Python 語法、YAML/XML、shell 與資源路徑檢查；任務與 GUI 已在開發時完成離線流程及無硬體桌面預覽檢查。未重新完整編譯，也未自動進行實車導航測試。上傳的是目前可審閱的來源快照，不是安全認證結果。部分原始套件 license 仍為 TODO，保留原始聲明。
