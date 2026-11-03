# Ros2_tirt

ROS 2 Humble 麥克納姆輪送餐比賽工作區，整理自 2026-11-03 實車使用中的程式與設定。包含終端機比賽流程、Nav2、RViz、STM32 底盤、EKF、光達、車身遮蔽、相機／QR、點位記錄，以及 tf2 鎖死修正庫。

## 環境與依賴

目標環境為 Ubuntu 22.04 arm64／aarch64、ROS 2 Humble。內附的 `tf2_0.25.24/lib/libtf2.so` 是官方 arm64 套件的修正核心庫；不可直接拿到 x86 電腦使用。它只在本專案導航啟動的子程序內載入，不覆蓋系統套件。版權與來源見 `tf2_0.25.24/COPYRIGHT`、`PROVENANCE.json`。

需已安裝 ROS 2 Humble 與 colcon。新機器可先安裝主要依賴（會修改系統，請由操作人執行）：

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-yaml \
  python3-serial python3-numpy python3-opencv python3-pil python3-pil.imagetk python3-tk \
  libpcap-dev libboost-thread-dev libzbar-dev libopencv-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 \
  ros-humble-robot-localization ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-xacro ros-humble-laser-filters \
  ros-humble-usb-cam ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-conversions ros-humble-diagnostic-updater ros-humble-rosbag2
```

若有其他缺少的依賴，可在本目錄執行 `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble`；先完成 rosdep 初始化。原專案部分套件的 license 欄位尚為 TODO，保留原始聲明，未另行宣稱授權。

## 編譯

```bash
git clone https://github.com/keys-lll/Ros2_tirt.git
cd Ros2_tirt
bash scripts/build.sh
```

## 分開啟動

先停止其他工作區的導航與任務，將車放在已記錄的 start 位置與朝向。兩個終端機均進入本儲存庫目錄。

終端機 1：導航、RViz、rosbag 與日誌

```bash
bash scripts/run.sh nav
```

終端機 2：比賽終端機

```bash
bash scripts/run.sh mission
```

任務載入所有記錄點，輸入 1～5 號桌後先回 start，再按 Enter 出發。點餐與加點步驟開啟相機、掃 QR 或按 Enter 放行；備餐區按 Enter 出發。加點步驟直接 Enter 表示不加點並回 start。送完加點後的最後桌邊步驟只接受 Enter 確認返回 start，不啟動 QR。`q` 加 Enter 中止任務，Ctrl+C 停止程式。相機不需第三個終端機。

日誌與錄製資料存於 `test_logs/日期時間_nav_*/`；`latest_test.txt` 指向最近一次導航。每次錄製保留當次導航參數及點位快照。請在導航終端機 Ctrl+C 結束，以完成 bag 寫入。

## 手動調點與記錄

```bash
nano src/car_waypoints/config/waypoints.yaml
```

第一行必須為 `waypoints:`。x、y 為地圖公尺座標，yaw 為弧度。新任務重新載入點位，原本已送出的導航目標不會隨檔案改動。

需要重新記錄目前車位時，先啟動導航並確認定位：

```bash
source scripts/env.sh
ros2 run car_waypoints wp_record
```

底盤遙控（不要與進行中的自主導航同時使用）：

```bash
source scripts/env.sh
ros2 run keyboard_control keyboard_control
```

## 目前主要設定

| 項目 | 設定 |
|---|---|
| 平移速度上限 | 0.38 m/s |
| 旋轉速度上限 | 1.4 rad/s |
| 控制頻率 | 20 Hz |
| 到點位置／方向容許誤差 | 0.05 m／20 度 |
| 膨脹半徑／衰減係數 | 0.30 m／1.5 |
| RotateToGoal 角度評分前瞻 | 0.5 秒 |
| 車身 footprint | 長 0.25 m、寬 0.32 m |
| 光達相對 base_link | x=0.12 m、z=0.23 m |

Nav2 設定為 `src/mecanum_navigation/config/nav2_params.yaml`；車身遮蔽為同目錄 `self_filter.yaml`。額外 collision monitor 未啟動，Nav2 仍使用 costmap／footprint 判斷碰撞。4 號桌仍包含原有的外側 35 公分對準步驟。

地圖為 `maps/mecanum_map.yaml` 與對應 PGM；地圖原點與圖檔必須成對保留。RViz 預設顯示 Map、TF、AMCL、雷射、全域路徑，以及全域／局部 costmap。實車 USB serial-by-id、相機 `/dev/video0` 設定沿用原機，換機時請確認 `hardware.launch.py`、`navigation.launch.py` 與相機裝置。

透過腳本啟動時會設定 `TIRT_WORKSPACE`，因此點位、修正庫與地圖使用這個儲存庫自身檔案。直接使用 ros2 命令前請先 `source scripts/env.sh`。

## 驗證範圍

整理時進行編譯與離線檢查；未自動啟動底盤或進行實車任務。TF 修正的既有離線 A/B 證據見 `docs/tf_deadlock_debug_20261102.md`；歷史報告中的原機路徑僅供追溯。
