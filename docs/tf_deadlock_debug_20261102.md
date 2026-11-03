# TF 鎖死除錯：2026-11-02

## 結論

已用失敗測試的錄製資料，在不啟動底盤、不送導航目標的隔離環境重現 Nav2 costmap 的 TF 凍結；GDB 堆疊確認 `tf2` 0.25.23 的鎖順序反轉。

系統仍使用原本的 tf2 0.25.23；導航平移 0.22 m/s、控制 10 Hz 設定未變。本次僅增加專案內的診斷與離線測試檔案。

## 環境與證據

- ROS Humble / arm64；Nav2 1.1.20；tf2 與 tf2_ros 0.25.23。
- 實車錄製：`test_logs/20261102_210116_10hz_u86t`。
- 異常期間 `/scan_raw`、`/scan` 約 10 Hz，EKF TF 約 20 Hz，AMCL TF 約 10 Hz，皆持續更新。
- 控制器持續輸出 0.32 m/s，直到操作手取消；不是底盤僅保留一筆舊命令。這份錄製早於降速至 0.22。
- 0.22 測試 `test_logs/20261102_211535_10hz_juzK` 則接近桌邊後輸出零速度，但沒有成功完成，並反覆報相同舊 TF。

## 離線重現

`offline_tf_debug.py` 在 Domain 191 啟動地圖、規劃器、控制器與 lifecycle manager，以 2 倍速重播同一份資料。只重播 TF、雷射和濾波里程計，不重播速度／任務 topic。無硬體節點，無導航目標，控制器速度輸出另行 remap。

- `offline_tf_result/samples.json`：全域 footprint 時間停在 1793624528.962，局部停在 1793624528.462；外部 TF 更新至 1793624582.512。
- 全域位置與外部 map pose 相差 1.248 m。
- costmap 關閉時無法正常結束，最終僅清理本次隔離測試程序。
- `--gdb` 在 Domain 192 第二次重現：`offline_tf_gdb_result/`。

GDB 在規劃器與控制器都看到兩條互相等待的執行緒：

```
雷射 MessageFilter：
  Buffer::waitForTransform
    → BufferCore::addTransformableRequest
    → 等待 transformable_requests_mutex_

TF TransformListener：
  BufferCore::setTransform
    → BufferCore::testTransformableRequests
    → transform-ready callback
    → 等待 timer_to_request_map_mutex_
```

前者持有 timer map mutex 等 requests mutex；後者持有 requests mutex 等 timer map mutex，形成鎖循環。外部 TF 持續更新，卡住的 buffer 仍可能回傳它最後收到的位置，因此不保證導航會自動報失敗或停止。

直接附加 GDB 被 ptrace 限制阻擋，因此改以 GDB 作為隔離測試程序的父程序取得堆疊，未改系統 ptrace 設定，也未附加實車程序。

## 官方對照與修正方向

- https://github.com/ros2/geometry2/issues/992
- https://github.com/ros2/geometry2/issues/995
- https://github.com/ros2/geometry2/blob/0.25.24/tf2/src/buffer_core.cpp

本機 apt 索引提供 tf2 0.25.24。已對照 0.25.23 / 0.25.24 原始碼：新版把 transform-ready callbacks 延到內部 mutex 解鎖後執行，移除這條鎖循環。

下載的官方 arm64 `.deb` 僅解壓於 `tf2_02524_trial/`；`--fixed` 只讓 Domain 193 的離線程序載入新版 libtf2.so，不安裝、不覆蓋 `/opt/ros/humble`。A/B 結果記錄於 `offline_tf_fixed_result/`。

## A/B 結果

| 測試 | 全域 costmap | 局部 costmap | 關閉 |
|---|---|---|---|
| 原版 0.25.23 | 時間與位置凍結 | 時間與位置凍結 | 無法正常關閉 |
| 原版 0.25.23 + GDB | 再次凍結，取得鎖循環堆疊 | 再次凍結，取得鎖循環堆疊 | 擷取堆疊後終止離線程序 |
| 僅測試程序使用 0.25.24 libtf2 | 完整重播持續更新 | 完整重播持續更新 | 所有離線子程序正常退出 |

修正版最後樣本：外部 TF 時間 1793624582.562，全域 footprint 1793624582.112，局部 footprint 1793624582.463；全域 footprint 與同時外部 map pose 的位置差小於 1 微米（重播末段車輛已靜止）。這僅用來驗證最後位置一致，不代表實車定位精度。

測試載入路徑經 `ldd` 確認：libtf2.so 取自 `tf2_02524_trial/opt/ros/humble/lib/`，libtf2_ros.so 仍為系統 0.25.23。只更換核心庫即消除這份資料觸發的鎖死。

離線重播驗證只涵蓋 TF/costmap 處理與程序關閉；不等於完成實車兩趟導航驗證。
