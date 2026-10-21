# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 工作規則

- 回覆使用者一律使用中文。
- 每次開發前，請先閱讀本檔案（CLAUDE.md）以掌握專案現況。
- 任何開發歷程（完成的工作、重大決策、已知問題等）都要記錄在本檔案的「開發歷程」章節中，保持更新。

## Project overview

PlatformIO-based embedded firmware project for a mechatronics final project. The primary target is an ESP32 board; `mega`/`uno` AVR environments also exist in the build config but the current library code is ESP32-specific (see Architecture notes below).

## 專案介紹

本專案為使用三顆 MG996 角度控制伺服馬達，驅動一個平衡球平台（ball balancing platform）的專案。預計的系統架構（資料流）如下：

1. 紅外線感測陣列（IR sensor array）讀取數據作為輸入。
2. 將感測陣列數據換算成球在平台上的位置（球的 x, y 座標）。
3. 根據球的位置計算出平台需要傾斜的角度（平台的 roll / pitch）。
4. 將平台傾斜角度換算成三顆伺服馬達各自應旋轉的角度（逆向運動學，inverse kinematics）。
5. 最後控制三顆伺服馬達轉到對應角度，實現平台傾斜與球的平衡控制。

## Commands

This is a PlatformIO project (`platformio.ini` at the repo root). Use the PlatformIO CLI (`pio`):

- Build default environment (esp32): `pio run`
- Build a specific environment: `pio run -e esp32` / `pio run -e mega` / `pio run -e uno`
- Upload to board: `pio run -e esp32 -t upload`
- Open serial monitor (115200 baud, set via `monitor_speed` for esp32): `pio device monitor`
- Clean build artifacts: `pio run -t clean`

There is no test suite or linter configured in this repo.

## Architecture

- `platformio.ini` defines three environments:
  - `esp32` (default): `espressif32` platform, `esp32dev` board, depends on `madhephaestus/ESP32Servo`.
  - `mega` / `uno`: `atmelavr` platform, depend on `arduino-libraries/Servo`, `SoftwareSerial`, and `FreeRTOS`.
- `include/Config.h` 是全專案共用的設定檔（接腳、感測器數量、平台幾何、PID 參數等），所有模組都會 include 它。裡面標 `TODO` 的數值都是佔位值，需依實際硬體調整。
- `src/main.cpp` 的 `loop()` 依固定週期（`LOOP_INTERVAL_MS`）執行完整 pipeline，對應「專案介紹」的 5 個階段，依序呼叫下面四個 `lib/` 模組 + `ServoMotor`：
  1. `lib/IRSensorArray` — 讀取紅外線感測陣列原始值（`update()` / `reading(i)`）。目前 `update()` 是用 `analogRead` 的佔位實作，需依實際感測器型別調整。
  2. `lib/BallPositionEstimator` — 把感測陣列讀值換算成球的位置 `BallPosition2D{x, y, valid}`。目前 `compute()` 用「假設感測器環狀均勻分布、讀值當權重做加權平均」的佔位演算法，需依實際感測器佈局重新設計。
  3. `lib/BalanceController` — 吃球的位置，用簡單 PID 算出平台該傾斜的角度 `PlatformTilt{rollDeg, pitchDeg}`。增益值在 `Config.h` 的 `PID_KP/KI/KD`，目前都是佔位值待調校。
  4. `lib/PlatformKinematics` — 把 `PlatformTilt` 換算成三顆馬達各自的角度 `ServoAngles3{theta1, theta2, theta3}`。目前 `computeServoAngles()` 只是假設三馬達 120° 環狀佈局的簡化近似公式（非真正逆向運動學），需依實際平台機構（搖臂長度、佈局半徑等）重新推導。
  5. `lib/ServoMotor` — 控制單顆伺服馬達。`ServoMotor` 用 ESP32 LEDC 直接產生 PWM，支援立即寫入角度（`writeAngle`/`writeMicroseconds`）與平滑轉動（`setTargetAngle` + 每週期呼叫 `run(stepDeg)` 逼近目標角度）。**注意：** 目前直接呼叫 ESP32-only 的 LEDC API（`ledcSetup`、`ledcAttachPin`、`ledcWrite`、`ledcDetachPin`），沒有走 `lib_deps` 宣告的 `ESP32Servo`/`Servo` 抽象層，所以這個 library 現在只能在 `esp32` 環境編譯，無法用於 `mega`/`uno` 環境。
- 四個資料流模組之間用上一階段定義的 struct 傳遞資料（`BallPosition2D` 定義在 `BallPositionEstimator.h`、`PlatformTilt` 定義在 `BalanceController.h`、`ServoAngles3` 定義在 `PlatformKinematics.h`），下游模組 include 上游的 header 取得型別。
- 目前所有感測/定位/控制演算法都是功能骨架（佔位邏輯，標了 TODO），可以編譯執行跑完整個 pipeline，但數值不會是正確的平衡控制結果，需等硬體規格確定後逐一替換。

## 開發歷程

- 2026-10-16：建立 CLAUDE.md，記錄專案架構與建置指令；新增工作規則（中文回覆、開發前閱讀本檔、開發歷程記錄於此）。
- 2026-10-16：新增「專案介紹」章節，記錄平衡球平台的系統架構與資料流（紅外線感測陣列 → 球位置 → 平台傾斜角度 → 三顆馬達角度 → 伺服馬達控制）。
- 2026-10-16：搭出程式骨架，把 5 階段資料流拆成對應模組：新增 `include/Config.h`（共用設定/腳位/參數，皆為 TODO 佔位值）、`lib/IRSensorArray`、`lib/BallPositionEstimator`、`lib/BalanceController`（PID）、`lib/PlatformKinematics`（逆向運動學，暫用簡化近似公式）；補上原本缺少的 `lib/ServoMotor/ServoMotor.h`（並把檔名從 `servoMotor.h` 改成與 `#include "ServoMotor.h"` 大小寫一致的 `ServoMotor.h`，避免在大小寫敏感的檔案系統上編譯失敗）；`src/main.cpp` 串接整條 pipeline（讀取感測 → 算球位置 → PID 算傾斜角 → 逆向運動學算三馬達角度 → 輸出到伺服馬達）。所有演算法目前都是可編譯執行的佔位邏輯，待確認硬體規格（感測器型號/佈局、平台機構幾何）後逐一替換成正確實作。
- 2026-10-16：確定實際感測器為 **VL53L7CX**（ST 的 8x8 多區域 ToF 測距感測器，I2C 介面），取代目前 `lib/IRSensorArray` 骨架假設的「多顆類比紅外線感測器」設計。待辦事項：
  - `lib/IRSensorArray` 需改寫為透過 I2C 讀取 VL53L7CX 的 8x8（64 點）距離矩陣，`Config.h` 的 `IR_SENSOR_COUNT`/`IR_SENSOR_PINS` 腳位設計需移除/替換成 I2C 腳位設定。
  - `lib/BallPositionEstimator` 的加權平均佔位演算法需重新設計，改成基於 8x8 距離矩陣估算球位置。
  - 參考的 `stm32duino/VL53L7CX` repo 是 STM32 平台專用，esp32 環境不能直接用，需另找/自製 ESP32 相容的 driver（或改包 ST 官方 ULD C driver）。
- 2026-10-16：改寫 `lib/IRSensorArray`，改為透過 I2C 驅動 VL53L7CX（8x8=64 區域 ToF 測距）：
  - 查證結果：`stm32duino/VL53L7CX` library 底層只用 `TwoWire`/`pinMode`/`digitalWrite`/`delay()` 等標準 Arduino API，沒有呼叫任何 STM32 HAL 函式，所以雖然 `library.properties` 的 `architectures` 只標示 STM32/SAM，實際上可以在 ESP32（Arduino core）上編譯運作。因此**更正**前一則記錄裡「該 library 不能用在 ESP32，需另找/自製 driver」的結論；改為直接把它加進 `platformio.ini` 的 `esp32` 環境 `lib_deps`（用 GitHub URL `https://github.com/stm32duino/VL53L7CX.git` 引入，避開 PlatformIO 註冊表 library 名稱比對問題）。
  - `Config.h`：移除 `IR_SENSOR_COUNT`/`IR_SENSOR_PINS`，改為 `TOF_I2C_SDA_PIN`/`TOF_I2C_SCL_PIN`/`TOF_I2C_FREQ_HZ`/`TOF_LPN_PIN`/`TOF_I2C_RST_PIN`/`TOF_RANGING_FREQUENCY_HZ`（皆為 TODO 佔位值，待依實際接線調整；LPN 是感測器電源致能腳位，大多數模組都會接出）。
  - `IRSensorArray` 類別名稱保留（維持 pipeline 其他模組介面相容），但內部已改為包裝 `VL53L7CX` 物件：`begin()` 初始化 I2C 與感測器（解析度設為 8x8、設定更新頻率、開始量測），`update()` 檢查資料是否就緒並讀取 `VL53L7CX_ResultsData`，`count()` 回傳 64，`reading(index)` 回傳該區域距離（mm），新增 `readingValid(index)` 回傳該區域量測是否有效（`target_status` 為 5 或 9）。
  - **尚未處理**：`lib/BallPositionEstimator` 目前仍假設感測器均勻分布在圓周上（用索引算角度、讀值當權重做加權平均），對 8x8 距離矩陣來說已經不合理，下一步需要重新設計成依 8x8 區域網格與 `readingValid()` 來估算球位置。
  - 因為目前環境沒有安裝 PlatformIO CLI，這次改動**未實際編譯驗證**，之後建議在有 `pio`/VS Code PlatformIO 擴充套件的環境跑一次 `pio run -e esp32` 確認。
