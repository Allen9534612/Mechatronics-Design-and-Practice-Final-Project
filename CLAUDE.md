# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 工作規則

- 回覆使用者一律使用中文。
- 每次開發前，請先閱讀本檔案（CLAUDE.md）以掌握專案現況。
- 任何開發歷程（完成的工作、重大決策、已知問題等）都要記錄在本檔案的「開發歷程」章節中，保持更新。
- 排版：C/C++ 用 `clang-format -i -style=file`（src/lib/include），Python 用 `ruff format script/` 與 `ruff check --fix script/`（指令見 README）；改完程式後先排版再 commit。

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
- 監控 GUI（OpenGL，可縮放）：`pip install -r script/requirements.txt` 後執行 `python script/heatmap.py`（自動偵測序列埠）、`--port COM5` 指定埠、`--demo` 用模擬資料（不接硬體）、`--snapshot out.png` 存一張畫面後離開（除錯用）。

There is no test suite or linter configured in this repo.

## Architecture

- `platformio.ini`：`esp32`（預設，`nodemcu-32s`）依賴 ESP32Servo、Adafruit PWM Servo Driver、stm32duino/VL53L7CX；`mega`/`uno` 僅保留設定，現有程式碼只能在 esp32 編譯。`[env] build_flags = -Iinclude` 讓 `lib/*` 也能 `#include "Config.h"`。
- `include/Config.h`：**全專案唯一的設定來源**（I2C 腳位 SDA=22/SCL=21、VL53L7CX 參數與篩選門檻、感測器座標旋轉/翻轉、PCA9685 與伺服脈寬、伺服校正 `SERVO_CAL_DEFAULT`、平台機構尺寸、`MAX_TILT_DEG`、PID 預設增益等）。`src/main.cpp` 與所有 `lib/*` 都 include 它，不要在其他檔案重複定義常數。
- 資料流（`src/main.cpp` 的 `loop()`，`CONTROL_PERIOD_MS` 週期）：
  1. `lib/IRSensorArray`：透過 I2C 驅動 VL53L7CX，`readData()` 取得 4x4/8x8 的 `grid_strength`、`grid_dist`（無效區域為 0）與有效區域數；`changeResolution(4|8)` 切換解析度。
  2. `lib/BallPositionEstimator`：`estimate()` 回傳 `BallEstimate{valid, gridX, gridY, distMm, xMm, yMm}`。先找峰值，再用三點 log 拋物線求次格點位置（邊緣峰值同側外推，不在邊界外補值），距離取球附近以訊號強度加權的平均；`gridToMm()` 依 `TOF_FOV_DEG`（60°）與距離把格點換算成 mm，原點在感測器視野中心（+x 向右、+y 向下）。
  3. `main.cpp` 的 `sensorToPlatform()`：套用 `SENSOR_ROTATION_DEG` / `SENSOR_FLIP_X/Y`，轉成平台座標（mm）。
  4. `lib/BalanceController`：PID，**輸入 mm（與目標點的誤差）、輸出傾角度數**；kp=度/mm、ki=度/(mm·s)、kd=度/(mm/s)；輸出 `theta`（傾角）與 `phi`（往下傾的方向）。
  5. `lib/PlatformKinematics`：移植自 I-M-Robotics-Lab/Ball-Balancing-Robot 的三軸逆向運動學，`solveSpherical(theta, phi, h, modelDeg[3])` 與 `computeMaxTilt(h)`。
  6. `lib/ServoMotor`：透過 PCA9685 輸出 PWM，`setAngle(channel, angle)`。
  `main.cpp` 的 `modelToServo()` 依 `servoCal`（neutral/gain/offset/限幅）把模型角轉成伺服角。`maxTilt = min(運動學可達最大傾角, MAX_TILT_DEG)`。
- 序列埠：`DATA,<N>,<N*N 個距離>,<gridX>,<gridY>` 供 `script/heatmap.py` 使用，**gridX/gridY 維持格點座標、無球為 -1**；指令列表輸入 `?` 查看。
- 序列埠另有 `STAT,mode,present,x,y,theta,phi,s1,s2,s3,kp,ki,kd,targetX,targetY,height,maxTilt,distMm,validZones,integX,integY`（每 `STAT_PERIOD_MS` 一次，不受模式限制；由 `main.cpp` 的 `printStat()` 輸出）。韌體不再週期性輸出人類可讀狀態列，指令回覆（`>>` 開頭）與 `s` 指令仍保留。**改欄位時 `script/heatmap.py` 的 `STAT_FIELDS` 要同步。**
- `script/heatmap.py`：moderngl（OpenGL 3.3）+ pygame 的監控 GUI，視窗可任意縮放（版面等比例縮放），配色為中性灰階加單一強調色。左側為資料流數值（感測器 → 位置估測 → PID → 伺服），中間為距離熱圖（色階依畫面內距離範圍自動調整）、球位置、傾斜方向與韌體訊息，右側為可調項（解析度、自動/回水平、PID、目標點、高度、傾斜測試、指令輸入框）與懸停提示。pyserial 背景執行緒收發並自動重連，顯示值一律以 STAT 為準。程式以 DPI-aware 啟動（避免 Windows 縮放造成模糊），啟動時偵測顯卡並在左下角顯示實際渲染的 GPU；用到內顯而系統有獨顯時會警示，`--gpu high` 可寫入 Windows 圖形偏好（`HKCU\Software\Microsoft\DirectX\UserGpuPreferences`）並重新啟動。GUI 內的 `SERVO_MIN_DEG/MAX_DEG` 需與 Config.h 一致。
- `test/raw.cpp`：獨立的感測器原始資料測試程式（不屬於主 build）。

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
- 2026-10-22：用 mock 資料檢查整條 pipeline（`pio run -e esp32` 編譯成功；另在 host 端用 g++ 搭配假的 Arduino/Wire/VL53L7CX/PCA9685 mock，編譯真實的 `src/main.cpp` 與全部 `lib/*`，餵模擬的 4x4/8x8 感測器畫面、序列埠指令與簡易球滾動模擬，共 135 項檢查，134 通過）。測試腳本放在暫存目錄，未納入 repo。結論與已知問題：
  - 正常：伺服校正與 PCA9685 計數值換算、逆向運動學（最大傾角範圍內全方向可解、極端姿態回傳 false 而非 NaN）、PID 方向（球在 +X → phi=180）與限幅/抗積分飽和、無球時輸出衰減回水平、雜訊/無效畫面不產生 NaN 或超出伺服上下限、4x4↔8x8 切換、全部序列埠指令（含錯誤格式）、閉迴路模擬方向正確且收斂。
  - **已知問題（座標系）**：`BallPositionEstimator` 輸出的是「格點索引」（4x4 為 0~3、原點在感測器左上角），但 `main.cpp` 直接把它當 mm、以 (0,0) 為目標點餵給 PID。結果球在盤面正中央時也會讓平台傾斜（實測約 0.7 度），且 kp 的單位（度/mm）與輸入單位（格）不符。需在 estimator 或 main 加上「格點→mm」換算（格距）並把原點移到盤面中心。
  - `Config.h` 目前沒有被任何模組使用（腳位仍是舊的 LEDC/類比設計），與實際的 PCA9685 + VL53L7CX 接線不一致；本檔「Architecture」章節描述的模組介面（`update()/reading()`、`computeServoAngles()`、LEDC `ServoMotor`）也已過時，與現行程式不符。
  - `IRSensorArray::halt()` 在感測器偵測不到時會無限迴圈（開機卡死，需靠接線檢查）；邊角的球位置估測有往中心偏的偏差（約 0.35 格）。
- 2026-10-22（修改）：依 mock 檢查結果修正並整合設定：
  - `BallPositionEstimator` 改版：介面改為 `estimate(strength, dist, N, validCount) → BallEstimate`；改用 3 點 log 拋物線求次格點位置，邊緣峰值改用同側三點外推（原本的邊界複製補值會把邊角的球往外推約 0.35~0.47 格，現在 mock 高斯模型下邊緣誤差 ≤0.035 格、內部 ≤0.01 格；3% 雜訊下平均 0.03 格、最差 ~0.28 格）。新增格點→mm 換算（`TOF_FOV_DEG`/N 為每格視角，乘上球附近加權平均距離），輸出以視野中心為原點的 `xMm/yMm`，PID 輸入與目標點 `g x y` 都改為以盤面中心為原點的 mm。
  - 先前「球在中心仍傾斜 0.7 度」的原因：不是伺服 offset，而是座標原點在格點 (0,0) 角落而非盤面中心，PID 把中心位置當成誤差；另外當時測試把球放在 (3.5,3.5)，那其實已超出 4x4 格（中心是 1.5），0.7 度是被高估的數字。修正後球在中心時估測為 (0,0) mm，傾角 0.008 度（殘留為前一次位置累積的積分，`r` 指令可重置）。伺服 offset 本身沒有問題（水平時 90 / 85.5 / 86 度）。
  - PID 單位對齊：輸入 mm、輸出度，增益預設值改由 Config.h 提供（`PID_KP/KI/KD` 改成原本類別內的實際值 0.12/0.01/0.06，`PID_OUTPUT_ALPHA`、`PID_D_ALPHA`、`PID_INTEGRAL_LIMIT_DEG`），`p` 指令與狀態列標明單位。
  - 所有模組改用 `Config.h`：I2C 腳位（SDA=22、SCL=21，PCA9685 與 VL53L7CX 共用）、VL53L7CX 位址/頻率/篩選門檻/LPn、PCA9685 位址與脈寬、機構尺寸、伺服校正、控制週期等；移除 `IRSensorArray.h` 的 `#define` 與 main.cpp 的重複常數。`MAX_TILT_DEG`（15 度）現在會與運動學可達範圍取小（原本實際為 16.3 度）。`platformio.ini` 新增 `-Iinclude`，否則 `lib/*` 找不到 Config.h。
  - 驗證：`pio run -e esp32` 成功、無警告；host 端 mock 測試 1137 項全數通過（含估測器真值比對、雜訊、mm 換算、閉迴路球滾動模擬收斂到中心）。
  - 仍待處理：`TOF_FOV_DEG`、感測器安裝方向與 `SENSOR_FLIP/ROTATION`、實際訊號形狀（目前以高斯球模擬）、PID 增益都需上機校正；`IRSensorArray::halt()` 找不到感測器時仍會無限迴圈。
- 2026-10-22：更正 `TOF_FOV_DEG` 為 60（VL53L7CX 單邊 60°x60°、對角約 90°；先前誤用 45°，會讓 mm 換算偏小約 25%）。
- 2026-10-22：整理 git（3 個 commit：設定集中化、估測器 mm 化、文件）。新增 `STAT` 遙測行（`main.cpp` 的 `printStat()`、`Config.h` 的 `STAT_PERIOD_MS`），並重寫 `script/heatmap.py` 為 moderngl/OpenGL 的可縮放 GUI（左：資料流數值；右：可調項按鈕與提示；pyserial 背景執行緒收發；`--demo` 模擬模式；`--snapshot` 截圖）。新增 `script/requirements.txt`。驗證：用 pygame 事件模擬 19 項點擊/鍵盤/滾輪/縮放操作，確認各按鈕送出正確指令；把 host 端編譯的真實韌體輸出（DATA/STAT）餵給 GUI 解析器，格式吻合；韌體 mock 測試 1137 項與 `pio run -e esp32` 皆通過。已知注意：4x4 / 60 Hz 時 DATA 約 8 KB/s，加上 STAT 與狀態列已接近 115200 baud 上限（約 11.5 KB/s），若序列埠出現延遲/掉資料，考慮降低 4x4 頻率、提高 baud（需同步改 `SERIAL_BAUD`、`monitor_speed`、heatmap `--baud`）或移除人類可讀狀態列。GUI 尚未在真實硬體上驗證。
- 2026-10-22：移除週期性狀態列（改由 STAT/GUI 顯示）；src/include/lib 註解精簡為 Google 風格；`heatmap.py` 改版：單一強調色的扁平卡片版面、熱圖色階自動調整、Google 風格 docstring（4 格縮排，以 ruff 排版）。模糊的原因是 Windows 縮放 175% 下程式非 DPI-aware 被系統放大（本機只有一張 RTX 4060，並非用錯顯卡），已改為 DPI-aware 並依縮放比例決定初始視窗大小；另加入顯卡偵測、GL 渲染器顯示與 `--gpu` 選項。驗證：21 項 GUI 互動測試、顯卡函式測試、韌體 mock 測試 1136 項與 `pio run -e esp32` 通過；`--gpu high` 的寫入與重新啟動流程只測了登錄檔讀寫函式，未實際執行。
- 2026-10-22：PCA9685 偵測不到時不再讓韌體卡死（原本 `setup()` 印出 `PCA9685 initialization failed!` 後無限迴圈，導致感測器與 `loop()` 都不執行、heatmap 收不到資料）。現在只警告並停用伺服輸出（`servoReady`），感測、估測、PID 與 STAT/DATA 照常輸出，`s` 指令會顯示伺服驅動狀態；`ServoMotor::begin()` 失敗時會將 `initialized` 設回 false。注意：沒接 PCA9685 時 GUI 的伺服角度是「若有接會送出的值」，並未真的輸出。host mock 測試新增「PCA9685 缺席」情境（共 1146 項），模擬球訊號峰值改為 900 以符合 `TOF_FILTER_MAX_STRENGTH`=1000。
- 2026-10-22：修正 `heatmap.py` 崩潰：文字貼圖快取超過上限時在繪製途中釋放貼圖，造成本幀已排入的文字失效而拋出 `InvalidObject`（8×8 每幀有 64 個變動數字，更容易觸發）。現在只在幀開頭整理快取，上限提高到 4000（電腦端顯示記憶體，與 ESP32 無關）。另外：視窗失焦時清除按住連發狀態、序列埠寫入加 0.5 秒逾時、傾斜測試角度與平台高度（`HEIGHT_MIN_MM/MAX_MM`，需與機構尺寸一致）限制在有效範圍、崩潰時寫入 `script/heatmap_crash.log`（已 gitignore）。PCA9685 偵測不到時韌體不再卡死。已知：PID 增益每次調整韌體都會清除積分（`p` 指令的既有行為）。
