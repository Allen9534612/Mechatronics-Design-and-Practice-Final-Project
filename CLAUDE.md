# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 工作規則

- 回覆使用者一律使用中文。
- 每次開發前，請先閱讀本檔案（CLAUDE.md）以掌握專案現況。
- 任何開發歷程（完成的工作、重大決策、已知問題等）都要記錄在本檔案的「開發歷程」章節中，保持更新。
- 排版：C/C++ 用 `clang-format -i -style=file`（src/lib/include），Python 用 `ruff format script/` 與 `ruff check --fix script/`（指令見 README）；改完程式後先排版再 commit。
- commit message 使用英文且盡量簡要。
- 每次執行 git 操作前先問過使用者，包含 commit 及 worktree、branch 建立。
- 本地 worktree 使用完刪除。

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

排版與靜態檢查（Python 只有 `script/`，韌體在 `src/`、`lib/`）：
- 格式：`ruff format script/`、`ruff check --fix script/`（設定在 `ruff.toml`）
- 型別：`mypy`（設定在 `mypy.ini`，`files = script`）。pyserial 沒有附型別資訊，
  已用 `[mypy-serial.*] ignore_missing_imports` 放寬；想真正檢查 serial 的用法就
  裝 `types-pyserial` 並移除那段。**刪掉 `mypy.ini` 不會停用 mypy**——編輯器的
  擴充套件仍會以預設值執行，要停用請關閉 VS Code 的
  `ms-python.mypy-type-checker`，或在 `.vscode/settings.json` 加
  `"mypy-type-checker.ignorePatterns": ["**/*.py"]`。

本 repo 沒有測試套件（host 端的驗證腳本都放在暫存目錄，未納入 repo）。

## Architecture

- `platformio.ini`：`esp32`（預設，`nodemcu-32s`）依賴 ESP32Servo、Adafruit PWM Servo Driver、stm32duino/VL53L7CX；`mega`/`uno` 僅保留設定，現有程式碼只能在 esp32 編譯。`[env] build_flags = -Iinclude` 讓 `lib/*` 也能 `#include "Config.h"`。
- **平台高度的三層上限**（排查「為什麼調不上去」時先看這裡，以目前尺寸計算）：
  `96.9`（舊的 40~140 伺服安全角度上限，放寬到 20~160 後已非瓶頸）→
  `101.67`（`computeMaxTilt()` 歸零：可達傾角不足 `KINEMATICS_TILT_MARGIN_DEG`
  的餘裕時回傳 0，`h` 指令會報「這個高度無法保持水平」）→
  `102.33`（機構硬極限 `maxHeight()`＝連桿與擺臂完全伸直成一直線，
  `sqrt((l1+l2)^2 - (lp-lb)^2)`，再高運動學無解）。下限則由伺服安全角度決定，
  20~160 對應約 20.1 mm。
- 容許值一律放 `Config.h`：伺服角度分成 `SERVO_RANGE_MIN/MAX_DEG`（機構工作範
  圍，運動學輸出與 `d` 指令用，`SERVO_CAL_DEFAULT` 的 minDeg/maxDeg 由它填）與
  `SERVO_HARD_MIN/MAX_DEG`（伺服物理極限，校正用的 `a` 指令與
  `ServoMotor::setAngle()` 的最終夾限、脈寬換算用）；平台高度為
  `PLATFORM_MIN/MAX_HEIGHT_MM`，由 `main.cpp` 的 `minUsableHeight()/
  maxUsableHeight()` 與運動學可達範圍取交集（作法與 `MAX_TILT_DEG` 相同）。
  **這些都是機構範圍，不等於伺服角度實際允許的範圍**——後者更窄，由
  `poseWithinServoRange()` 在下指令時實際檢查。另有運動學搜尋參數
  （`KINEMATICS_TILT_*`）與開迴路軌跡檢查的掃描解析度（`SWEEP_PHI_CHECK_STEP_DEG`、
  `RANGE_SCAN_*`）。
- `include/Config.h`：**全專案唯一的設定來源**（I2C 腳位：VL53L7CX 用 `Wire` SDA=22/SCL=21、PCA9685 用獨立的 `Wire1` SDA=18/SCL=19；VL53L7CX 參數與篩選門檻、感測器座標旋轉/翻轉、PCA9685 與伺服脈寬、伺服校正 `SERVO_CAL_DEFAULT`、平台機構尺寸、`MAX_TILT_DEG`、PID 預設增益等）。`src/main.cpp` 與所有 `lib/*` 都 include 它，不要在其他檔案重複定義常數。
- 資料流（`src/main.cpp` 的 `loop()`，`CONTROL_PERIOD_MS` 週期）：
  1. `lib/IRSensorArray`：透過 I2C 驅動 VL53L7CX，`readData()` 取得 4x4/8x8 的 `grid_strength`、`grid_dist`（無效區域為 0）與有效區域數；`changeResolution(4|8)` 切換解析度。
  2. `lib/BallPositionEstimator`：`estimate()` 回傳 `BallEstimate{valid, gridX, gridY, distMm, xMm, yMm}`。透過壓克力板量測時板子是鏡面、距離不可靠，所以改用「訊號強度區塊」：強度取 log10 → 去除單點鏡面尖峰（比 8 鄰居中位數高 `SPIKE_JUMP`）→ 背景取最低 1/8 的強度，門檻 = max(背景 + `RISE`, `MIN_PEAK`) → 取最大的 4 連通區塊（至少 `MIN_ZONES` 個區域）→ 強度加權重心；距離取區塊內加權平均。8x8 與 4x4 各一組參數（`Config.h` 的 `BALL_*_8X8/_4X4`）。`gridToMm()` 依 `TOF_FOV_DEG`（60°）與距離把格點換算成 mm，原點在感測器視野中心（+x 向右、+y 向下）。
  3. `main.cpp` 的 `sensorToPlatform()`：套用 `SENSOR_ROTATION_DEG` / `SENSOR_FLIP_X/Y`，轉成平台座標（mm）。
  4. `lib/BalanceController`：PID，**輸入 mm（與目標點的誤差）、輸出傾角度數**；kp=度/mm、ki=度/(mm·s)、kd=度/(mm/s)；輸出 `theta`（傾角）與 `phi`（往下傾的方向）。
  5. `lib/PlatformKinematics`：移植自 I-M-Robotics-Lab/Ball-Balancing-Robot 的三軸逆向運動學，`solveSpherical(theta, phi, h, modelDeg[3])` 與 `computeMaxTilt(h)`。
  6. `lib/ServoMotor`：透過 PCA9685（接在 `Wire1`，與感測器分開的匯流排）輸出 PWM，`setAngle(channel, angle)`。
  `main.cpp` 的 `modelToServo()` 依 `servoCal`（neutral/gain/offset/限幅）把模型角轉成伺服角。`maxTilt = min(運動學可達最大傾角, MAX_TILT_DEG)`。
- 序列埠：`DATA,<N>,<N*N 個距離>,<gridX>,<gridY>` 供 `script/heatmap.py` 使用，**gridX/gridY 維持格點座標、無球為 -1**；指令列表輸入 `?` 查看。
- 序列埠另有 `STAT,mode,present,x,y,theta,phi,s1,s2,s3,kp,ki,kd,targetX,targetY,height,maxTilt,distMm,validZones,integX,integY,outAlpha,dAlpha,integLimit,hMin,hMax,servoMin,servoMax`（每 `STAT_PERIOD_MS` 一次，不受模式限制；由 `main.cpp` 的 `printStat()` 輸出）。韌體不再週期性輸出人類可讀狀態列，指令回覆（`>>` 開頭）與 `s` 指令仍保留。**改欄位時 `script/heatmap.py` 的 `STAT_FIELDS` 要同步。**
- `script/heatmap.py`：moderngl（OpenGL 3.3）+ pygame 的監控 GUI，視窗可任意縮放（版面等比例縮放），配色為中性灰階加單一強調色。左側為資料流數值（感測器 → 位置估測 → PID → 伺服），中間為距離熱圖（色階依畫面內距離範圍自動調整）、球位置、傾斜方向與韌體訊息，右側分兩欄，為可調項（第一欄：解析度、自動/回水平、PID、濾波與積分；第二欄：目標點、高度、伺服角度與原點、指令輸入框、提示）與懸停提示。pyserial 背景執行緒收發並自動重連，顯示值一律以 STAT 為準。程式以 DPI-aware 啟動（避免 Windows 縮放造成模糊），啟動時偵測顯卡並在左下角顯示實際渲染的 GPU；用到內顯而系統有獨顯時會警示，`--gpu high` 可寫入 Windows 圖形偏好（`HKCU\Software\Microsoft\DirectX\UserGpuPreferences`）並重新啟動。GUI 內的 `SERVO_MIN_DEG/MAX_DEG` 需與 Config.h 一致。
- PID 濾波：`f 輸出α D的α 積分上限` 設定 `outputAlpha`/`dAlpha`/`integralLimit`（α 需在 (0,1]、上限 >= 0，不清除積分，不存 NVS，重開機回到 `Config.h` 預設）。
- 伺服原點校正：`d a1 a2 a3` 直接設定三顆伺服的實際角度（不加 offset，限制在各自的 `minDeg~maxDeg`，進入「直接角度」模式、自動平衡暫停）；`o` 把目前的 `d` 角度寫入為水平原點（`offset = 角度 − neutral`，存進 ESP32 的 NVS 並在開機時載入，斷電保留；只在直接角度模式且 PCA9685 有連線時允許）；`o r` 還原成 `Config.h` 的 `SERVO_CAL_DEFAULT` 並清除儲存值。`l`（回水平）與自動平衡都以 `neutral + offset` 為水平角度。GUI 的「伺服角度與原點」卡片對應這三個指令（寫入原點要在 3 秒內按兩次確認）。
- 開迴路驗證模式（不讀球位置，沒接感測器也能跑）：`w 傾角 週期` 是圓錐傾斜掃掠
  （固定傾角、`phi` 以固定角速度繞圈，球會沿盤緣繞行），用來在調 PID 之前確認
  運動學、伺服方向與座標系符號；`v 振幅 週期` 是高度振盪（`theta=0`，高度走正
  弦），用來確認伺服是否快得足以把球彈起來。兩者都在 `loop()` 裡用 `millis()`
  算參考值（不經由 host 送指令，避免佔用已接近上限的序列埠頻寬），`l` 可離開。
  參數與週期下限在 `Config.h` 的 `SWEEP_*` / `HOP_*`；下指令時會用
  `poseWithinServoRange()` 檢查整段軌跡都不會被伺服角度上下限夾限。
  `applyTilt()` 會把實際送出的高度交給 `IRSensorArray::setHeight()`，所以振盪
  期間 ToF 的距離篩選窗會跟著平台走，離開振盪時自動還原。
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
- 2026-10-22：用實體裝置（COM4，暫時上傳含原始資料輸出的診斷韌體，測完已還原成正式版）調查「8×8 沒有資料」：序列埠與 GUI 解析皆正常（`DATA,8` 68 欄位完整），資料全為 0 是因為感測器在 8×8 下幾乎不回報目標。實測感測器視野內有約 4~13 cm 的近距離表面，4×4 偵測率約 76%，8×8 僅約 3~4%（多數區域狀態碼 255＝無目標）。排除項目：積分時間 10~50 ms 無改善、開機直接用 8×8 結果相同（非切換流程問題）。另外 8×8 的 `signal_per_spad` 約為 4×4 的 5~30 倍（中位數約 5000~30000 vs 約 900~1400），目前 `TOF_FILTER_MAX_STRENGTH`=1000 即使偵測到也會被濾掉；`TOF_FILTER_MIN_DISTANCE`=50 也會濾掉更近的目標。結論：此安裝距離請用 4×4；要用 8×8 需拉高感測器或重新驗證偵測距離，並依解析度分別設定強度上限。GUI 在沒有有效區域時顯示「沒有有效區域」提示。
- 2026-10-22（更正）：先前「8×8 近距離偵測不到、是感測器限制」的結論**不正確**。真正原因是 VL53L7CX 的銳化（sharpener，預設約 14%）把互相重疊的近距離目標（壓克力板與球靠得很近）標成狀態碼 12（被另一個目標模糊）而被篩選丟棄；8×8 區域更小、重疊更嚴重，所以幾乎全滅。實測（COM4，暫時上傳可即時調參與輸出所有目標的診斷韌體，測完已還原）：銳化 0% 時 4×4 近距離目標有效率 45%→99%（16/16 區域）、8×8 6%→100%（64/64 區域），各區域距離標準差約 1~1.5 mm；積分時間、解析度切換流程、串擾邊限皆非主因。已在 `Config.h` 新增 `TOF_SHARPENER_PERCENT = 0` 並於 `IRSensorArray::begin()` 設定（切換解析度後設定會保留）。
  - 現場量測（感測器在壓克力板下方、乒乓球在板上正中心）：有球與無球的距離分布幾乎相同，球只讓中間幾格增加約 10~40 mm；邊緣與四角有穩定的強反射（無球時 8×8 角落約 19~28 mm、強度 1 萬~3 萬 kcps/spad），經使用者確認是**實體障礙物擋住視野**（不是可抵銷的反射），需把感測器裝高或移開障礙，使 60°×60° 視野內只剩壓克力板。目前估測器以訊號強度找峰值，會被邊緣強反射拉偏（球在中心時 4×4 估到約 (0.38, 1.91)，中心為 (1.5, 1.5)）。
  - 待辦/建議：感測器裝高後重測（驗收標準：無球時邊緣區域距離應大於中心、無強度 >1000 的熱點）；軟體只能做「區域遮罩」（無球時找出被障礙擋住的區域並標為無效，避免估測被拉偏）；被擋住的區域球仍然看不到，所以有效偵測範圍會小於整塊壓克力板。背景相減對被擋住的區域無效。
- 2026-10-23：PCA9685 改接獨立的 I2C 匯流排（`Wire1`，SDA=18、SCL=19），VL53L7CX 維持在 `Wire`（SDA=22、SCL=21）。`Config.h` 的 `I2C_SDA_PIN/I2C_SCL_PIN/I2C_FREQ_HZ` 拆成 `TOF_I2C_*` 與 `SERVO_I2C_*`；`ServoMotor` 建構子新增 `TwoWire& bus`（預設 `Wire1`），探測與 `begin()` 都改走該匯流排；Adafruit 函式庫內部再呼叫的 `Wire1.begin()` 在匯流排已啟動時只會警告、不會改變腳位。host mock 測試新增「雙匯流排」情境（腳位/時脈正確、伺服匯流排失效時感測器照常輸出，共 1155 項），`pio run -e esp32` 通過。**需要實體接線配合：PCA9685 的 SDA 接 GPIO18、SCL 接 GPIO19，並與 ESP32 共地；尚未用實體 PCA9685 驗證。**
- 2026-10-23：GUI 的「傾斜測試」改為「伺服角度與原點」：三顆伺服各一組 +/-（步進 0.5°/1°/2°，滾輪與按住連發可用，限制在 `SERVO_MIN_DEG~SERVO_MAX_DEG`），調整時送 `d`；「寫入原點」（需直接角度模式，3 秒內再按一次確認）送 `o`，「還原」送 `o r`。韌體新增 `d`、`o` 指令與開機載入（見上方「伺服原點校正」）。傾斜指令 `t` 仍可從指令輸入框使用。host mock 測試新增 `Preferences` 與 d/o 情境（共 1177 項）、GUI 互動測試 33 項、短版隨機測試通過；`pio run -e esp32` 通過。**尚未用實體 PCA9685 驗證**：原點寫入的實際儲存與斷電保留需接上馬達後確認。
- 2026-10-23：開機預設改為「回水平、不自動平衡」：`setup()` 套用已儲存的原點後停在水平（`MODE_TILT`、θ=0），即使視野內有球也不會動，要輸入 `r`（或 GUI 的「自動平衡」）才開始 PID；`Config.h` 新增 `AUTO_BALANCE_AT_BOOT`（預設 false，設 true 則開機直接自動平衡）。GUI 的 `--demo` 模擬器同步改為開機水平。host mock 測試更新為 1180 項。
- 2026-10-23：反轉伺服輸出方向（正向角度原本使連桿向下）：只把 `Config.h` 的 `SERVO_CAL_DEFAULT` 三顆伺服的 `gain` 從 +1 改成 −1（`伺服角度 = neutral + gain × (模型角 − 水平模型角) + offset`），運動學（`PlatformKinematics`）完全沒動，水平時模型角差為 0，所以水平原點與已儲存的 offset 都不受影響。`d` 直接角度指令與 GUI 的伺服角度仍是實際的伺服度數（正向＝連桿向下，未反轉）。只用 host 假 PCA9685 輸出驗證（未動實體裝置、未上傳）：t 5 0 時腳 2 伺服 −8.03°、腳 1/3 各 +4.29°（原本相反）；在整個可達範圍內伺服角度 = neutral − (模型角 − 水平) + offset 且都在上下限內；假 PWM 計數與角度吻合；原點寫入/回水平不變；對照實驗確認 gain=+1 時新測試會失敗。host mock 測試共 1204 項。**上傳後請先用小角度實機確認方向，再放心使用。**
- 2026-10-23：球偵測改為「強度區塊」演算法（取代 log 拋物線峰值）。實測壓克力板是鏡面：軸心附近單一區域有 10^4~10^5 kcps/spad 的尖峰、其餘很弱，距離受 range-walk 影響不可靠且不隨球變化；球（漫反射）則是一片明顯的強度區塊（8x8 約 10^3.1~10^3.9）。感測器已架高、框架不在視野內。
  - `BallPositionEstimator` 重寫（見上方 Architecture）；`Config.h` 新增 `BALL_SPIKE_JUMP/RISE/MIN_PEAK/MIN_ZONES`，8x8 與 4x4 分開（4x4 `MIN_PEAK`=2.7：空板子去除尖峰後最強 10^2.54，球最弱區域 10^2.88；8x8 為 2.9）。強度篩選放寬（`TOF_FILTER_MAX_STRENGTH`=1000000、`MIN_DISTANCE`=10），尖峰交由估測器處理。
  - 驗證（實機 COM4，暫時上傳診斷韌體，馬達只在原點 ±10° 內、不自動平衡）：空板子水平與 7 種傾斜姿態共 8x8 138 幀、4x4 270 幀皆無誤報；球在中心 8x8 28/28 幀偵測到、位置 (3.34, 4.23) 標準差約 0.01 格，4x4 72/72 幀、(1.45, 2.00)。實錄幀用 host mock 重播（共 204 項測試通過），`pio run -e esp32` 通過。
  - 撤回：先前說「xtalk margin 可能有用」不對（那是校正前蓋很近的玻璃用的，板子在約 50 mm 外沒效果）。
  - 已知/待辦：球在板子正中央時估測不在視野中心（8x8 約 (3.34, 4.23)、4x4 約 (1.45, 2.00)；感測器軸心與板中心有偏移），需用目標點 `g x y` 校正；球偏離中心時的方向/比例尚未實機驗證（球在板上不要傾斜）；mm 換算用的距離有 range-walk 偏差，比例只是近似；感測器位置變動後要重驗門檻。開發板目前可能仍是診斷韌體，請自行上傳正式版。
- 2026-10-23：`Config.h` 的 `SENSOR_FLIP_X` 改為 true（X 軸方向反了）。GUI 熱圖是感測器座標、`phi` 是平台座標，所以箭頭改為反向套用旋轉/翻轉後再畫；`heatmap.py` 新增 `SENSOR_ROTATION_DEG/FLIP_X/FLIP_Y` 常數，**需與 Config.h 一致**。尚未實機確認 X 方向。
- 2026-10-23：GUI 的「PID 增益」與「目標點與高度」卡片新增步進按鈕（與伺服卡片一致，按一下循環切換）：PID 的 Kp/Ki/Kd 共用步進 0.001/0.01/0.1（預設 0.01），目標點與高度為 0.1/0.5/1/2/5 mm（預設 1，行為不變）。只用 `--demo --snapshot` 檢查版面，未做點擊測試。
- 2026-10-26：GUI 新增「濾波與積分」卡片，可調輸出 α、D α、積分上限（步進 0.01/0.05/0.1/0.5）。韌體新增 `f` 指令，STAT 尾端新增 `outAlpha,dAlpha,integLimit` 三欄（`STAT_FIELDS` 已同步）。右側欄改為兩欄、左欄縮窄（設計尺寸 1500x760）。驗證：`pio run -e esp32` 通過、host mock 測試通過、GUI 互動測試 38 項通過（含新卡片）；`f` 指令尚未用實體裝置驗證。
- 2026-10-27：開發環境從 Windows 換到 Linux 後，`heatmap.py` 抓不到開發板（原本的 COM4）。查明原因：裝置本身的權限沒問題（CH340，`/dev/ttyUSB0`，核心預設給 `crw-rw-rw-`，不需加入 `dialout` 群組）；問題在 `find_port()` 的自動偵測——(1) 關鍵字只比對 `"USB-SERIAL"`（含連字號），但 Linux 下 pyserial 對 CH340 回報的描述是 `"USB Serial"`（無連字號），比對不到；(2) 比對失敗就 fallback 到 `ports[0].device`，而這台機器上 `/dev/ttyS0`~`/dev/ttyS31`（Linux 內建的虛擬序列埠）排在列表最前面，於是選到錯誤的埠。修正 `find_port()`：關鍵字加入 `"USB SERIAL"`、新增 VID:PID 比對（CH340 1A86:7523、CP210x 10C4:EA60、FTDI 0403:6001），都比對不到時優先選 `ttyUSB*`/`ttyACM*` 而非任意第一個埠。只改了 host 端腳本，沒有動韌體/MCU。驗證：`find_port()` 單獨呼叫回傳 `/dev/ttyUSB0`、`ruff format`/`ruff check` 通過（既有的 `EXE002` shebang 警告為修改前就存在，與此次改動無關）。
- 2026-11-11：`TOF_FILTER_MIN_DISTANCE`/`TOF_FILTER_MAX_DISTANCE`（感測器區域有效性的距離篩選窗）原本是固定值，不會隨平台高度調整。改成 `IRSensorArray::setHeight(heightMm)`：以 `PLATFORM_DEFAULT_HEIGHT_MM`（82.6 mm）量測到的 10~500 mm 為基準，依目前高度與基準高度的差距整段平移（感測器固定在機構上方往下看，使用者實測確認 `height` 數字增加＝平台降低＝感測器到平台距離變遠，兩者視為 1:1 mm 對 mm 的近似）。`main.cpp` 在 `setup()` 與 `h` 指令改高度後都會呼叫 `setHeight()`。驗證：`pio run -e esp32` 通過；只改了距離篩選窗的計算方式，沒有動運動學或 PID。尚未用實體裝置在不同高度下驗證窗口平移後的偵測率。
- 2026-11-11：新增兩個開迴路驗證模式（為後續的「連續彈球」與「球繞圈」閉迴路
  模式鋪路）。架構決定：繞圈的閉迴路版本是「現有 PID + 會動的目標點」而不是新
  控制器，所以水平（目標點軌跡）與垂直（平台高度）拆成兩個正交的軸，不把組合
  塞進 `enum Mode`；開迴路的 `MODE_SWEEP` 不會變成死碼，它之後是永久的診斷工具
  （PID 調壞時用它確認機構與座標系仍正常）。彈球的閉迴路版本會沿用這次替
  `applyTilt()` 加的高度參數，只是把「時間的函式」換成狀態機的輸出。
  - `Config.h` 新增 `SWEEP_TILT_DEG/SWEEP_PERIOD_S/SWEEP_MIN_PERIOD_S` 與
    `HOP_AMPLITUDE_MM/HOP_PERIOD_S/HOP_MIN_PERIOD_S`；`main.cpp` 的 `enum Mode`
    末端加 `MODE_SWEEP`/`MODE_HOP`、`applyTilt()` 加高度參數（預設沿用全域
    `height`，既有呼叫行為不變）、新增 `modeName()`、`rawServoAngle()`、
    `poseWithinServoRange()`、`usableSweepTilt()`、`usableHopAmplitude()`。
  - **發現（既有問題，非這次引入）**：`computeMaxTilt()` 與 `minHeight()/
    maxHeight()` 只看機構可達性，沒有考慮伺服的 `minDeg/maxDeg`。預設高度
    82.6 mm 下 IK 可到 16.3 度，但全方向都不被 `constrain()` 夾限的最大傾角只有
    **12.0 度**；`t 15 0` 這類指令有 201/360 個方向會靜默夾限（最糟超出 9.19
    度）。高度同理：IK 說 60.5~102.3 mm，伺服不夾限的振幅上限只有 ±14.7 mm。
    既有的 `t`/`h` 指令維持原行為沒動，但新的 `w`/`v` 會連續停在極限附近，所以
    改用伺服角度範圍當守衛，被拒絕時會回報目前高度下實際可用的值。
  - 週期下限拆成兩個：`CONTROL_PERIOD_MS`=20 時，掃掠 0.5 秒只有 25 步/圈（每步
    14 度），再短會一格一格跳；振盪是單軸來回，步數可以少一些所以下限 0.2 秒。
  - STAT 欄位完全沒動（`targetX/targetY/height` 本來就在裡面）。`height` 刻意
    維持「基準高度」而非瞬時高度，因為 GUI 的高度 stepper（`heatmap.py`）拿它
    當加減基準，回報振盪值會讓基準抖動；代價是 GUI 看不到振盪波形。
  - `heatmap.py` 同步：`MODE_NAMES` 補兩個名稱、模式顯示的上界從硬寫死的 `3`
    改成 `len(MODE_NAMES)`（之前加模式會漏改而顯示「—」）、新增
    `SWEEP_MIN_PERIOD_S`/`HOP_MIN_PERIOD_S`、`--demo` 模擬器支援 `w`/`v`
    （`w` 會讓球真的繞圈；`v` 只切換模式，因為 demo 的 `_servos()` 不模擬高度）。
  - 驗證：`pio run -e esp32` 成功無警告；host 端用假的 Arduino 標頭編譯真實的
    `PlatformKinematics`，對掃掠/振盪做 108 項檢查全數通過（phi 覆蓋整圈且步進
    連續、被接受的參數全程不夾限、被拒絕的參數確實會夾限）；demo 模擬器互動
    17 項通過；`--demo --snapshot` 版面正常。測試腳本放在暫存目錄，未納入 repo。
  - **尚未實機驗證**：`w`/`v` 都還沒上傳到開發板跑過，第一次請用小傾角/小振幅
    （例如 `w 3 3`、`v 2 1`）確認方向與機構沒問題再加大。
- 2026-11-11（接續）：併入 `11a1d29`（ToF 距離篩選窗依平台高度平移）後調整開迴路
  模式。原本 `IRSensorArray::setHeight()` 只在 `setup()` 與 `h` 指令呼叫，但
  `MODE_HOP` 的平台高度每個控制週期都在變，篩選窗會停在基準高度不動。改成由
  `applyTilt()` 用實際送出的高度呼叫 `setHeight(h)`——它是所有會改變平台高度的
  路徑唯一都會經過的地方，所以離開振盪（`l` / `t` / PID）時也會自動還原；`a`/`d`
  直接角度模式下平台高度無從得知，另外明確還原成基準高度。`h` 指令原本的呼叫
  保留（非 `MODE_TILT` 時不會走 `applyTilt()`，仍然需要）。
  - 記錄一個 `setHeight()` 的既有行為：下緣是 `max(0, TOF_FILTER_MIN_DISTANCE +
    delta)`，而 `TOF_FILTER_MIN_DISTANCE` 只有 10，所以平台低於基準 10mm 以上時
    下緣會被夾在 0、不再跟著走。只是讓下緣更寬鬆（不會濾掉板面），不影響上緣。
  - 驗證：`pio run -e esp32` 成功無警告；新增 53 項篩選窗檢查（振盪期間板面全程
    在窗內、未觸及夾限時餘裕完全不變、跟著走的餘裕不低於不跟著走、離開後還原、
    夾限行為）全數通過；先前的 108 項掃掠/振盪檢查與 demo 互動測試無回歸。
- 2026-11-11（接續）：把散落各處的容許值收進 `Config.h`（全部為等值替換，行為
  不變）。原本硬編在四個地方：`SERVO_CAL_DEFAULT` 裡重複三次的 `40.0f, 140.0f`、
  `a` 指令與 `ServoMotor::setAngle()` 的 `0~180`、`computeMaxTilt()` 的搜尋上限
  `25`／收斂門檻 `0.05`／方向取樣 `10`／安全餘裕 `0.5`、以及開迴路模式的掃描
  步進。新增 `SERVO_RANGE_MIN/MAX_DEG`、`SERVO_HARD_MIN/MAX_DEG`、
  `PLATFORM_MIN/MAX_HEIGHT_MM`、`KINEMATICS_TILT_*`、`SWEEP_PHI_CHECK_STEP_DEG`、
  `RANGE_SCAN_*`（見上方 Architecture）。
  - 伺服角度刻意分成兩組：工作範圍（40~140，運動學與 `d` 指令）與物理極限
    （0~180，只有校正用的 `a` 指令和脈寬換算會用到）。原本兩者都是魔術數字，
    混在一起看不出差別。`ServoMotor` 的脈寬換算同步改成
    `(angle - SERVO_HARD_MIN_DEG) / span`，`SERVO_HARD_MIN_DEG`=0 時與原式等價。
  - 平台高度新增 `minUsableHeight()/maxUsableHeight()`，與運動學取交集。這順帶
    修掉一個 GUI/韌體不一致：`heatmap.py` 寫 61~102，但韌體原本只檢查運動學的
    60.5~102.3，GUI 擋得比韌體嚴。現在兩邊都是 61~102，GUI 的常數也加上對應的
    Config 名稱註解。
  - 驗證：`pio run -e esp32` 成功無警告；新增 32 項等價性檢查（工作範圍與物理
    極限數值未變、脈寬換算在 0/40/90/140/180 度與舊式相同、`computeMaxTilt()`
    仍為 16.3 度、可用傾角仍為 12.0 度、可用振幅仍為 ±14.5mm、常數間的關係如
    工作範圍落在物理極限內）全數通過；既有的 108 + 53 + demo 測試無回歸。
- 2026-11-11（接續）：平台高度下限從 60.5 mm 降到 41 mm。原因是
  `minUsableHeight()` 原本用 `PlatformKinematics::minHeight()`，而那是個**保守的
  解析式估計**，不是求解器的真正極限——它算的是「擺臂完全水平朝外」那一刻的
  高度（`sqrt(l1² - (lb + l2 - lp)²)` = 60.46），但 `solveSpherical()` 在更低的
  高度仍然有解（實測一路到 0 都有解；連桿可及範圍是 17~107 mm，h=0 時轉軸到球
  接頭才 31.25 mm，根本沒超出）。所以把 `PLATFORM_MIN_HEIGHT_MM` 改小沒有用，
  `max()` 會被 60.46 蓋掉。
  - 改成從預設高度往兩邊掃描，找「水平姿態下三顆伺服都不被 `modelToServo()`
    夾限」的範圍，再以 `PLATFORM_MIN/MAX_HEIGHT_MM` 當硬性邊界往內收。目前校正
    值（NVS offset 23/23/27）下實際範圍是 **41.0 ~ 96.9 mm**；純以伺服角度算
    可到約 28 mm，41 是 Config 的硬性下限擋住的。
  - 上限順帶修掉一個舊問題：原本 `h 100` 會被接受，但水平時伺服角度就要 143.8
    度（超出 140），平台連水平都站不住且三腳夾限程度不同會變歪。現在上限
    96.9 mm，`h 100` 會被拒絕。
  - 掃描用整數步數算高度（`PLATFORM_DEFAULT_HEIGHT_MM - i * HEIGHT_SCAN_STEP_MM`）
    而不是反覆加減浮點數，否則累積誤差會讓掃到硬性邊界時回傳 41.000004——顯示
    成 41.0 但輸入 `h 41` 會被拒絕。`h` 指令的比較另外留半個掃描步進的容差。
  - **注意**：高度壓到 60 mm 以下時擺臂會低於轉軸平面（41 mm 時模型角約 65 度，
    預設高度是 88 度），`minHeight()` 原本大概就是想避開這個區域。調低前請先用
    `d` 指令手動確認機構不會干涉。`PLATFORM_MIN_HEIGHT_MM` 就是用來擋這個的。
  - 另：`s` / `h` 回報的「最大傾角」仍是機構值（`MAX_TILT_DEG` 夾過的
    `computeMaxTilt()`），不是伺服實際允許的角度——例如 41 mm 時回報 15 度但
    伺服只到 11 度。`w` 指令用的是嚴格的伺服檢查，`t` 仍用舊的機構判斷。
  - 驗證：`pio run -e esp32` 無警告；新增 15 項高度範圍檢查（範圍內每一點水平
    都不夾限、邊界外確實不可用、顯示值可直接輸入、掃到硬性邊界回傳精確值）；
    既有 108 + 53 + 32 項全部重新編譯後無回歸；實機確認 `h 30`/`h 100` 被拒、
    `h 41`/`h 96.9`/`h 60` 被接受。
- 2026-11-11（更正）：**撤回前一則「平台高度下限可以降到 41 mm」的結論，那是
  錯的**，而且當時已經上傳到開發板。兩個錯誤：
  - **高度方向說反了**：`h` 是平台中心離基座的高度，`maxHeight()` 是連桿完全
    伸直（`l1+l2`=107）＝平台**最高**，所以 **h 越大＝平台越高**。先前寫成
    「數字越大＝平台越低」是照抄 merge 進來的 `11a1d29` 註解而沒有驗證。那段
    註解的推理（「感測器裝在上方往下看」＋「高度增加＝平台降低」）兩處都不對，
    不過 `setHeight()` 的公式（+delta）剛好仍然正確——感測器在壓克力板**下方**，
    平台升高＝離感測器更遠。
  - **`minHeight()` 不是保守估計，是機構奇異點**：模型角在 h≈60.46 達到極大值
    （89.9 度，擺臂轉到水平），再低就折回另一個解分支。實測同一個模型角對應兩
    個高度：64.63 度 →{80.6, 42}、76.20 度 →{71.5, 50}。我們只能控制伺服角度，
    所以從預設高度下 `h 41` 送出的角度同時也是 h≈80.6 的角度，平台會停在上分支
    的 80.6 mm，**根本到不了 41**。低於奇異點的高度需要讓擺臂穿過水平姿態翻到
    另一分支，不是下指令能做到的。
  - 修正：新增 `heightFloor()` = max(奇異點, `PLATFORM_MIN_HEIGHT_MM`)，
    `minUsableHeight()` 從它開始掃描。可用範圍回到 **60.5 ~ 96.9 mm**（上限仍是
    上一輪修好的伺服限制，`h 100` 會被拒絕）。
  - 依需求把 minHeight 參數化：`Config.h` 新增
    `PLATFORM_KINEMATIC_MIN_HEIGHT_MM`，設 0（預設）表示自動用
    `PlatformKinematics::minHeight()` 算，只有在實際機構與 Config 的尺寸不符、
    量測出不同的奇異點位置時才填實測值覆寫。`PLATFORM_MIN_HEIGHT_MM` 維持原本
    「再往內收的硬性邊界」語意，不會讓範圍超出奇異點。
  - `heatmap.py` 的 `HEIGHT_MIN_MM` 同步改為 60.5（原本跟著改成 41）。
  - 驗證（全部在 host 端，未動馬達、未上傳）：`pio run -e esp32` 無警告；新增
    15 項檢查（maxHeight 即連桿伸直、模型角極大值落在 minHeight、奇異點以下的
    高度全都在上分支有同角度對應點 9/9、修正後下限不低於奇異點、可用範圍內模型
    角對高度嚴格單調＝未跨越奇異點、41 mm 已被拒絕）；既有 108 + 53 + 32 項
    重新編譯後無回歸。
  - **開發板上目前仍是錯誤版本（下限 41）**，重新上傳前請勿用 `h` 設低於 60.5
    的值——那會讓平台跑到上分支的另一個高度，而不是你以為的低位置。
- 2026-11-11（接續）：依需求把高度與伺服角度的上下限收斂成 **Config.h 單一來源，
  其他模組一律尊重**。起因是使用者把下限改成 50 卻「還是卡在 60.5」——實際擋住
  的是 `heatmap.py` 自己抄的 `HEIGHT_MIN_MM`，同一個限制散在 Config.h、main.cpp
  的推導邏輯、GUI 常數三個地方。
  - **高度**：`PLATFORM_MIN/MAX_HEIGHT_MM` 成為唯一來源，`minUsableHeight()/
    maxUsableHeight()` 直接回傳它們，不再被運動學或伺服掃描覆寫（移除
    `heightFloor()`、`PLATFORM_KINEMATIC_MIN_HEIGHT_MM` 與掃描邏輯）。目前設定
    50.0 ~ 96.9 mm。
  - 改以**開機警告**取代阻擋：`warnHeightRange()` 在設定低於機構奇異點、或上下
    限的水平姿態超出伺服角度時印警告但不阻擋——無法自動判斷 Config 的機構尺寸
    是否與實體相符，所以由使用者決定。實機確認警告會印出。
  - **伺服安全角度**：新增 `SERVO_SAFE_MIN/MAX_DEG`，由
    `ServoMotor::setAngle()` 夾限，是所有模組（含 `a`/`d` 指令與未來新增的模式）
    的最後防線。角度設定現在分三層：`SERVO_HARD_*`（物理量程，只用於脈寬刻度）
    ⊇ `SERVO_SAFE_*`（安全角度）⊇ `SERVO_RANGE_*`（運動學工作範圍）。
  - **GUI 不再抄設定**：STAT 末端新增 `hMin,hMax,servoMin,servoMax` 四欄
    （23 → 27 欄），`heatmap.py` 的 `App.limit()` 一律以 STAT 為準，原本的常數
    降級成「尚未連線時的後備值」。改 Config.h 後 GUI 自動跟著變，不需要改
    Python。新增一項 Config.h↔GUI 後備值的一致性檢查（解析兩個檔案比對）。
  - 驗證：`pio run -e esp32` 無警告；新增 20 項（Config 為唯一來源、安全角度對
    9 種輸入都夾進範圍、三層角度關係、設定範圍內無姿態被夾限）＋ GUI 11 項
    （27 欄、`limit()` 的 STAT/後備值切換、韌體改設定時 GUI 跟著變）＋ 一致性
    4 項；既有 108 + 53 + 32 項無回歸。已上傳，實機確認 `h 50`/`h 55`/`h 96.9`
    接受、`h 49`/`h 97` 拒絕、STAT 27 欄且四個限制值正確、開機警告正常。
  - **保留的已知事實**：`h 50` 雖然被接受，但 50 mm 在奇異點以下，平台實際會停
    在上分支的對應高度（模型角 76.2 度 ↔ 71.5 mm），不是 50 mm。要真的降低平台
    必須改機構尺寸（`Config.h` 的 lp/l1/l2/lb），例如連桿 l1 從 62 縮到 51.9 可
    讓奇異點降到 50 mm，代價是最高高度從 102.3 降到 91.7。
- 2026-11-11（更正 2）：**「60.5 mm 是機構奇異點」也是錯的**，使用者實測指出機構
  在該點沒有折返（搖桿與推桿並未共線），反轉的是伺服角度。查 `solveVector()`
  後確認這是**程式的分支選擇問題**：
  - `PlatformKinematics.cpp` 原本寫 `double c13 = sqrt(z1), c23 = sqrt(z2),
    c33 = sqrt(z3);`——`c?3` 是擺臂末端關節的 Z 座標，一律取正根，等於強制擺臂
    末端不得低於基座平面。`th = atan2(..., c13)` 因此被限制，模型角上限正好是
    90 度（擺臂水平，Z=0）。
  - 低於該高度時求解器不是沿著負的 Z 延伸，而是跳到鏡像解，所以模型角由升轉
    降、伺服角反向。**擺臂水平（Z=0）在機構上沒有任何特殊性，不是力學死點**
    （死點是擺臂與連桿共線），先前說「機構奇異點」「過中心機構會卡死」都不對。
  - 修正：`solveVector()/solveSpherical()/computeMaxTilt()` 新增
    `armBelowBase` 參數（預設 false，行為完全不變），為 true 時 `c?3` 取負根，
    模型角可超過 90 度。`Config.h` 新增 `ALLOW_ARM_BELOW_BASE`（預設 false），
    `main.cpp` 的 `armBelowBaseFor(h)` 在開關開啟且高度低於交界時套用。
  - 實測延伸結果（host 計算，含開發板的 NVS offset 23/23/27）：模型角在
    40~62 mm 之間隨高度下降嚴格遞增（88.04 → 118.45），跨越交界處無跳變；
    伺服角 86.9 → 56.5 度，**全程在安全範圍 40~140 內，不需要重新安裝舵機**。
  - `PLATFORM_MIN_HEIGHT_MM` 暫設 62.0（交界 60.46 再留約 2 度餘裕），開關關閉
    時的安全值。開啟 `ALLOW_ARM_BELOW_BASE` 後可往下放寬到約 40。
  - **開啟前必須確認擺臂往下擺不會撞到底板或結構件**，可用 `d` 指令手動轉到對應
    的伺服角度（例如 40 mm 對應腳 1 約 56.5 度）檢查。
  - 驗證（全部 host 端，未動馬達、未上傳）：`pio run -e esp32` 無警告；新增 11
    項（預設關閉時模型角/computeMaxTilt 完全不變、負根＝對 90 度鏡射、負根在
    40~交界之間嚴格遞增、跨交界整體單調且無跳變、伺服角全在安全範圍、對照組
    確認不切負根會反轉）；既有 108 + 53 + 32 + 22 項無回歸。
- 2026-11-11（工具）：設定 mypy。mypy 2.4.0 裝在使用者的 `~/.local/bin`（不在
  `script/requirements.txt`），編輯器的擴充套件一直以預設值在這個 repo 執行
  （`.mypy_cache/` 為證），但 repo 內沒有任何設定檔。實際只有 3 條錯誤，全是
  pyserial 缺型別 stub，程式本身乾淨。新增 `mypy.ini`（`files = script`、
  `[mypy-serial.*] ignore_missing_imports`），並把漏掉的 `.mypy_cache` 加進
  `.gitignore`（`.ruff_cache` 本來就有）。驗證：從 repo 根、`script/`、指定檔案
  路徑三種 cwd 執行皆為 `Success: no issues found`；`ruff` 不受影響。
- 2026-11-11（高度上下限的成因）：釐清「為什麼高度調不上去/調不下去」。
  - **上限有三層**（見上方 Architecture）。使用者回報「大於 100 會說無法平衡」
    即第二層：`computeMaxTilt()` 在 101.67 mm 歸零，`h` 指令的
    `if (newMax <= 0)` 就印「這個高度無法保持水平」。真正的機構極限是
    102.33 mm（連桿 62 + 擺臂 45 = 107 完全伸直，水平跨距固定 31.25）。
  - **下限由伺服安全角度決定**。把 `SERVO_SAFE_*`/`SERVO_RANGE_*` 從 40~140
    放寬到 20~160 後，可用高度變成 20.1 ~ 102.2 mm；`h 20` 只差 0.1 mm 會被夾限
    （模型角 154.99 度對應伺服 19.9 度，低於安全下限 20）。放寬前的下限是
    30.3 mm。
  - `PLATFORM_MIN/MAX_HEIGHT_MM` 目前為 20 / 100，`Config.h` 的註解已改成記錄這
    三層的實際數字（原註解還停在 50 / 96.9 的舊值）。
