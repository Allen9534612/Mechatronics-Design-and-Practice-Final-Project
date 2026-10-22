# 機電系統設計與實務　期末專題 (Ball Balancing)

參考 BaBot 設計的史都華平台控制專案，目標是在平台上平衡和擊打桌球。

## 建置環境與依賴

### 硬體設備

- NodeMCU-32s
- VL53L7CX
- MG996R Servo Motor

### 軟體環境

- Python 3.13
- Python Package:
    - pyserial
    - pygame
    - clang-format
    - ruff
    
- Visual Studio Code
- VS Code Extension:
    - PlatformIO
    - clangd 
    - Ruff

## 安裝步驟

1. 複製專案到本地端：
   ```bash
   git clone git@github.com:Allen9534612/Mechatronics-Design-and-Practice-Final-Project.git

2. 安裝 Python 函式庫：
   ```bash
   pip install pygame pyserial clang-format ruff

3. 安裝 VS Code 插件：
    - [PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
    - [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
    - [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)

4. 上傳並燒錄韌體程式
    (Via PlatformIO)

## 排版工具使用

本專案程式使用 `Google` 風格撰寫

- 可在 VS Code 使用命令列排版。另外，儲存檔案時也會自動排版。
  ```bash
  ruff check --fix script/
  ```

  ```bash
  Get-ChildItem -Path "src", "lib", "include" -Include *.cpp, *.c, *.h -Recurse | ForEach-Object { clang-format -i -style=file $_.FullName }
  ```

- 在檔案按下 `Shift + Alt + F` 亦可排版。
- 鼠標懸浮在 Warnings 時，按下 `Ctrl + .` 可檢視 Linter 提供的快速修正選項。