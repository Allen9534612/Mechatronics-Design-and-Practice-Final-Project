import sys

import pygame
import serial
import serial.tools.list_ports

BAUD_RATE = 115200
WINDOW_SIZE = 640
MAX_DISTANCE = 500


def find_nodemcu_port():
    print("正在尋找 NodeMCU 序列埠...")
    ports = serial.tools.list_ports.comports()

    keywords = ["CP210", "CH340", "UART", "Silicon Labs", "USB-SERIAL"]

    for port in ports:
        if any(keyword.upper() in port.description.upper() for keyword in keywords):
            print(f"➜ 成功自動偵測到裝置: {port.device} ({port.description})")
            return port.device

    if ports:
        print(
            f"➜ 找不到典型特徵，嘗試使用唯一可用的裝置: {ports[0].device} ({ports[0].description})"
        )
        return ports[0].device

    return None


COM_PORT = find_nodemcu_port()

if not COM_PORT:
    print("\n[錯誤] 找不到任何連線的 COM Port。")
    sys.exit()

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"➜ 已成功開啟 {COM_PORT}，準備接收數據...")
except serial.SerialException as e:
    if "PermissionError" in str(e) or "存取被拒" in str(e):
        print(f"[錯誤] 無法開啟 {COM_PORT}：存取被拒")
    else:
        print(f"\n[錯誤] 無法開啟 {COM_PORT}: {e}\n")
    sys.exit()

# 初始化 Pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("VL53L7CX 可視化介面 (按 4 或 8 切換)")
font = pygame.font.SysFont(None, 24)


def distance_to_color(dist):
    if dist <= 0:
        return (30, 30, 30)
    ratio = min(max(dist / MAX_DISTANCE, 0), 1)
    r = int(255 * (1 - ratio))
    b = int(255 * ratio)
    return (r, 50, b)


running = True
current_grid_size = 4  # 預設變數，會隨著 ESP32 傳來的資料更新
cell_size = WINDOW_SIZE // current_grid_size

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                # print(f"偵測到按鍵，按鍵代碼: {event.key}", flush=True)

                if event.key == pygame.K_c:
                    print("➜ 收到關閉指令，結束程式...", flush=True)
                    running = False

                elif event.key == pygame.K_4 or event.key == pygame.K_KP4:
                    print("➜ 發送切換指令: 4x4 (60Hz)", flush=True)
                    ser.write(b"4")
                    ser.flush()
                elif event.key == pygame.K_8 or event.key == pygame.K_KP8:
                    print("➜ 發送切換指令: 8x8 (15Hz)", flush=True)
                    ser.write(b"8")
                    ser.flush()

        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line.startswith("DATA,"):
                data = line.split(",")

                if len(data) >= 4:
                    try:
                        received_grid_size = int(data[1])

                        expected_length = (
                            1 + 1 + (received_grid_size * received_grid_size) + 2
                        )

                        if len(data) == expected_length:
                            current_grid_size = received_grid_size
                            cell_size = WINDOW_SIZE // current_grid_size

                            screen.fill((0, 0, 0))

                            idx = 2
                            for y in range(current_grid_size):
                                for x in range(current_grid_size):
                                    dist = float(data[idx])
                                    idx += 1

                                    color = distance_to_color(dist)
                                    rect = pygame.Rect(
                                        x * cell_size,
                                        y * cell_size,
                                        cell_size,
                                        cell_size,
                                    )
                                    pygame.draw.rect(screen, color, rect)
                                    pygame.draw.rect(screen, (50, 50, 50), rect, 1)

                                    if dist > 0:
                                        text = font.render(
                                            f"{int(dist)}", True, (255, 255, 255)
                                        )
                                        screen.blit(
                                            text, (x * cell_size + 5, y * cell_size + 5)
                                        )

                            center_x = float(data[-2])
                            center_y = float(data[-1])

                            if center_x != -1.0 and center_y != -1.0:
                                pixel_x = int((center_x + 0.5) * cell_size)
                                pixel_y = int((center_y + 0.5) * cell_size)
                                radius = int(cell_size * 0.1)
                                pygame.draw.circle(
                                    screen,
                                    (0, 255, 0),
                                    (pixel_x, pixel_y),
                                    radius,
                                    3,
                                )

                            pygame.display.flip()

                    except ValueError:
                        pass

except KeyboardInterrupt:
    print("使用者手動中斷程式。")

finally:
    ser.close()
    pygame.quit()
