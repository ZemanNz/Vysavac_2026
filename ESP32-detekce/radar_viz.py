import pygame
import serial
import sys
import struct
import math

# --- Konfigurace ---
PORT      = '/dev/ttyUSB0'
BAUDRATE  = 921600
FADE_MS   = 600
BG_COLOR  = (15, 15, 15)

WINDOW_SIZE   = 900
PHYSICAL_SPAN = 1500.0   # mm zobrazeno v okně (250 navíc na každé straně)
SCALE         = WINDOW_SIZE / PHYSICAL_SPAN
OFFSET_MM     = 250      # arena X=0 je 250 mm od levého okraje okna

pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("LIDAR SLAM – Robot v aréně 1×1 m")
clock = pygame.time.Clock()

try:    font = pygame.font.SysFont("Courier", 16, bold=True)
except: font = pygame.font.SysFont(None, 20)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"[OK] Připojeno na {PORT}")
except Exception as e:
    print(f"[CHYBA] {e}"); sys.exit(1)

# Stav robota – posílá ho ESP32 každý snímek
robot_x   = 500.0
robot_y   = 500.0
robot_hdg = 0.0   # stupně, CW od severu

points    = []  # (x, y, timestamp)  – globální mm
lines     = []  # (x1,y1,x2,y2,col,timestamp) – globální mm
opponents = []  # (x, y, timestamp)

def to_screen(x_mm, y_mm):
    """Převod globálních mm → pixely. Y+ = nahoru (север)."""
    sx = int((x_mm + OFFSET_MM) * SCALE)
    sy = int(WINDOW_SIZE - (y_mm + OFFSET_MM) * SCALE)
    return (sx, sy)

def draw_scene(surf):
    surf.fill(BG_COLOR)

    # Jemná orientační mřížka (po 100 mm)
    for dm in range(-2, 13):
        v = dm * 100
        color = (25, 25, 25) if (dm < 0 or dm > 10) else (40, 40, 40)
        pygame.draw.line(surf, color, to_screen(v, -250), to_screen(v, 1250), 1)
        pygame.draw.line(surf, color, to_screen(-250, v), to_screen(1250, v), 1)

    # Pevné ohraničení arény 1×1 m (hrubá čára)
    arena = [to_screen(0,0), to_screen(1000,0), to_screen(1000,1000), to_screen(0,1000)]
    pygame.draw.lines(surf, (180, 180, 180), True, arena, 3)

    # ---- Robot: modrý čtverec 30×30 cm ----
    half = 150.0  # 150 mm = polovina 30 cm
    rad  = math.radians(robot_hdg)
    # Rohové body v lokálním systému (CW rotace)
    corners_local = [(-half,-half),(half,-half),(half,half),(-half,half)]
    corners_global = []
    for (lx, ly) in corners_local:
        gx = robot_x + lx*math.cos(rad) + ly*math.sin(rad)
        gy = robot_y - lx*math.sin(rad) + ly*math.cos(rad)
        corners_global.append(to_screen(gx, gy))
    pygame.draw.polygon(surf, (0, 120, 255), corners_global, 3)

    # Tečka středu
    pygame.draw.circle(surf, (0, 200, 255), to_screen(robot_x, robot_y), 5)

    # Směrová šipka (35 cm dopředu = +Y lokálně)
    arr_len = 350.0
    ax = robot_x + arr_len * math.sin(rad)   # forward = local +Y → global: sin(hdg), cos(hdg)
    ay = robot_y + arr_len * math.cos(rad)
    pygame.draw.line(surf, (0, 255, 200),
                     to_screen(robot_x, robot_y), to_screen(ax, ay), 3)

    # Info text
    info = font.render(f"X={int(robot_x)} Y={int(robot_y)} H={int(robot_hdg)}°", True, (120,120,120))
    surf.blit(info, (10, 10))

buffer  = bytearray()
running = True

print("Čekám na data z ESP32 (0xBB 0x55)…")

while running:
    now = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
    except OSError:
        print("[CHYBA] Zařízení odpojeno!")
        running = False; break

    # --- Parsování paketu ---
    while len(buffer) >= 5:
        idx = buffer.find(b'\xBB\x55')
        if idx == -1:
            buffer = buffer[-1:]; break
        elif idx > 0:
            buffer = buffer[idx:]; continue

        if len(buffer) < 4: break
        msg_type = buffer[2]
        msg_len  = buffer[3]
        total    = 4 + msg_len + 1  # header(4) + payload + checksum

        if len(buffer) < total: break

        payload  = buffer[4 : 4 + msg_len]
        checksum = buffer[4 + msg_len]
        calc_cs  = (msg_type + msg_len + sum(payload)) & 0xFF

        if calc_cs == checksum:
            if msg_type == 0 and msg_len == 4:
                # Zelená tečka (globální souřadnice)
                x, y = struct.unpack('<hh', payload)
                if -250 <= x <= 1250 and -250 <= y <= 1250:
                    points.append((x, y, now))

            elif msg_type == 1 and msg_len == 8:
                # Primka (globální souřadnice)
                x1, y1, x2, y2 = struct.unpack('<hhhh', payload)
                col = (255, 220, 0) if len(lines) == 0 else (180, 0, 255)
                lines.append((x1, y1, x2, y2, col, now))

            elif msg_type == 2 and msg_len == 6:
                # Nový snímek → aktualizace pozice robota + smazání starých primek
                rx, ry, hd = struct.unpack('<hhh', payload)
                if 0 <= rx <= 1000 and 0 <= ry <= 1000:
                    robot_x   = float(rx)
                    robot_y   = float(ry)
                    robot_hdg = float(hd)
                lines.clear()
                opponents.clear()

            elif msg_type == 3 and msg_len == 4:
                # Soupeř (globální souřadnice)
                ox, oy = struct.unpack('<hh', payload)
                if -250 <= ox <= 1250 and -250 <= oy <= 1250:
                    opponents.append((ox, oy, now))

        buffer = buffer[total:]

    # --- Kreslení ---
    draw_scene(screen)

    # Zelené tečky (fade)
    live_pts = []
    for (x, y, ts) in points:
        age = now - ts
        if age < FADE_MS:
            live_pts.append((x, y, ts))
            intens = max(0, 255 - int(age / FADE_MS * 255))
            pygame.draw.circle(screen, (0, intens, 0), to_screen(x, y), 3)
    points = live_pts

    # Primky stěn (krátká životnost – nahrazují se každý snímek)
    live_lines = []
    for (x1,y1,x2,y2,col,ts) in lines:
        if now - ts < 200:
            live_lines.append((x1,y1,x2,y2,col,ts))
            pygame.draw.line(screen, col, to_screen(x1,y1), to_screen(x2,y2), 4)
    lines = live_lines

    # Soupeř
    live_opp = []
    for (ox, oy, ts) in opponents:
        if now - ts < 300:
            live_opp.append((ox, oy, ts))
            pos = to_screen(ox, oy)
            pygame.draw.circle(screen, (255, 40, 40), pos, 15)
            pygame.draw.circle(screen, (255, 255, 255), pos, 15, 2)
            lbl = font.render("SOUPEŘ", True, (255, 150, 150))
            screen.blit(lbl, (pos[0]+18, pos[1]-10))
    opponents = live_opp

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
