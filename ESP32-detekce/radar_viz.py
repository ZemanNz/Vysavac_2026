import pygame
import serial
import sys
import struct

# --- Konfigurace ---
PORT = '/dev/ttyUSB0'  # Upravte podle portu ESP32
BAUDRATE = 921600
FADE_TIME_MS = 600     # Rychlost zmizení Lidar bodů
BG_COLOR = (15, 15, 15)

# Absolutní rozšíření "světa" přesahuje 1m arénu. Vykreslujeme pásmo široké přes 1500 mm.
# Samotná soutěžní hrací plocha leží na mapě exaktně v souřadnicích `0x0` až `1000x1000`.
# To jí ale nechává volných štědrých 25 centimetrů prostoru na okrajích, pro efektní "výběhy" oněch fialových matematických stěn.
WINDOW_SIZE = 900
PHYSICAL_SPAN = 1500.0  # Výřez reprezentující rozpětí fyzického mapovaného světa (-250 až 1250 mm)
SCALE = WINDOW_SIZE / PHYSICAL_SPAN
OFFSET_MM = 250  # Nulová hrana hřiště leží 250 milimetrů vpravo od lemu zobrazené plochy okna

pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("LIDAR D200 - Vektorové vykreslování shluků a stěn s výběhy")
clock = pygame.time.Clock()

try:
    font = pygame.font.SysFont("Courier", 16, bold=True)
except:
    font = pygame.font.SysFont(None, 20)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"[OK] Připojeno na port {PORT}.")
except Exception as e:
    print(f"[CHYBA] Nelze otevřít port {PORT}: {e}")
    sys.exit(1)

# Dynamické proměnné odometrie z ESP
robot_x_mm = 500
robot_y_mm = 0
points = []  
lines = []   
opponents = []

def to_screen(x_mm, y_mm):
    # Převod MM na pixely obrazovky posunuté o extra 25cm zrcadlovku pro ten tmavý lem arény.
    sx = int((x_mm + OFFSET_MM) * SCALE)
    sy = int(WINDOW_SIZE - ((y_mm + OFFSET_MM) * SCALE))
    return (sx, sy)

def draw_grid(surf):
    surf.fill(BG_COLOR)
    
    # Slabší orientační mřížka veškerého volného prostoru i mimo "ostrou" arénu (od -20cm až po 120cm hranice)
    for dm in range(-2, 13):
        val = dm * 100
        # Pouze slabunké linky pro pochopení prostoru
        color = (25, 25, 25) if dm < 0 or dm > 10 else (35, 35, 35) 
        
        pygame.draw.line(surf, color, to_screen(val, -250), to_screen(val, 1250), 1)
        pygame.draw.line(surf, color, to_screen(-250, val), to_screen(1250, val), 1)

    # ----------------------------------------------------
    # OHRANIČENÍ STRIKTNÍ TZV "HERNÍ ARÉNY" 1x1 METR HRUBOU LINIOU
    # ----------------------------------------------------
    arena_points = [
        to_screen(0, 0), to_screen(1000, 0),
        to_screen(1000, 1000), to_screen(0, 1000)
    ]
    pygame.draw.lines(surf, (150, 150, 150), True, arena_points, 3)
    
    # Indikační cedulka
    text = font.render(f"START ZONE", True, (80, 80, 80))
    surf.blit(text, to_screen(450, 50)) 

    # --- Kreslení samotného "šoupajícího se" robota ---
    robot_scr = to_screen(robot_x_mm, robot_y_mm)
    pygame.draw.circle(surf, (200, 0, 0), robot_scr, 8)
    pygame.draw.circle(surf, (255, 255, 0), robot_scr, 2)
    # Čumák (směřující k horní hraně arény / Y = 1000)
    pygame.draw.line(surf, (0, 255, 255), robot_scr, to_screen(robot_x_mm, robot_y_mm + 40), 2)


buffer = bytearray()
running = True

print("Aplikace Linear Regresse rozšířeného prostoru spuštěna.")
print("Čekám na datový stream z ESP32 (Protokol 0xBB)...")

while running:
    current_time = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    try:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
    except OSError:
        print("\n[CHYBA] Zařízení bylo odpojeno!")
        running = False
        break
        
    while len(buffer) >= 6:
        idx = buffer.find(b'\xBB\x55')
        
        if idx == -1:
            buffer = buffer[-1:] 
            break
        elif idx > 0:
            buffer = buffer[idx:]
        else:
            if len(buffer) >= 4:
                msg_type = buffer[2]
                msg_len = buffer[3]
                
                if len(buffer) >= 4 + msg_len + 1:
                    payload = buffer[4:4+msg_len]
                    checksum = buffer[4+msg_len]
                    
                    calc_cs = (msg_type + msg_len + sum(payload)) & 0xFF
                    
                    if calc_cs == checksum:
                        if msg_type == 0 and msg_len == 4:
                            x, y = struct.unpack('<hh', payload)
                            # Zobrazujeme už naprosto všechna odražená data z LIdaru ležící i mírně mimo hrubou arénu 
                            # limitovaná max rožpětím view-portu !
                            if -250 <= x <= 1250 and -250 <= y <= 1250:
                                points.append((x, y, current_time))
                                
                        elif msg_type == 1 and msg_len == 8:
                            x1, y1, x2, y2 = struct.unpack('<hhhh', payload)
                            # Type 1 Stěna (Fialově prokládaná Metodou Nejmenšího Čtverce / Linear Regression) z ESP
                            # Vykresluje se na obří vzdálenosti bez omezování
                            lines.append((x1, y1, x2, y2, (200, 0, 255), current_time))
                            
                        elif msg_type == 2 and msg_len == 4:
                            # Type 2 Aktuální kalkulovaná Pozice/Odometrie robota dle shluků
                            rx, ry = struct.unpack('<hh', payload)
                            if 0 <= rx <= 1000 and 0 <= ry <= 1000:
                                robot_x_mm = rx
                                robot_y_mm = ry
                                
                        elif msg_type == 3 and msg_len == 4:
                            # Type 3 Detekovaný soupeř (těžiště nepřátelského zbytku shluku)
                            ox, oy = struct.unpack('<hh', payload)
                            if -250 <= ox <= 1250 and -250 <= oy <= 1250:
                                opponents.append((ox, oy, current_time))
                    
                    buffer = buffer[4+msg_len+1:]
                else:
                    break
            else:
                break

    draw_grid(screen)
    
    new_points = []
    for pt in points:
        x, y, timestamp = pt
        age = current_time - timestamp
        
        if age < FADE_TIME_MS:
            new_points.append(pt)
            intensity = max(0, 255 - int((age / FADE_TIME_MS) * 255))
            if intensity > 0:
                COLOR = (0, intensity, 0)
                pos = to_screen(x, y)
                pygame.draw.circle(screen, COLOR, pos, 3)
    points = new_points

    new_lines = []
    for l in lines:
        x1, y1, x2, y2, col, timestamp = l
        age = current_time - timestamp
        # Čáry držíme jen kratince (150ms), navzájem se nesmí vrstvit (už jich nebude 10)
        if age < 150:
            new_lines.append(l)
            pos1 = to_screen(x1, y1)
            pos2 = to_screen(x2, y2)
            # Matematické zdi
            pygame.draw.line(screen, col, pos1, pos2, 4)
    lines = new_lines
    
    new_opponents = []
    for opp in opponents:
        ox, oy, timestamp = opp
        age = current_time - timestamp
        # Držíme vizualizaci těžiště protivníka déle (např 250ms), aby neproblikávala
        if age < 250:
            new_opponents.append(opp)
            pos = to_screen(ox, oy)
            # Velký červený detekční terč + nápis
            pygame.draw.circle(screen, (255, 30, 30), pos, 15)
            pygame.draw.circle(screen, (255, 255, 255), pos, 15, 2)
            try:
                text = font.render("SOUPEŘ", True, (255, 150, 150))
                screen.blit(text, (pos[0]+20, pos[1]-10))
            except:
                pass
    opponents = new_opponents
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
