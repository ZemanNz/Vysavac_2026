import pygame
import serial
import math
import sys

# --- Konfigurace ---
PORT = '/dev/ttyUSB0'  # Upravte podle portu ESP32
BAUDRATE = 921600      # Nový vysokorychlostní baudrate
MAX_DIST_MM = 2000     # Zmenšené měřítko do 2 metrů
FADE_TIME_MS = 600     # Rychlejší blednutí pro dynamičtější vzhled radaru
BG_COLOR = (5, 15, 5)  # Temně zelená

# Inicializace okna radaru
WIDTH = 1000
HEIGHT = 500
CENTER = (WIDTH // 2, HEIGHT - 20)
SCALE = (WIDTH // 2 - 40) / MAX_DIST_MM

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LIDAR D200 - High-Speed 2m Radar")
clock = pygame.time.Clock()
try:
    font = pygame.font.SysFont("Courier", 18, bold=True)
except:
    font = pygame.font.SysFont(None, 24)

# Připojení na Serial
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"Připojeno na {PORT} rychlostí {BAUDRATE} baudů (Binární režim).")
except Exception as e:
    print(f"Chyba při otevírání portu {PORT}: {e}")
    sys.exit(1)

points = []  # Seznam bodů. Formát: (úhel, vzdálenost_mm, časový_krok_ms)

def draw_grid(surf):
    surf.fill(BG_COLOR)
    
    # Mřížka po 500 mm (0.5 m)
    for dm in range(1, (MAX_DIST_MM // 500) + 1):
        radius = int(dm * 500 * SCALE)
        pygame.draw.circle(surf, (0, 60, 0), CENTER, radius, 1)
        
        # Popisek vzdálenosti
        text_surface = font.render(f"{dm * 0.5:.1f} m", True, (0, 120, 0))
        surf.blit(text_surface, (CENTER[0] + 5, CENTER[1] - radius - 20))
        
    # Úhlové čáry po 30°
    for angle in range(0, 181, 30):
        rad = math.radians(angle)
        x = CENTER[0] + min(WIDTH//2, HEIGHT) * math.cos(rad)
        y = CENTER[1] - min(WIDTH//2, HEIGHT) * math.sin(rad)
        
        pygame.draw.line(surf, (0, 60, 0), CENTER, (int(x), int(y)), 1)
        
        text_surface = font.render(f"{angle}°", True, (0, 120, 0))
        surf.blit(text_surface, (int(x), int(y) - 15))
        
    # Červená tečka reprezentující samotný LiDAR uprostřed
    pygame.draw.circle(surf, (255, 0, 0), CENTER, 6)

buffer = bytearray()
running = True

print("Čekám na rychlá data...")

while running:
    current_time = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    # Načtení všech dat ze sériové linky s ošetřením odpojení
    try:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
    except OSError:
        print("\n[CHYBA] Zařízení bylo odpojeno! (Např. při nahrávání nového kódu do ESP).")
        print("Počkej na dokončení nahrávání a spusť vizualizaci znovu.")
        running = False
        break
        
    # Binární parser (hledá sekvenci 0xAA 0x55)
    while len(buffer) >= 6:
        idx = buffer.find(b'\xAA\x55')
        
        if idx == -1:
            # Hlavička nenalezena, zahodit neužitečná data, 
            # ale nechat poslední byte pro případ, že je to 'AA' před půlkou
            buffer = buffer[-1:] 
            break
        elif idx > 0:
            # Odstranit balast před hlavičkou
            buffer = buffer[idx:]
        else:
            # Hlavička je na začátku (idx == 0)
            if len(buffer) >= 6:
                angle_int = buffer[2] | (buffer[3] << 8)
                distance = buffer[4] | (buffer[5] << 8)
                
                # Zpracování kompletního paketu
                buffer = buffer[6:]
                
                angle = angle_int / 100.0
                if 0 <= angle <= 180:
                    points.append((angle, distance, current_time))
            else:
                # Není k dispozici celý paket, počkat na další cyklus
                break

    # Překreslení
    draw_grid(screen)
    
    # Filtrace zachovaných bodů a vykreslení s vizuálními efekty (Glow)
    new_points = []
    
    for pt in points:
        angle_deg, dist, timestamp = pt
        age = current_time - timestamp
        
        if age < FADE_TIME_MS:
            new_points.append(pt)
            
            # Nechceme zobrazovat daleké body (mimo 2 metry grafu)
            if dist > MAX_DIST_MM:
                continue
            
            if dist > 0:
                rad = math.radians(angle_deg)
                x = CENTER[0] + dist * SCALE * math.cos(rad)
                y = CENTER[1] - dist * SCALE * math.sin(rad)
                
                # Výpočet intenzity od 0 do 255
                intensity = max(0, 255 - int((age / FADE_TIME_MS) * 255))
                
                if intensity > 0:
                    # 1. Glow efekt (rozmazaný/velký kruh)
                    if intensity > 80:
                        glow_color = (0, int(intensity * 0.35), 0)
                        pygame.draw.circle(screen, glow_color, (int(x), int(y)), 5)
                    
                    # 2. Jasný střed (jádro)
                    core_color = (int(intensity * 0.4), intensity, int(intensity * 0.4))
                    pygame.draw.circle(screen, core_color, (int(x), int(y)), 2)
                
    points = new_points
    
    pygame.display.flip()
    clock.tick(60) # Plynulý update při 60 FPS

pygame.quit()
ser.close()
