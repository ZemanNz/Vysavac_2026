import pygame, serial, sys, struct, math
from collections import deque

PORT       = '/dev/ttyUSB0'
BAUDRATE   = 921600
FADE_MS    = 600
BG         = (15, 15, 15)

# === VELIKOST ARÉNY (mm) — ZMĚŇ TADY ===
ARENA_SIZE = 2500.0

MARGIN     = ARENA_SIZE * 0.25   # okraj kolem arény pro vizualizaci
WIN        = 900
SPAN       = ARENA_SIZE + 2 * MARGIN
SC         = WIN / SPAN
OFF        = MARGIN

pygame.init()
screen = pygame.display.set_mode((WIN, WIN))
pygame.display.set_caption(f"LIDAR SLAM – Robot v aréně {ARENA_SIZE/1000:.1f}×{ARENA_SIZE/1000:.1f} m")
clock = pygame.time.Clock()
try:    font = pygame.font.SysFont("Courier", 16, bold=True)
except: font = pygame.font.SysFont(None, 20)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"[OK] {PORT}")
except Exception as e:
    print(f"[ERR] {e}"); sys.exit(1)

rob_x, rob_y, rob_h = 500.0, 500.0, 0.0
rob_dist_front = 9999.0
points, lines, opps = [], [], []

# --- Domovska pozice (pravy dolni roh, offsetovano o 300mm od kazde steny) ---
# Robot je 30x30cm, takze stred musi byt 150mm od steny (+ 150mm rezerva = 300mm)
HOME_X = ARENA_SIZE - 300.0  # pravý okraj - 300
HOME_Y = 300.0               # spodní okraj + 300

# --- Trasa robota ---
# Kazdy prvek: (x, y) v mm souradnicich areny
PATH_MAX_LEN = 3000          # max pocet ulozenych bodu trasy
PATH_MIN_MOVE_MM = 5.0       # minimalni pohyb (mm) pro ulozeni bodu
PATH_MIN_TURN_DEG = 1.0      # minimalni otoceni (°) pro ulozeni bodu
path_history = deque(maxlen=PATH_MAX_LEN)
last_path_x, last_path_y, last_path_h = None, None, None

def ts(x, y):
    return (int((x + OFF) * SC), int(WIN - (y + OFF) * SC))

def draw(surf):
    surf.fill(BG)
    # Mrizka po 100mm
    grid_start = -int(MARGIN / 100)
    grid_end   = int((ARENA_SIZE + MARGIN) / 100)
    for d in range(grid_start, grid_end + 1):
        v = d * 100
        inside = (0 <= v <= ARENA_SIZE)
        c = (40,40,40) if inside else (25,25,25)
        pygame.draw.line(surf, c, ts(v, -MARGIN), ts(v, ARENA_SIZE + MARGIN), 1)
        pygame.draw.line(surf, c, ts(-MARGIN, v), ts(ARENA_SIZE + MARGIN, v), 1)
    # Hranice areny (bile tlusté čáry = "zdi")
    a = [ts(0,0), ts(ARENA_SIZE,0), ts(ARENA_SIZE,ARENA_SIZE), ts(0,ARENA_SIZE)]
    pygame.draw.lines(surf, (200,200,200), True, a, 3)

    # --- Trasa robota (oranžová čára, vykreslena PRED robotem) ---
    path_pts = list(path_history)
    if len(path_pts) >= 2:
        n = len(path_pts)
        for i in range(1, n):
            # Fade: starsi body jsou tmavsi (25% az 100% jasu)
            alpha = 0.25 + 0.75 * (i / n)
            col = (int(255 * alpha), int(140 * alpha), 0)
            pygame.draw.line(surf, col, ts(path_pts[i-1][0], path_pts[i-1][1]),
                             ts(path_pts[i][0],   path_pts[i][1]),   2)

    # Robot 35x30cm – LiDAR je referenční bod (4cm od přední hrany, uprostřed šířky)
    # Souřadnice rohů RELATIVNĚ k LiDARu v lokálním rámci (Y=vpřed, X=vpravo)
    #   Přední hrana: +40 mm od LiDARu
    #   Zadní hrana:  -310 mm od LiDARu  (350 - 40)
    #   Boční hrany:  ±150 mm
    r  = math.radians(rob_h)
    cr = math.cos(r); sr = math.sin(r)
    # (lx=lokální X=vpravo, ly=lokální Y=vpřed)
    robot_corners_local = [(-150, 40), (150, 40), (150, -320), (-150, -320)]
    corners = []
    for lx, ly in robot_corners_local:
        # Transformace: globální = lidar_pos + lx*pravý + ly*vpřed
        # Pravý vektor (CW): (cos h, -sin h)  =>  gx += lx*cos, gy -= lx*sin
        # Vpřed vektor:      (sin h,  cos h)  =>  gx += ly*sin, gy += ly*cos
        corners.append(ts(rob_x + lx*cr + ly*sr,
                          rob_y - lx*sr + ly*cr))
    pygame.draw.polygon(surf, (0,120,255), corners, 3)
    # LiDAR bod (tyrkysová tečka)
    pygame.draw.circle(surf, (0,200,255), ts(rob_x, rob_y), 5)
    # Šipka vpřed (8 cm) od LiDARu
    al = 80.0
    pygame.draw.line(surf, (0,255,200), ts(rob_x, rob_y),
                     ts(rob_x + al*sr, rob_y + al*cr), 3)

    # --- Kužel vzdálenosti vpředu (±15°) ---
    cone_len = 150.0  # délka čar kuželu v mm
    for ang in [-15, 15]:
        ar = math.radians(rob_h + ang)
        pygame.draw.line(surf, (255, 255, 0), ts(rob_x, rob_y), 
                         ts(rob_x + cone_len*math.sin(ar), rob_y + cone_len*math.cos(ar)), 1)
    
    if rob_dist_front < 9000:
        # Vykreslení hodnoty u nárazníku
        dist_txt = font.render(f"{int(rob_dist_front)}mm", True, (255,255,0))
        surf.blit(dist_txt, ts(rob_x + 50*sr + 20*cr, rob_y + 50*cr - 20*sr))

    # --- Domovska pozice: krizek + navigacni cara ---
    hp = ts(HOME_X, HOME_Y)
    # Krizek
    pygame.draw.line(surf, (0,200,80), (hp[0]-12, hp[1]), (hp[0]+12, hp[1]), 2)
    pygame.draw.line(surf, (0,200,80), (hp[0], hp[1]-12), (hp[0], hp[1]+12), 2)
    pygame.draw.circle(surf, (0,255,100), hp, 6, 2)
    # Zelena cara: robot -> home
    rp = ts(rob_x, rob_y)
    pygame.draw.line(surf, (0,255,80), rp, hp, 2)
    # Vypocet navigace (kompas: H=0 = sever/+Y, CW kladny)
    dx = HOME_X - rob_x
    dy = HOME_Y - rob_y
    dist_mm = math.sqrt(dx*dx + dy*dy)
    target_h = math.degrees(math.atan2(dx, dy))   # bearing север=0, CW+
    turn_deg = target_h - rob_h
    # Normalizace do -180 .. +180
    while turn_deg >  180: turn_deg -= 360
    while turn_deg < -180: turn_deg += 360
    turn_str  = f"{'doleva' if turn_deg < 0 else 'doprava'} {abs(turn_deg):.0f}°"
    nav_label = f"DOM: otoč {turn_str}  |  jet {dist_mm:.0f} mm"
    nav_surf  = font.render(nav_label, True, (80,255,130))
    surf.blit(nav_surf, (10, 30))
    # Info
    surf.blit(font.render(f"X={int(rob_x)} Y={int(rob_y)} H={int(rob_h)}°",
              True, (120,120,120)), (10,10))

buf = bytearray()
run = True
while run:
    now = pygame.time.get_ticks()
    for e in pygame.event.get():
        if e.type == pygame.QUIT: run = False
    try:
        if ser.in_waiting > 0: buf.extend(ser.read(ser.in_waiting))
    except: run = False; break

    while len(buf) >= 5:
        idx = buf.find(b'\xBB\x55')
        if idx == -1: buf = buf[-1:]; break
        if idx > 0: buf = buf[idx:]; continue
        if len(buf) < 4: break
        mt, ml = buf[2], buf[3]
        tot = 4 + ml + 1
        if len(buf) < tot: break
        pay = buf[4:4+ml]
        cs = buf[4+ml]
        if ((mt + ml + sum(pay)) & 0xFF) == cs:
            if mt == 0 and ml == 4:
                x, y = struct.unpack('<hh', pay)
                if -MARGIN<=x<=ARENA_SIZE+MARGIN and -MARGIN<=y<=ARENA_SIZE+MARGIN:
                    points.append((x, y, now))
            elif mt == 1 and ml == 8:
                x1,y1,x2,y2 = struct.unpack('<hhhh', pay)
                col = (255,220,0) if len(lines)==0 else (180,0,255)
                lines.append((x1,y1,x2,y2,col,now))
            elif mt == 2 and ml == 6:
                rx,ry,hd = struct.unpack('<hhh', pay)
                if 0<=rx<=ARENA_SIZE and 0<=ry<=ARENA_SIZE:
                    rob_x, rob_y, rob_h = float(rx), float(ry), float(hd)
                    # --- Zaznam do trasy ---
                    if last_path_x is None:
                        # Prvni bod vzdy ulozime
                        path_history.append((rob_x, rob_y))
                        last_path_x, last_path_y, last_path_h = rob_x, rob_y, rob_h
                    else:
                        dx = rob_x - last_path_x
                        dy = rob_y - last_path_y
                        dist = math.sqrt(dx*dx + dy*dy)
                        d_ang = abs(rob_h - last_path_h)
                        # Uhel: normalizace do <0, 180>
                        if d_ang > 180: d_ang = 360 - d_ang
                        if dist >= PATH_MIN_MOVE_MM or d_ang >= PATH_MIN_TURN_DEG:
                            path_history.append((rob_x, rob_y))
                            last_path_x, last_path_y, last_path_h = rob_x, rob_y, rob_h
                lines.clear(); opps.clear()
            elif mt == 3 and ml == 4:
                ox,oy = struct.unpack('<hh', pay)
                if -MARGIN<=ox<=ARENA_SIZE+MARGIN and -MARGIN<=oy<=ARENA_SIZE+MARGIN:
                    opps.append((ox,oy,now))
            elif mt == 4 and ml == 2:
                df, = struct.unpack('<h', pay)
                rob_dist_front = float(df)
        buf = buf[tot:]

    draw(screen)
    points = [(x,y,t) for x,y,t in points if now-t < FADE_MS]
    for x,y,t in points:
        i = max(0, 255 - int((now-t)/FADE_MS*255))
        if i>0: pygame.draw.circle(screen, (0,i,0), ts(x,y), 3)
    lines = [(a,b,c,d,col,t) for a,b,c,d,col,t in lines if now-t<200]
    for x1,y1,x2,y2,col,t in lines:
        pygame.draw.line(screen, col, ts(x1,y1), ts(x2,y2), 4)
    opps = [(x,y,t) for x,y,t in opps if now-t<300]
    for ox,oy,t in opps:
        p = ts(ox,oy)
        pygame.draw.circle(screen, (255,40,40), p, 15)
        pygame.draw.circle(screen, (255,255,255), p, 15, 2)
        screen.blit(font.render("SOUPEŘ", True, (255,150,150)), (p[0]+18,p[1]-10))
    pygame.display.flip()
    clock.tick(60)
pygame.quit()
