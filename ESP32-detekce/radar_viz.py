import pygame, serial, sys, struct, math
from collections import deque

PORT     = '/dev/ttyUSB0'
BAUDRATE = 921600
FADE_MS  = 600
BG       = (15, 15, 15)

WIN  = 900
SPAN = 1500.0
SC   = WIN / SPAN
OFF  = 250

pygame.init()
screen = pygame.display.set_mode((WIN, WIN))
pygame.display.set_caption("LIDAR SLAM – Robot v aréně 1×1 m")
clock = pygame.time.Clock()
try:    font = pygame.font.SysFont("Courier", 16, bold=True)
except: font = pygame.font.SysFont(None, 20)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    print(f"[OK] {PORT}")
except Exception as e:
    print(f"[ERR] {e}"); sys.exit(1)

rob_x, rob_y, rob_h = 500.0, 500.0, 0.0
points, lines, opps = [], [], []

# --- Domovska pozice (pravy dolni roh, offsetovano o 300mm od kazde steny) ---
# Robot je 30x30cm, takze stred musi byt 150mm od steny (+ 150mm rezerva = 300mm)
HOME_X = 700.0  # 1000 - 300
HOME_Y = 300.0  # 0   + 300

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
    for d in range(-2, 13):
        v = d * 100
        c = (25,25,25) if d<0 or d>10 else (40,40,40)
        pygame.draw.line(surf, c, ts(v,-250), ts(v,1250), 1)
        pygame.draw.line(surf, c, ts(-250,v), ts(1250,v), 1)
    # Hranice areny 1x1m (bile tlusté čáry = "zdi")
    a = [ts(0,0), ts(1000,0), ts(1000,1000), ts(0,1000)]
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
    robot_corners_local = [(-150, 40), (150, 40), (150, -310), (-150, -310)]
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
    # Šipka vpřed (8 cm = 1/5 původní) od LiDARu
    al = 80.0
    pygame.draw.line(surf, (0,255,200), ts(rob_x, rob_y),
                     ts(rob_x + al*sr, rob_y + al*cr), 3)

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
                if -250<=x<=1250 and -250<=y<=1250:
                    points.append((x, y, now))
            elif mt == 1 and ml == 8:
                x1,y1,x2,y2 = struct.unpack('<hhhh', pay)
                col = (255,220,0) if len(lines)==0 else (180,0,255)
                lines.append((x1,y1,x2,y2,col,now))
            elif mt == 2 and ml == 6:
                rx,ry,hd = struct.unpack('<hhh', pay)
                if 0<=rx<=1000 and 0<=ry<=1000:
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
                if -250<=ox<=1250 and -250<=oy<=1250:
                    opps.append((ox,oy,now))
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
