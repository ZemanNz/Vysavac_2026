import pygame, serial, sys, struct, math

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
    # Robot 30x30cm
    half = 150.0
    r = math.radians(rob_h)
    cr = math.cos(r); sr = math.sin(r)
    corners = []
    for lx, ly in [(-half,-half),(half,-half),(half,half),(-half,half)]:
        corners.append(ts(rob_x + lx*cr + ly*sr, rob_y - lx*sr + ly*cr))
    pygame.draw.polygon(surf, (0,120,255), corners, 3)
    pygame.draw.circle(surf, (0,200,255), ts(rob_x, rob_y), 5)
    # Sipka (35cm dopredu = lokalni +Y)
    al = 350.0
    pygame.draw.line(surf, (0,255,200), ts(rob_x,rob_y),
                     ts(rob_x + al*sr, rob_y + al*cr), 3)
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
