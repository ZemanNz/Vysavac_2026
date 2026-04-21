#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
╔═══════════════════════════════════════════════════════════╗
║  SIMULÁTOR MOZKU — 2D vizuální tester stavového automatu  ║
║  Zrcadlí logiku z ESP32-detekce/src/mozek.h               ║
║                                                            ║
║  Heading konvence (z lidar_no_viz.h):                      ║
║    0° = +Y (sever/nahoru)                                  ║
║   90° = +X (východ/vpravo)                                 ║
║  -90° = -X (západ/vlevo)                                   ║
║  180° = -Y (jih/dolů)                                      ║
║                                                            ║
║  Souřadnice:                                               ║
║    X →  (0 = levá zeď, 1500 = pravá zeď)                  ║
║    Y ↑  (0 = spodní zeď/naše, 1500 = horní zeď/soupeř)   ║
║    HOME = pravý dolní roh (300×300mm čtverec)              ║
║                                                            ║
║  Ovládání:                                                 ║
║    SPACE — Start zápasu                                    ║
║    B     — Simuluj náraz vpředu (bump)                    ║
║    P     — Přidej puk (+1)                                ║
║    S     — Toggle soupeře před robota                     ║
║    R     — Reset simulace                                 ║
║    ESC   — Ukončit                                        ║
╚═══════════════════════════════════════════════════════════╝

ROZDÍLY OPROTI mozek.h (navržené opravy):
  1. Pořadí lajn: mozek.h má shora dolů (lajna 0 = Y=1350), ale přechodový
     manévr vždy otáčí VLEVO, což vede nahoru. Oprava: lajny zdola nahoru
     (lajna 0 = Y=150).
  2. Směr otáčení při přechodu: mozek.h vždy otáčí VLEVO. Oprava: otáčíme
     podle aktuálního směru jízdy (doprava → VLEVO, doleva → VPRAVO).
  3. Startovní pozice: mozek.h má TODO. Simulátor startuje robota na levém
     okraji první lajny (Y=150) otočeného doprava.
"""

import pygame
import math
import time
import sys

# =============================================================================
#  KONSTANTY (odpovídají mozek.h a lidar_no_viz.h)
# =============================================================================

ARENA_SIZE_MM = 1500.0
SIRKA_ROBOTA_MM = 300.0
DELKA_ROBOTA_MM = 350.0
BEZPECNA_VZDALENOST_ZDI = 200.0
HOME_X = ARENA_SIZE_MM - 500.0   # 1200 mm — vnitřní roh domovské zóny
HOME_Y = 500.0                    # 300 mm

# Mřížka pokrytí
BUNKA_MM = SIRKA_ROBOTA_MM
POCET_BUNEK_X = int(ARENA_SIZE_MM / BUNKA_MM)  # 5
POCET_BUNEK_Y = int(ARENA_SIZE_MM / BUNKA_MM)  # 5

# Soupeř
VZDALENOST_SOUPERE_STOP = 400.0
UHEL_SOUPERE_VPRED = 45.0

# Puky
PUKY_PLNY_ZASOBNIK = 5

# Čas
DELKA_ZAPASU_S = 90.0
CAS_NOUZOVEHO_NAVRATU_S = 10.0

# Simulační rychlosti
RYCHLOST_ZAKLAD = 200.0     # mm/s … odpovídá 60% výkonu motorů
RYCHLOST_COUVANI = 150.0    # mm/s
RYCHLOST_OTACENI = 90.0     # °/s

# =============================================================================
#  PYGAME — Rozložení okna
# =============================================================================

OKNO_SIRKA = 1280
OKNO_VYSKA = 700

ARENA_OKRAJ = 40
ARENA_PX = int(min(OKNO_SIRKA // 2 - 2 * ARENA_OKRAJ,
                    OKNO_VYSKA - 2 * ARENA_OKRAJ - 60))
ARENA_X0 = ARENA_OKRAJ
ARENA_Y0 = (OKNO_VYSKA - ARENA_PX) // 2

GRID_X0 = OKNO_SIRKA // 2 + 60
GRID_Y0 = 90
GRID_BUNKA_PX = 80

MERITKO = ARENA_PX / ARENA_SIZE_MM  # mm → px

# =============================================================================
#  BARVY
# =============================================================================

class B:
    BG           = (18, 18, 24)
    ARENA_BG     = (28, 33, 42)
    ARENA_BORDER = (55, 65, 85)
    GRID_LINE    = (38, 45, 58)

    ROBOT_BODY   = (0, 200, 120)
    ROBOT_DIR    = (200, 255, 200)
    STOPA        = (0, 90, 55)

    SOUPER       = (220, 50, 50)
    HOME_FILL    = (255, 200, 50, 30)
    HOME_BORDER  = (255, 200, 50)

    BUNKA_EMPTY  = (32, 38, 50)
    BUNKA_COVER  = (0, 140, 85)
    BUNKA_ROBOT  = (0, 210, 125)

    TXT          = (200, 210, 230)
    TXT_DIM      = (100, 110, 130)
    TXT_ACC      = (80, 200, 255)
    TXT_WARN     = (255, 160, 50)
    TXT_ERR      = (255, 70, 70)
    TXT_OK       = (80, 220, 130)

    STAV = {
        'CEKAM':   (120, 120, 140),
        'JEDU':    (0, 200, 120),
        'PRECH':   (255, 200, 50),
        'VYHYB':   (255, 100, 50),
        'DOMU':    (80, 150, 255),
        'VYKLAM':  (200, 100, 255),
        'NOUZOV':  (255, 50, 50),
    }

STAV_BARVA_MAP = {}  # vyplní se níže

# =============================================================================
#  STAVY ROBOTA  (zrcadlí enum StavRobota z mozek.h)
# =============================================================================

CEKAM    = 'CEKAM_NA_START'
NAJEZD   = 'NAJEZD_NAHORU'
JEDU     = 'JEDU_LAJNU'
PRECHOD  = 'PRECHOD_LAJNY'
VYHYBAM  = 'VYHYBAM_SE'
DOMU     = 'VRACIM_DOMU'
VYKLADAM = 'VYKLADAM'
NOUZOVY  = 'NOUZOVY_NAVRAT'

STAV_BARVA_MAP = {
    CEKAM:    B.STAV['CEKAM'],
    NAJEZD:   (100, 200, 255),
    JEDU:     B.STAV['JEDU'],
    PRECHOD:  B.STAV['PRECH'],
    VYHYBAM:  B.STAV['VYHYB'],
    DOMU:     B.STAV['DOMU'],
    VYKLADAM: B.STAV['VYKLAM'],
    NOUZOVY:  B.STAV['NOUZOV'],
}

# =============================================================================
#  HEADING KONVENCE  (atan2(dx, dy) — navigační)
#
#  mozek.h: senzory.heading = nv_g_h * 180 / PI
#  lidar:   angle = atan2f(dx, dy)
#
#  0°   = +Y (sever, nahoru)
#  90°  = +X (východ, vpravo)
#  -90° = -X (západ, vlevo)
#  180° = -Y (jih, dolů)
#
#  Pohyb:  x += sin(h) * speed,  y += cos(h) * speed
#  Otoc vlevo (CCW z pohledu shora):  heading KLESÁ
#  Otoc vpravo (CW):                  heading ROSTE
# =============================================================================

def heading_to_rad(deg):
    return math.radians(deg)

def normalize_heading(deg):
    """Normalizuj do (-180, 180]."""
    while deg > 180: deg -= 360
    while deg <= -180: deg += 360
    return deg


# =============================================================================
#  SIM RBCX  (mock Slave)
# =============================================================================

class SimRbcx:
    def __init__(self):
        self.hotovo = True
        self._timer = 0.0
        self._popis = ""
        self.pocet_puku = 0
        self.bump_vpredu = False

    def prikaz(self, cmd, param=0):
        self.hotovo = True  # default
        self._popis = cmd

        if cmd == 'STOP':
            pass  # ihned hotovo
        elif cmd == 'JED':
            pass  # průběžný, ihned READY
        elif cmd == 'OTOC_L':
            self._timer = abs(param) / RYCHLOST_OTACENI
            self.hotovo = False
            self._popis = f"OTOC_L {param}°"
        elif cmd == 'OTOC_R':
            self._timer = abs(param) / RYCHLOST_OTACENI
            self.hotovo = False
            self._popis = f"OTOC_R {param}°"
        elif cmd == 'COUVEJ':
            self._timer = abs(param) / RYCHLOST_COUVANI
            self.hotovo = False
            self._popis = f"COUVEJ {param}mm"
        elif cmd == 'VYLOZ':
            self._timer = 1.0  # čas na otevření
            self.hotovo = False
            self._popis = "VYLOŽ"
        elif cmd == 'ZAVRI':
            self._timer = 1.0  # čas na zavření
            self.hotovo = False
            self._popis = "ZAVŘI"

    def update(self, dt):
        if self.hotovo:
            return
        self._timer -= dt
        if self._timer <= 0:
            self._timer = 0
            self.hotovo = True


# =============================================================================
#  LAJNOVÁ NAVIGACE
# =============================================================================

class Navigace:
    def __init__(self):
        self.cislo_lajny = 0
        self.smer_doprava = False  # začínáme DOLEVA (od HOME pryč)
        self.cil_x = 0.0
        self.pocet_lajn = 0
        self.lajna_y = []
        self.celkem_lajn = 0
        self.dokoncena_kola = 0

    def inicializuj(self):
        """Spočítej Y středy lajn.
        Lajny shora dolů: L0=Y1350 (nahoře), L4=Y150 (dole u HOME).
        Robot nejdřív vyjede nahoru, pak zig-zaguje dolů.
        """
        self.pocet_lajn = int(ARENA_SIZE_MM / SIRKA_ROBOTA_MM)
        self.lajna_y = []
        for i in range(self.pocet_lajn):
            # Shora dolů: lajna 0 nahoře
            y = ARENA_SIZE_MM - (i + 0.5) * SIRKA_ROBOTA_MM
            self.lajna_y.append(y)

        self.cislo_lajny = 0
        self.smer_doprava = False  # první lajna jede DOLEVA
        self.celkem_lajn = 0
        self.dokoncena_kola = 0

    def dalsi_lajna(self):
        self.cislo_lajny += 1
        self.smer_doprava = not self.smer_doprava
        self.celkem_lajn += 1
        # Ne-wrapujeme! Volající musí zkontrolovat cislo_lajny >= pocet_lajn


# =============================================================================
#  ROBOT  (stavový automat + kinematika)
# =============================================================================

class Robot:
    def __init__(self):
        self.reset()

    def reset(self):
        # Pozice a heading (navigační konvence)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # 0° = +Y (nahoru)

        self.stav = CEKAM
        self.krok = 0
        self.cas_startu = None
        self.cas_zapas_s = 0.0

        self.rbcx = SimRbcx()
        self.nav = Navigace()
        self.nav.inicializuj()

        self.mapa = [[False]*POCET_BUNEK_Y for _ in range(POCET_BUNEK_X)]

        # Soupeř
        self.sup_on = False
        self.sup_x = 0.0
        self.sup_y = 0.0

        # Kinematika
        self._jedu = False
        self._rychlost = 0.0
        self._otacim = False
        self._otoc_smer = 0   # +1 = vpravo (CW), -1 = vlevo (CCW)
        self._otoc_zbyva = 0.0
        self._couvam = False
        self._couv_zbyva = 0.0

        self.stopa = []
        self.log = []
        self._t_krok3 = None

    # ─── Heading & pozice ────────────────────────────────

    def _posun(self, vzdalenost):
        """Posuň ve směru heading."""
        r = heading_to_rad(self.heading)
        self.x += math.sin(r) * vzdalenost
        self.y += math.cos(r) * vzdalenost

    def _posun_zpet(self, vzdalenost):
        """Couvni (opačný směr)."""
        r = heading_to_rad(self.heading)
        self.x -= math.sin(r) * vzdalenost
        self.y -= math.cos(r) * vzdalenost

    # ─── Senzorová data (zrcadlí mozek.h) ────────────────

    def _domov_vzd(self):
        dx = HOME_X - self.x
        dy = HOME_Y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def _domov_uhel_rel(self):
        """Relativní úhel k domovu (se znaménkem).
        Odpovídá mozek.h: angle_home = atan2f(dx, dy)
                          rel = (angle_home - nv_g_h)
        """
        dx = HOME_X - self.x
        dy = HOME_Y - self.y
        angle_home = math.degrees(math.atan2(dx, dy))
        rel = angle_home - self.heading
        return normalize_heading(rel)

    def _domov_uhel(self):
        return abs(self._domov_uhel_rel())

    def _domov_smer(self):
        """'R' nebo 'L' — zrcadlí mozek.h: (rel >= 0) ? 'R' : 'L'"""
        rel = self._domov_uhel_rel()
        return 'R' if rel >= 0 else 'L'

    def _sup_v_ceste(self):
        if not self.sup_on:
            return False
        dx = self.sup_x - self.x
        dy = self.sup_y - self.y
        vzd = math.sqrt(dx*dx + dy*dy)
        angle_sup = math.degrees(math.atan2(dx, dy))
        rel = normalize_heading(angle_sup - self.heading)
        return vzd < VZDALENOST_SOUPERE_STOP and abs(rel) < UHEL_SOUPERE_VPRED

    def _sup_vzd(self):
        if not self.sup_on:
            return 9999
        dx = self.sup_x - self.x
        dy = self.sup_y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def _naraz(self):
        return self.rbcx.bump_vpredu

    # ─── Mapa pokrytí ────────────────────────────────────

    def _bx(self):
        return max(0, min(int(self.x / BUNKA_MM), POCET_BUNEK_X - 1))

    def _by(self):
        return max(0, min(int(self.y / BUNKA_MM), POCET_BUNEK_Y - 1))

    def _oznac_pokryti(self):
        self.mapa[self._bx()][self._by()] = True

    def _pokryto(self):
        return sum(1 for x in range(POCET_BUNEK_X)
                   for y in range(POCET_BUNEK_Y) if self.mapa[x][y])

    # ─── Lajnová navigace ────────────────────────────────

    def _nastav_cil(self):
        """Nastav cílové X pro aktuální lajnu. (zrcadlí nastav_cil_lajny)"""
        if self.nav.smer_doprava:
            self.nav.cil_x = ARENA_SIZE_MM - BEZPECNA_VZDALENOST_ZDI  # 1000
        else:
            self.nav.cil_x = BEZPECNA_VZDALENOST_ZDI  # 500

    def _na_konci_lajny(self):
        if self.nav.smer_doprava:
            return self.x >= self.nav.cil_x
        else:
            return self.x <= self.nav.cil_x

    # ─── Příkazy RBCX ────────────────────────────────────

    def _cmd_stop(self):
        self.rbcx.prikaz('STOP')
        self._jedu = False
        self._otacim = False
        self._couvam = False
        self._rychlost = 0

    def _cmd_jed(self, procent):
        self.rbcx.prikaz('JED', procent)
        self._jedu = True
        self._otacim = False
        self._couvam = False
        self._rychlost = RYCHLOST_ZAKLAD * (procent / 60.0)

    def _cmd_otoc_vlevo(self, stupne):
        """Otočení VLEVO = CCW z pohledu shora = heading KLESÁ v nav konvenci."""
        self.rbcx.prikaz('OTOC_L', stupne)
        self._jedu = False
        self._otacim = True
        self._couvam = False
        self._otoc_smer = -1   # heading klesá
        self._otoc_zbyva = abs(stupne)

    def _cmd_otoc_vpravo(self, stupne):
        """Otočení VPRAVO = CW z pohledu shora = heading ROSTE v nav konvenci."""
        self.rbcx.prikaz('OTOC_R', stupne)
        self._jedu = False
        self._otacim = True
        self._couvam = False
        self._otoc_smer = +1   # heading roste
        self._otoc_zbyva = abs(stupne)

    def _cmd_couvej(self, mm):
        self.rbcx.prikaz('COUVEJ', mm)
        self._jedu = False
        self._otacim = False
        self._couvam = True
        self._couv_zbyva = abs(mm)

    def _cmd_vyloz(self):
        self.rbcx.prikaz('VYLOZ')
        self._jedu = False

    def _cmd_zavri(self):
        self.rbcx.prikaz('ZAVRI')
        self._jedu = False

    # ─── Změna stavu ──────────────────────────────────────

    def _zmen(self, novy):
        self._log_msg(f"STAV: {self.stav} → {novy}")
        self.stav = novy
        self.krok = 0
        self._t_krok3 = None

    def _log_msg(self, txt):
        self.log.append((time.time(), txt))
        if len(self.log) > 60:
            self.log = self.log[-60:]

    # ─── Kinematika ───────────────────────────────────────

    def _update_kine(self, dt):
        if self._otacim and self._otoc_zbyva > 0:
            rot = min(RYCHLOST_OTACENI * dt, self._otoc_zbyva)
            self._otoc_zbyva -= rot
            self.heading += self._otoc_smer * rot
            self.heading = normalize_heading(self.heading)
            if self._otoc_zbyva <= 0:
                self._otacim = False

        elif self._couvam and self._couv_zbyva > 0:
            d = min(RYCHLOST_COUVANI * dt, self._couv_zbyva)
            self._couv_zbyva -= d
            self._posun_zpet(d)
            if self._couv_zbyva <= 0:
                self._couvam = False

        elif self._jedu:
            self._posun(self._rychlost * dt)

        # Ohranič v aréně
        self.x = max(0, min(self.x, ARENA_SIZE_MM))
        self.y = max(0, min(self.y, ARENA_SIZE_MM))

        # Stopa
        self.stopa.append((self.x, self.y))
        if len(self.stopa) > 3000:
            self.stopa = self.stopa[-3000:]

    # ─── HLAVNÍ UPDATE ────────────────────────────────────

    def update(self, dt):
        self.rbcx.update(dt)
        self._update_kine(dt)

        if self.stav != CEKAM:
            self._oznac_pokryti()

        # Čas
        if self.cas_startu is not None:
            self.cas_zapas_s = time.time() - self.cas_startu
            zbyva = DELKA_ZAPASU_S - self.cas_zapas_s

            if (zbyva < CAS_NOUZOVEHO_NAVRATU_S
                and self.stav != NOUZOVY
                and self.stav != VYKLADAM):
                self._log_msg("!!! ČAS KONČÍ → NOUZOVÝ NÁVRAT !!!")
                self._cmd_stop()
                self._zmen(NOUZOVY)

        self._rozhoduj()

    # ─── STAVOVÝ AUTOMAT  (zrcadlí mozek_rozhoduj) ────────

    def _rozhoduj(self):

        # ── ČEKÁM NA START ─────────────────────────────────
        if self.stav == CEKAM:
            pass

        # ── NÁJEZD NAHORU ──────────────────────────────────
        #  Robot jede z HOME nahoru po pravé straně.
        #  Když Y dosáhne první lajny → otoč doleva → JEDU_LAJNU
        elif self.stav == NAJEZD:
            k = self.krok
            if k == 0:
                # Jedeme nahoru, čekáme na dosažení lajny 0
                if self.y >= self.nav.lajna_y[0] - SIRKA_ROBOTA_MM / 2:
                    self._cmd_stop()
                    self._cmd_otoc_vlevo(90)  # 0° → -90° (doleva)
                    self.krok = 1
            elif k == 1:
                if self.rbcx.hotovo:
                    self._nastav_cil()
                    self._cmd_jed(60)
                    self._log_msg(f"Nahoře! Lajna 0 → DOLEVA")
                    self._zmen(JEDU)

        # ── JEDU LAJNU ─────────────────────────────────────
        elif self.stav == JEDU:
            # [A] Plný zásobník
            if self.rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK:
                self._log_msg(f"Plný zásobník ({self.rbcx.pocet_puku}) → DOMŮ")
                self._cmd_stop()
                self._zmen(DOMU)
                return

            # [B] Soupeř v cestě
            if self._sup_v_ceste():
                self._log_msg(f"Soupeř v cestě! ({self._sup_vzd():.0f}mm) → VYHÝBÁM")
                self._cmd_stop()
                self._zmen(VYHYBAM)
                return

            # [C] Náraz vpředu → couvni + přechod
            if self._naraz():
                self._cmd_stop()
                self.rbcx.bump_vpredu = False
                if self.y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0)):
                    self._log_msg("Náraz vpředu → PŘECHOD (s couvním)")
                    self._zmen(PRECHOD)
                else:
                    self._log_msg("Náraz zcela dole → VYKLÁDÁM")
                    self._zmen(VYKLADAM)
                return

            # [D] Konec lajny (bezpečná vzdálenost)
            if self._na_konci_lajny():
                self._cmd_stop()
                if self.y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0)):
                    self._log_msg(f"Konec lajny na Y={self.y:.0f} → PŘECHOD")
                    self._zmen(PRECHOD)
                    self.krok = 1  # přeskoč couvání
                else:
                    self._log_msg(f"Lajna na dně Y={self.y:.0f} hotová → VYKLÁDÁM")
                    self._zmen(VYKLADAM)
                return

        # ── PŘECHOD NA DALŠÍ LAJNU ─────────────────────────
        #
        #  Vícekrokový manévr:
        #    0: Couvni 100mm
        #    1: Otoč 90° (směr závisí na smer_doprava)
        #    2: Popojeď o šířku robota (~2s)
        #    3: Čekej na ujetí
        #    4: Otoč znovu 90° (stejný směr)
        #    5: Nastav novou lajnu → JEDU_LAJNU
        #
        #  ╔═══════════════════════════════════════════════╗
        #  ║  OPRAVA vs mozek.h:                           ║
        #  ║  mozek.h vždy otáčí VLEVO. To funguje jen     ║
        #  ║  při jízdě doprava. Při jízdě doleva musíme   ║
        #  ║  otáčet VPRAVO aby robot šel nahoru.           ║
        #  ║                                                ║
        #  ║  PRAVIDLO:                                     ║
        #  ║  smer_doprava → OTOC_VLEVO (heading klesá)    ║
        #  ║  !smer_doprava → OTOC_VPRAVO (heading roste)  ║
        #  ╚═══════════════════════════════════════════════╝
        #
        elif self.stav == PRECHOD:
            k = self.krok

            if k == 0:  # Couvni
                self._cmd_couvej(100)
                self.krok = 1

            elif k == 1:  # Čekej na couvání → otoč DOLŮ
                if self.rbcx.hotovo:
                    # Otáčíme aby robot šel DOLŮ:
                    #   doleva  → VLEVO  (h klesá → dolů)
                    #   doprava → VPRAVO (h roste → dolů)
                    if self.nav.smer_doprava:
                        self._cmd_otoc_vpravo(90)
                    else:
                        self._cmd_otoc_vlevo(90)
                    self.krok = 2

            elif k == 2:  # Čekej na otočení → jeď o šířku
                if self.rbcx.hotovo:
                    self._cmd_jed(40)
                    self._t_krok3 = time.time()
                    self.krok = 3

            elif k == 3:  # Popojíždíme o šířku robota (~2s)
                if self._t_krok3 and time.time() - self._t_krok3 > 2.0:
                    self._cmd_stop()
                    self._t_krok3 = None
                    self.krok = 4

            elif k == 4:  # Druhé otočení (stejný směr)
                if self.rbcx.hotovo:
                    if self.nav.smer_doprava:
                        self._cmd_otoc_vpravo(90)
                    else:
                        self._cmd_otoc_vlevo(90)
                    self.krok = 5

            elif k == 5:  # Hotovo → nová lajna nebo domů
                if self.rbcx.hotovo:
                    self.nav.dalsi_lajna()
                    self._nastav_cil()
                    self._cmd_jed(60)
                    smer = "→" if self.nav.smer_doprava else "←"
                    self._log_msg(f"Lajna (smer: {smer})")
                    self._zmen(JEDU)

        # ── VYHÝBÁM SE SOUPEŘI ─────────────────────────────
        elif self.stav == VYHYBAM:
            k = self.krok
            if k == 0:
                # 1. Natočení po směru dolů
                if self.nav.smer_doprava:
                    self._cmd_otoc_vpravo(90)
                else:
                    self._cmd_otoc_vlevo(90)
                self.krok = 1

            elif k == 1:
                # 2. Čekání na dotočení a jízda dolů o šířku robota
                if self.rbcx.hotovo:
                    self._cmd_jed(40)
                    self._t_krok3 = time.time()
                    self.krok = 2

            elif k == 2:
                # 3. Zastavení po zhruba 300 mm jízdě (odhadem 2 sekundy při 40 %)
                if self._t_krok3 and time.time() - self._t_krok3 > 1.5:
                    self._cmd_stop()
                    self.krok = 3

            elif k == 3:
                # 4. Vrácení se zpět do původního kurzu
                if self.rbcx.hotovo:
                    if self.nav.smer_doprava:
                        self._cmd_otoc_vlevo(90)
                    else:
                        self._cmd_otoc_vpravo(90)
                    self.krok = 4

            elif k == 4:
                # 5. Odjezd zpět jako JEDU a pokračování v kurzu
                if self.rbcx.hotovo:
                    self._cmd_jed(60)
                    self._zmen(JEDU)

        # ── VRACÍM SE DOMŮ ─────────────────────────────────
        elif self.stav == DOMU:
            k = self.krok
            if k == 0:
                if self._domov_uhel() > 10.0:
                    uhel = int(self._domov_uhel())
                    if self._domov_smer() == 'L':
                        self._cmd_otoc_vlevo(uhel)
                    else:
                        self._cmd_otoc_vpravo(uhel)
                    self.krok = 1
                else:
                    self.krok = 2
            elif k == 1:
                if self.rbcx.hotovo:
                    self.krok = 2
            elif k == 2:
                self._cmd_jed(70)
                self.krok = 3
            elif k == 3:
                if self._domov_vzd() < 150.0:
                    self._log_msg("Jsme doma!")
                    self._cmd_stop()
                    self._zmen(VYKLADAM)
                elif self._domov_uhel() > 30.0:
                    self._cmd_stop()
                    self.krok = 0  # znovu zamiř

        # ── VYKLÁDÁM PUKY ──────────────────────────────────
        elif self.stav == VYKLADAM:
            k = self.krok

            # === Krok 0: Urči cestu ===
            if k == 0:
                if not self.nav.smer_doprava:
                    # Cesta A: Šli jsme DOLEVA → otoč se 180° k HOME
                    self._log_msg("Vyklad: cesta A (z levé strany)")
                    self._cmd_otoc_vpravo(180)
                    self.krok = 10
                else:
                    # Cesta B: Šli jsme DOPRAVA → jsme v HOME, rovnou do společné fáze
                    self._log_msg("Vyklad: cesta B (z pravé strany)")
                    self.krok = 20

            # === Cesta A: Z levé strany → otočit + dojet do HOME ===
            elif k == 10:  # Čekej na 180° otočku
                if self.rbcx.hotovo:
                    self._cmd_jed(60)
                    self.krok = 11

            elif k == 11:  # Jedeme do HOME zóny
                if self.x >= ARENA_SIZE_MM - BEZPECNA_VZDALENOST_ZDI:  # k bezpečné vzdálenosti jako při jízdě lajny
                    self._cmd_stop()
                    self.krok = 20  # Společně se správně natoč

            # === Společná fáze: Natočení nahoru a dump manévr ===
            elif k == 20:
                # Jsme v HOME a koukáme doprava (+X)
                self._cmd_otoc_vlevo(90)  # tvář nahoru
                self.krok = 21

            elif k == 21:
                if self.rbcx.hotovo:
                    self.krok = 30

            elif k == 30:
                self._log_msg("Otevírám zásobníky...")
                self._cmd_vyloz()
                self.krok = 31

            elif k == 31:
                if self.rbcx.hotovo:
                    self._log_msg("Zásobníky otevřeny. Popojíždím 30 cm...")
                    self._cmd_jed(40)  # pomalá jízda vpřed
                    self._t_krok3 = time.time()
                    self.krok = 40

            elif k == 40:
                if self._t_krok3 and time.time() - self._t_krok3 > 1.5:  # ~30cm
                    self._cmd_stop()
                    self.krok = 50

            elif k == 50:
                self._log_msg("Zavírám zásobníky...")
                self._cmd_zavri()
                self.krok = 51

            elif k == 51:
                if self.rbcx.hotovo:
                    self.rbcx.pocet_puku = 0
                    pk = self._pokryto()
                    cel = POCET_BUNEK_X * POCET_BUNEK_Y
                    self._log_msg(f"Puky vyloženy! Pokryto {pk}/{cel}")
                    self.krok = 60

            elif k == 60:
                self._log_msg("═══ PŘIPRAVENA NA DALŠÍ KOLO ═══")
                self.krok = 61

            elif k == 61:
                pass  # Čekáme



        # ── NOUZOVÝ NÁVRAT ─────────────────────────────────
        elif self.stav == NOUZOVY:
            k = self.krok
            if k == 0:
                if self._domov_uhel() > 10.0:
                    uhel = int(self._domov_uhel())
                    if self._domov_smer() == 'L':
                        self._cmd_otoc_vlevo(uhel)
                    else:
                        self._cmd_otoc_vpravo(uhel)
                    self.krok = 1
                else:
                    self.krok = 2
            elif k == 1:
                if self.rbcx.hotovo:
                    self.krok = 2
            elif k == 2:
                self._cmd_jed(90)
                self.krok = 3
            elif k == 3:
                if self._domov_vzd() < 150.0:
                    self._cmd_stop()
                    self._zmen(VYKLADAM)

    # ─── START ZÁPASU ─────────────────────────────────────

    def start_zapasu(self):
        if self.stav != CEKAM:
            return
        self.cas_startu = time.time()
        self.nav.inicializuj()

        # Robot startuje v HOME (pravý dolní roh), míří NAHORU
        # heading = 0° = +Y (nahoru) v navigační konvenci
        self.x = ARENA_SIZE_MM - SIRKA_ROBOTA_MM / 2   # 1350 mm
        self.y = SIRKA_ROBOTA_MM / 2                     # 150 mm
        self.heading = 0.0                                # míří nahoru (+Y)

        self._cmd_jed(60)
        self._zmen(NAJEZD)
        self._log_msg("═══ ZÁPAS ZAHÁJEN — NÁJEZD NAHORU ═══")


# =============================================================================
#  RENDERER  (vykreslování)
# =============================================================================

class Renderer:
    def __init__(self, scr):
        self.scr = scr
        pygame.font.init()
        try:
            self.f = pygame.font.SysFont("DejaVu Sans", 14)
            self.fb = pygame.font.SysFont("DejaVu Sans", 18, bold=True)
            self.ft = pygame.font.SysFont("DejaVu Sans", 22, bold=True)
            self.fs = pygame.font.SysFont("DejaVu Sans", 11)
        except Exception:
            self.f = pygame.font.Font(None, 16)
            self.fb = pygame.font.Font(None, 22)
            self.ft = pygame.font.Font(None, 26)
            self.fs = pygame.font.Font(None, 13)

    def mm2px(self, mx, my):
        """Aréna mm → screen px. Y flip (mm Y↑, screen Y↓)."""
        px = ARENA_X0 + mx * MERITKO
        py = ARENA_Y0 + ARENA_PX - my * MERITKO
        return int(px), int(py)

    def draw(self, rob):
        self.scr.fill(B.BG)
        self._arena(rob)
        self._grid(rob)
        self._hud(rob)
        self._controls()

    # ─── Aréna ─────────────────────────────────────────

    def _arena(self, rob):
        ar = pygame.Rect(ARENA_X0, ARENA_Y0, ARENA_PX, ARENA_PX)
        pygame.draw.rect(self.scr, B.ARENA_BG, ar)
        pygame.draw.rect(self.scr, B.ARENA_BORDER, ar, 2)

        # Mřížka
        for i in range(1, POCET_BUNEK_X):
            x = ARENA_X0 + int(i * BUNKA_MM * MERITKO)
            pygame.draw.line(self.scr, B.GRID_LINE, (x, ARENA_Y0), (x, ARENA_Y0+ARENA_PX))
        for i in range(1, POCET_BUNEK_Y):
            y = ARENA_Y0 + int(i * BUNKA_MM * MERITKO)
            pygame.draw.line(self.scr, B.GRID_LINE, (ARENA_X0, y), (ARENA_X0+ARENA_PX, y))

        # HOME zóna (pravý dolní roh)
        hx1, hy1 = self.mm2px(HOME_X, 0)
        hx2, hy2 = self.mm2px(ARENA_SIZE_MM, HOME_Y)
        hr = pygame.Rect(min(hx1,hx2), min(hy1,hy2),
                         abs(hx2-hx1), abs(hy2-hy1))
        hs = pygame.Surface((hr.w, hr.h), pygame.SRCALPHA)
        hs.fill((255, 200, 50, 30))
        self.scr.blit(hs, hr.topleft)
        pygame.draw.rect(self.scr, B.HOME_BORDER, hr, 2)
        t = self.fs.render("HOME", True, B.HOME_BORDER)
        self.scr.blit(t, (hr.centerx - t.get_width()//2, hr.bottom + 2))

        # Navigační bod HOME (křížek)
        hpx, hpy = self.mm2px(HOME_X, HOME_Y)
        pygame.draw.line(self.scr, B.HOME_BORDER, (hpx-5, hpy), (hpx+5, hpy), 2)
        pygame.draw.line(self.scr, B.HOME_BORDER, (hpx, hpy-5), (hpx, hpy+5), 2)

        # Lajny (horizontální čáry Y)
        for i, ly in enumerate(rob.nav.lajna_y):
            _, py = self.mm2px(0, ly)
            c = (100, 200, 140) if i == rob.nav.cislo_lajny else (50, 65, 80)
            pygame.draw.line(self.scr, c, (ARENA_X0+2, py), (ARENA_X0+ARENA_PX-2, py), 1)
            t = self.fs.render(f"L{i}", True, c)
            self.scr.blit(t, (ARENA_X0+3, py-13))

        # Cíl X (vertikální čára)
        if rob.stav == JEDU:
            cx, _ = self.mm2px(rob.nav.cil_x, 0)
            pygame.draw.line(self.scr, (255,200,50,100),
                             (cx, ARENA_Y0+2), (cx, ARENA_Y0+ARENA_PX-2), 1)

        # Stopa robota
        if len(rob.stopa) > 1:
            pts = [self.mm2px(sx, sy) for sx, sy in rob.stopa]
            for i in range(1, len(pts)):
                a = min(255, 50 + 200*i//len(pts))
                pygame.draw.line(self.scr, (0, a//3, a//4), pts[i-1], pts[i], 2)

        # Soupeř
        if rob.sup_on:
            sx, sy = self.mm2px(rob.sup_x, rob.sup_y)
            r = int(SIRKA_ROBOTA_MM / 2 * MERITKO)
            pygame.draw.circle(self.scr, B.SOUPER, (sx, sy), r)
            pygame.draw.circle(self.scr, (255,100,100), (sx, sy), r, 2)
            t = self.fs.render("SOUPEŘ", True, B.SOUPER)
            self.scr.blit(t, (sx - t.get_width()//2, sy - r - 14))

        # Robot
        self._robot(rob)

        # Nadpis
        t = self.ft.render("ARÉNA (1500×1500 mm)", True, B.TXT)
        self.scr.blit(t, (ARENA_X0 + ARENA_PX//2 - t.get_width()//2, ARENA_Y0 - 28))

        # Osy
        t = self.fs.render("X →", True, B.TXT_DIM)
        self.scr.blit(t, (ARENA_X0 + ARENA_PX - 30, ARENA_Y0 + ARENA_PX + 4))
        t = self.fs.render("Y ↑", True, B.TXT_DIM)
        self.scr.blit(t, (ARENA_X0 - 24, ARENA_Y0 - 2))

    def _robot(self, rob):
        """Vykreslí robota jako otočený obdélník.
        V navigační konvenci: heading 0° = +Y, 90° = +X.
        pygame rotate: kladné stupně = CCW.
        Robot surface: šipka míří NAHORU (=heading 0°).
        rotate(heading) by rotoval CCW, ale my chceme CW pro kladný heading.
        → rotujeme o -heading.
        """
        rx, ry = self.mm2px(rob.x, rob.y)
        w = int(SIRKA_ROBOTA_MM * MERITKO)
        h = int(DELKA_ROBOTA_MM * 0.5 * MERITKO)

        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        barva = STAV_BARVA_MAP.get(rob.stav, B.ROBOT_BODY)
        pygame.draw.rect(surf, (*barva, 160), (0, 0, w, h), border_radius=4)
        pygame.draw.rect(surf, barva, (0, 0, w, h), 2, border_radius=4)

        # Šipka nahoru (vpřed = nahoře na surface)
        sipka = [
            (w//2, 4),
            (w//2 - 8, 16),
            (w//2 + 8, 16),
        ]
        pygame.draw.polygon(surf, B.ROBOT_DIR, sipka)

        # Otočení: heading 0° = surface tak jak je (šipka nahoru na screen)
        # heading 90° = robot míří vpravo → rotace -90° (CCW v pygame = kladné)
        # → rotujeme o -heading
        rotated = pygame.transform.rotate(surf, -rob.heading)
        rect = rotated.get_rect(center=(rx, ry))
        self.scr.blit(rotated, rect)

        # Střed
        pygame.draw.circle(self.scr, (255, 255, 255), (rx, ry), 3)

    # ─── Mřížka pokrytí ────────────────────────────────

    def _grid(self, rob):
        t = self.ft.render("MAPA POKRYTÍ", True, B.TXT)
        self.scr.blit(t, (GRID_X0, GRID_Y0 - 30))

        rbx, rby = rob._bx(), rob._by()

        for gx in range(POCET_BUNEK_X):
            for gy in range(POCET_BUNEK_Y):
                # Y flip: gy=0 (dolní) → kreslíme dole
                draw_gy = POCET_BUNEK_Y - 1 - gy
                px = GRID_X0 + gx * GRID_BUNKA_PX
                py = GRID_Y0 + draw_gy * GRID_BUNKA_PX
                r = pygame.Rect(px, py, GRID_BUNKA_PX, GRID_BUNKA_PX)

                if gx == rbx and gy == rby:
                    c = B.BUNKA_ROBOT
                elif rob.mapa[gx][gy]:
                    c = B.BUNKA_COVER
                else:
                    c = B.BUNKA_EMPTY

                pygame.draw.rect(self.scr, c, r, border_radius=6)
                pygame.draw.rect(self.scr, B.ARENA_BORDER, r, 1, border_radius=6)

                lbl = self.fs.render(f"{gx},{gy}", True, (160,170,190))
                self.scr.blit(lbl, (px+4, py+4))

                if rob.mapa[gx][gy]:
                    chk = self.fb.render("✓", True, (255,255,255))
                    self.scr.blit(chk, (px + GRID_BUNKA_PX//2 - 6,
                                         py + GRID_BUNKA_PX//2 - 6))

        pk = rob._pokryto()
        cel = POCET_BUNEK_X * POCET_BUNEK_Y
        pct = pk / cel * 100 if cel else 0
        t = self.f.render(f"Pokryto: {pk}/{cel} ({pct:.0f}%)", True, B.TXT_OK)
        self.scr.blit(t, (GRID_X0, GRID_Y0 + POCET_BUNEK_Y * GRID_BUNKA_PX + 8))

    # ─── HUD ───────────────────────────────────────────

    def _hud(self, rob):
        x0 = GRID_X0
        y = GRID_Y0 + POCET_BUNEK_Y * GRID_BUNKA_PX + 35

        # Stav
        sc = STAV_BARVA_MAP.get(rob.stav, B.TXT)
        t = self.fb.render(f"▶ {rob.stav}", True, sc)
        self.scr.blit(t, (x0, y)); y += 24

        # Krok
        t = self.f.render(f"  Krok: {rob.krok}", True, B.TXT_DIM)
        self.scr.blit(t, (x0, y)); y += 18

        # RBCX
        rc = B.TXT_OK if rob.rbcx.hotovo else B.TXT_WARN
        rs = "HOTOVO" if rob.rbcx.hotovo else f"BUSY ({rob.rbcx._popis})"
        t = self.f.render(f"  RBCX: {rs}", True, rc)
        self.scr.blit(t, (x0, y)); y += 18

        # Pozice
        t = self.f.render(f"  Pozice: ({rob.x:.0f}, {rob.y:.0f}) mm", True, B.TXT)
        self.scr.blit(t, (x0, y)); y += 18

        # Heading
        t = self.f.render(f"  Heading: {rob.heading:.1f}°  "
                          f"({'↑' if abs(rob.heading) < 45 else '→' if 45 <= rob.heading < 135 else '↓' if abs(rob.heading) > 135 else '←'})",
                          True, B.TXT)
        self.scr.blit(t, (x0, y)); y += 18

        # Puky — číslo + kolečka
        pc = B.TXT_ERR if rob.rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK else B.TXT
        t = self.f.render(f"  Puky: {rob.rbcx.pocet_puku}/{PUKY_PLNY_ZASOBNIK}", True, pc)
        self.scr.blit(t, (x0, y))
        for i in range(PUKY_PLNY_ZASOBNIK):
            cx = x0 + 130 + i * 18
            cy = y + 8
            if i < rob.rbcx.pocet_puku:
                pygame.draw.circle(self.scr, B.TXT_ACC, (cx, cy), 6)
            else:
                pygame.draw.circle(self.scr, B.TXT_DIM, (cx, cy), 6, 1)
        y += 20

        # Domov
        t = self.f.render(
            f"  Domov: {rob._domov_vzd():.0f}mm  "
            f"({rob._domov_uhel():.0f}° {rob._domov_smer()})",
            True, B.TXT_DIM)
        self.scr.blit(t, (x0, y)); y += 18

        # Lajna
        sm = "→" if rob.nav.smer_doprava else "←"
        t = self.f.render(
            f"  Lajna: {rob.nav.cislo_lajny}/{rob.nav.pocet_lajn} {sm}  "
            f"cíl_X={rob.nav.cil_x:.0f}  kolo {rob.nav.dokoncena_kola}",
            True, B.TXT_DIM)
        self.scr.blit(t, (x0, y)); y += 18

        # Čas
        if rob.cas_startu is not None:
            zb = max(0, DELKA_ZAPASU_S - rob.cas_zapas_s)
            cc = B.TXT_ERR if zb < CAS_NOUZOVEHO_NAVRATU_S else B.TXT
            t = self.f.render(f"  Čas: {zb:.1f}s zbývá", True, cc)
        else:
            t = self.f.render("  Čas: —", True, B.TXT_DIM)
        self.scr.blit(t, (x0, y)); y += 22

        # Log (posledních 4)
        t = self.fs.render("LOG:", True, B.TXT_DIM)
        self.scr.blit(t, (x0, y)); y += 14
        for _, msg in rob.log[-4:]:
            t = self.fs.render(f"  {msg}", True, B.TXT_DIM)
            self.scr.blit(t, (x0, y)); y += 13

    # ─── Ovládání ──────────────────────────────────────

    def _controls(self):
        t = self.ft.render("SIMULÁTOR MOZKU", True, B.TXT_ACC)
        self.scr.blit(t, (ARENA_X0, 8))

        keys = [
            ("SPACE", "Start"),
            ("B", "Bump"),
            ("P", "+Puk"),
            ("S", "Soupeř"),
            ("R", "Reset"),
        ]
        cx = OKNO_SIRKA // 2 + 60
        cy = 8
        for kk, pp in keys:
            bw = max(42, self.fs.size(kk)[0] + 12)
            br = pygame.Rect(cx, cy, bw, 18)
            pygame.draw.rect(self.scr, (48, 52, 65), br, border_radius=4)
            pygame.draw.rect(self.scr, (75, 85, 105), br, 1, border_radius=4)
            tk = self.fs.render(kk, True, B.TXT_ACC)
            self.scr.blit(tk, (cx + bw//2 - tk.get_width()//2, cy+2))
            tp = self.fs.render(pp, True, B.TXT_DIM)
            self.scr.blit(tp, (cx + bw + 4, cy+2))
            cx += bw + tp.get_width() + 14


# =============================================================================
#  MAIN LOOP
# =============================================================================

def main():
    pygame.init()
    scr = pygame.display.set_mode((OKNO_SIRKA, OKNO_VYSKA))
    pygame.display.set_caption("Simulátor Mozku — stavový automat robota")
    clock = pygame.time.Clock()
    renderer = Renderer(scr)
    robot = Robot()

    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    robot.start_zapasu()

                elif ev.key == pygame.K_b:
                    robot.rbcx.bump_vpredu = True
                    robot._log_msg("KLÁVESA: Bump vpředu!")

                elif ev.key == pygame.K_p:
                    robot.rbcx.pocet_puku += 1
                    robot._log_msg(f"KLÁVESA: +1 puk ({robot.rbcx.pocet_puku})")

                elif ev.key == pygame.K_s:
                    if robot.sup_on:
                        robot.sup_on = False
                        robot._log_msg("KLÁVESA: Soupeř odstraněn")
                    else:
                        r = heading_to_rad(robot.heading)
                        robot.sup_x = max(0, min(robot.x + math.sin(r)*350, ARENA_SIZE_MM))
                        robot.sup_y = max(0, min(robot.y + math.cos(r)*350, ARENA_SIZE_MM))
                        robot.sup_on = True
                        robot._log_msg(f"KLÁVESA: Soupeř ({robot.sup_x:.0f}, {robot.sup_y:.0f})")

                elif ev.key == pygame.K_r:
                    robot = Robot()
                    robot._log_msg("RESET")

                elif ev.key == pygame.K_ESCAPE:
                    running = False

        robot.update(dt)
        renderer.draw(robot)
        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
