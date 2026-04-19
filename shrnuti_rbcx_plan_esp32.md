# Shrnutí: RBCX Slave — hotovo. Plán pro ESP32 Master.

Projekt: Autonomní soutěžní robot na sbírání puků.  
Repo: `/home/nik/Plocha/Programing/C++/Platformio/Vysavac_2026/`  
Dvě desky: **RBCX** (motory, serva, třídění) + **ESP32** (LiDAR, SLAM, strategie).

---

## 1. Co je hotové (RBCX strana)

### Soubory

| Soubor | Účel |
|---|---|
| `src/main_final.cpp` | Hlavní program — while(true) smyčka + UART vlákno |
| `src/asynchroni_pohyb.h` | Funkce `jed_a_sbirej(speed)` — nekonečná jízda dopředu se sběrem puků |
| `src/funkce.h` | Třídění puků (barvový senzor + step motor) |
| `src/stepper_motor.h` | Ovládání step motoru (GPIO 25,26,27,14) |
| `src/main_old.cpp.bak` | Starý main.cpp (zálohovaný, neaktivní) |

### Architektura RBCX

```
VLÁKNO 1 (hlavní, priorita 1):
  while(true) → čeká na novy_prikaz flag → switch(cmd) → vykoná pohyb
  + manuální tlačítka LEFT/RIGHT pro testování

VLÁKNO 2 (UART, priorita 2):
  rkUartReceive() → nastaví booly → hlavní smyčka vykoná
  Periodicky (200ms) posílá stav na ESP32
  Periodicky (1s) vypisuje stav na Serial Monitor
```

### Komunikační protokol

#### ESP32 → RBCX (3 bajty)
```c
typedef struct __attribute__((packed)) {
    uint8_t cmd;       // příkaz
    int16_t param;     // parametr
} EspCommand;
```

#### RBCX → ESP32 (6 bajtů, posílá se automaticky každých 200ms)
```c
typedef struct __attribute__((packed)) {
    uint8_t status;       // STAT_READY=0x80 / STAT_BUSY=0x81 / STAT_DONE=0x82
    uint8_t buttons;      // bit0=UP, bit1=DOWN, bit2=LEFT, bit3=RIGHT
    int16_t pocet_puku;   // naše puky v zásobníku
    int16_t param;        // extra
} RbcxStatus;
```

#### Příkazy (ESP32 → RBCX)
| CMD | Hex | Param | Popis |
|---|---|---|---|
| CMD_NOP | 0x00 | - | Jen pošli stav |
| CMD_STOP | 0x01 | - | Zastav vše |
| CMD_JED_SBIREJ | 0x02 | rychlost % | Jeď dopředu + sbírej puky |
| CMD_OTOC_VLEVO | 0x03 | úhel ° | Otoč se doleva |
| CMD_OTOC_VPRAVO | 0x04 | úhel ° | Otoč se doprava |
| CMD_COUVEJ | 0x05 | vzdálenost mm | Couvej |
| CMD_VYLOZ | 0x06 | - | Vysyp puky |

#### Klíčové chování
- RBCX posílá stav **každých 200ms automaticky** — ESP32 vždy ví: stav, tlačítka, počet puků
- Když ESP32 pošle **jakýkoliv příkaz** (kromě NOP), RBCX nejdřív **zastaví** co běží (`zastav_jizdu = true`), pak vykoná nový
- Po dokončení příkazu RBCX pošle **STAT_DONE** → ESP32 ví, že může poslat další
- `jed_a_sbirej()` je blokující — jede nekonečně, zastaví se na příkaz (STOP/nový příkaz) nebo náraz (tlačítka UP/DOWN)

### Funkce jed_a_sbirej(speed)
- Plynulá akcelerace (krok 0.5% za 10ms)
- Plynulá decelerace (krok 1.5% za 10ms)  
- P-regulátor pro rovný směr (Kp=0.12, max korekce 3%) — pouze v CRUISE fázi
- Třídění puků inline každých ~100ms (barvový senzor + step motor)
- Zastaví se: `zastav_jizdu == true` NEBO jedno přední tlačítko (UP/DOWN)
- Polarita motorů: M4=LEFT (normal), M1=RIGHT (inverted)

### UART funkce (z robotka.h)
```c
bool rkUartInit(int baudRate = 115200, int rxPin = 16, int txPin = 17);
bool rkUartReceive(void* msg, size_t msgSize);  // neblokující
void rkUartSend(const void* msg, size_t msgSize);
```

---

## 2. Co je hotové na ESP32 (z minulých chatů)

| Soubor | Účel |
|---|---|
| `ESP32-detekce/src/lidar.h` | LiDAR SLAM — pozice `g_rx, g_ry` (mm), heading `g_h` (°), detekce soupeře |
| `ESP32-detekce/src/main.cpp` | Základní stavový automat (rozpracovaný z minulého chatu) |

### Co lidar.h umí
- **Pozice robota:** `g_rx`, `g_ry` v mm (absolutní na hřišti 1000×1000mm)
- **Heading:** `g_h` ve stupních (0-360)
- **Detekce soupeře:** vzdálenost a úhel k nejbližší překážce
- **Offset:** 7° korekce na fyzické uložení LiDARu
- **Viditelnost:** předních ~200° z 360°

---

## 3. Plán pro ESP32 stranu (další chat)

### 3.1 Struktura ESP32 programu

```
setup():
  - Inicializace LiDAR
  - Inicializace UART (stejné piny/baudrate jako RBCX)
  - Čekání na startovní signál

loop():
  - Každých 20ms: přečti LiDAR data (pozice, heading, soupeř)
  - Každých 20ms: přečti stav z RBCX (rkUartReceive)
  - Stavový automat rozhodne co dál
  - Pošli příkaz na RBCX (rkUartSend)
```

### 3.2 Stavový automat ESP32

```
STATE_WAIT_START
  → Čekáme na startovní signál (tlačítko nebo časovač)
  → Přejde do STATE_ALIGN

STATE_ALIGN
  → Pošle CMD_JED_SBIREJ na RBCX (pomalé, třeba 20%)
  → Sleduje LiDAR heading → až je rovnoběžně se zdí (0°/90°/180°/270° ±1.5°)
  → Pošle CMD_STOP
  → Přejde do STATE_SEARCH

STATE_SEARCH (lajnová strategie)
  → Pošle CMD_JED_SBIREJ(60)
  → Sleduje SLAM pozici — až ujede cca 800mm (konec lajny):
    → CMD_STOP
    → CMD_OTOC_VPRAVO(90) → čeká na STAT_DONE
    → CMD_JED_SBIREJ(60) → ujede 150mm (šířka lajny)
    → CMD_STOP
    → CMD_OTOC_VPRAVO(90) → čeká na STAT_DONE
    → CMD_JED_SBIREJ(60) → nová lajna opačným směrem
  → Pokud LiDAR vidí soupeře < 30cm:
    → CMD_STOP → vyhýbací manévr
  → Pokud RBCX hlásí náraz (buttons UP/DOWN):
    → CMD_COUVEJ(100) → CMD_OTOC(90) → pokračuj
  → Pokud pocet_puku >= 5:
    → Přejde do STATE_RETURN

STATE_RETURN
  → Vypočítat vektor k domácí zóně z g_rx, g_ry
  → Otočit se směrem k domovu
  → CMD_JED_SBIREJ(80) — rychle domů
  → Až SLAM říká "jsme v domácí zóně":
    → CMD_STOP → STATE_DUMP

STATE_DUMP
  → CMD_VYLOZ → čeká na STAT_DONE
  → Přejde zpět do STATE_SEARCH

STATE_EMERGENCY
  → Pokud zbývá < 10s → okamžitě STATE_RETURN
  → Pokud soupeř blokuje → vyhýbací manévr
```

### 3.3 Jak ESP32 bude číst RBCX

```cpp
// Definovat STEJNÉ struktury jako na RBCX:
// EspCommand, RbcxStatus, CmdID, StatID

// Odeslání příkazu:
void posli_prikaz(uint8_t cmd, int16_t param = 0) {
    EspCommand c;
    c.cmd = cmd;
    c.param = param;
    rkUartSend(&c, sizeof(c));  // nebo Serial1.write()
}

// Čtení stavu (neblokující, volat v loop):
RbcxStatus rbcx_stav;
bool stav_ok = rkUartReceive(&rbcx_stav, sizeof(rbcx_stav));
if (stav_ok) {
    bool naraz_up   = (rbcx_stav.buttons >> 0) & 1;
    bool naraz_down = (rbcx_stav.buttons >> 1) & 1;
    int puky        = rbcx_stav.pocet_puku;
    bool hotovo     = (rbcx_stav.status == STAT_DONE);
}

// Čekání na dokončení příkazu:
void cekej_na_done() {
    while (true) {
        RbcxStatus st;
        if (rkUartReceive(&st, sizeof(st)) && st.status == STAT_DONE) break;
        delay(20);
    }
}
```

### 3.4 UART zapojení ESP32 ↔ RBCX

```
ESP32 GPIO16 (RX) ←→ RBCX GPIO17 (TX)
ESP32 GPIO17 (TX) ←→ RBCX GPIO16 (RX)
+ společný GND
Baudrate: 115200
```

### 3.5 Co je potřeba dořešit na ESP32
1. **Implementovat stavový automat** s lajnovou strategií
2. **Navigace domů** — výpočet úhlu k domácí zóně ze SLAM
3. **Časovač zápasu** — po 80s přejít do STATE_RETURN
4. **Detekce soupeře** — z LiDARu, vyhýbací manévry
5. **Detekce domácí barvy** — barvový senzor na podlaze? Nebo z pozice?

---

## 4. Důležité poznámky

- `nase_barva = 'R'` — zatím hardcoded červená, později z ESP32
- Step motor piny (25,26,27,14) kolidují s GPIO26/27 které se dříve plánovaly pro UART — **UART teď jede na 16/17**
- P-regulátor v `jed_a_sbirej` má Kp=0.12 (sníženo z 0.23 protože to bylo moc agresivní)
- Modrá LED nefunguje (known issue)
- Pro line following preferuje user metodu s `map()` místo P-regulátoru
