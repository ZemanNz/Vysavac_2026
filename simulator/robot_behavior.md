# 🤖 Logika chování robota — simulator.py

> Kompletní, podrobný popis stavového automatu a všech rozhodnutí robota.  
> Aréna: **2500 × 2500 mm**. Souřadnice: X → (levá=0, pravá=2500), Y ↑ (dolní=0=naše zeď, horní=2500=soupeřova zeď).  
> HOME zóna = pravý dolní roh, **700 × 700 mm** čtverec (X: 1800–2500, Y: 0–700).

---

## 0. Fyzikální konstanty a limity

| Konstanta | Hodnota | Popis |
|---|---|---|
| Šířka robota | 300 mm | Reálné rozměry |
| Délka robota | 360 mm | Reálné rozměry |
| Bezpečný okraj (X) | 250 mm | Střed nesmí blíž ke zdi na X ose |
| Bezpečný okraj (Y) | 280 mm | Střed nesmí blíž ke zdi na Y ose |
| Základní rychlost | 200 mm/s (60 % výkonu) | Dopředu |
| Rychlost couvání | 150 mm/s | |
| Rychlost otáčení | 90 °/s | |
| Soupeř — STOP | 500 mm | Robot zastaví, když je soupeř do 50 cm čelně (±45°) |
| Soupeř — VOLNO | 650 mm | Robot se rozjede, když je soupeř nad 65 cm nebo mimo úhel |
| Plný zásobník | 10 puků | Triggr pro návrat domů |
| Délka zápasu | 180 s | |
| Nouzový návrat | 10 s do konce | |

---

## 1. Heading (orientace) — konvence

```
  0°   = míří NAHORU (+Y, sever)
 90°   = míří VPRAVO (+X, východ)
-90°   = míří VLEVO (-X, západ)
180°   = míří DOLŮ (-Y, jih)
```

Pohyb: `x += sin(heading) * speed`, `y += cos(heading) * speed`

---

## 2. Přehled stavů

```
CEKAM_NA_START
      │ [SPACE]
      ▼
NAJEZD_NAHORU  ──(soupeř/náraz)──► otočí + ► JEDU_LAJNU
      │ (dosáhl Y lajny 0)
      ▼
JEDU_LAJNU ────────────────────────────────────┐
  │ [plný zásobník]                             │
  ▼                                             │
VRACIM_DOMU                                     │
  │                                        VYHYBAM_SE
  ▼                                             │
VYKLADAM ──► CEKAM (čeká na 2. SPACE)          │
                                                │
CEKAM (2. SPACE) ──► PRESUN_Y ──► PRESUN_X ──► JEDU_LAJNU
                                                │
                            PRECHOD_LAJNY ◄─────┘
                                  │
                                  └──► JEDU_LAJNU (nová lajna)

Kdykoli: NOUZOVY_NAVRAT (≤10 s do konce zápasu)
```

---

## 3. Inicializace a start

### Před startem — `CEKAM_NA_START`
- Robot stojí, nedělá nic.
- Čeká na stisk **SPACE**.

### První SPACE — `start_zapasu()`
1. Robot se **umístí na startovní pozici**: X = 2250 mm (10 cm od pravé zdi), Y = 350 mm (střed HOME), heading = 0° (nahoru).
2. Spustí se timer zápasu.
3. Inicializuje se navigační mřížka lajn:
   - **8 lajn** (2500 / 300 = ~8), shora dolů: L0 = Y 2375, L1 = Y 2125 ... L7 = Y 125.
4. Robot dostane příkaz **JEĎ vpřed (60 %)**.
5. Přechod → stav **`NAJEZD_NAHORU`**.

---

## 4. Stav `NAJEZD_NAHORU`

> Robot jede z HOME nahoru po pravé straně arény směrem k Y=2375 (první lajna).

### Krok 0 — kontroly při jízdě nahoru

**Priorita A — plný zásobník:**
- Pokud `počet_puků >= 10` → `STOP` → přechod **`VRACIM_DOMU`**.

**Priorita B — soupeř v cestě:**
- Pokud je soupeř vzdálen < 500 mm a v čelním sektoru ±45°:
  - `STOP`
  - otočit **vlevo 90°** (heading 0° → -90°, míří doleva)
  - `krok = 1` (přeskočí čekání na Y lajny, rovnou bude jejet lajnu)

**Priorita C — náraz vpředu (senzor bumper):**
- Podobně jako B: `STOP`, vlevo 90°, `krok = 1`.

**Normální průběh:**
- Robot jede nahoru, dokud `Y >= lajna_y[0] - 150 mm`.
- Po dosažení → `STOP`, otočit **vlevo 90°** (heading 0° → -90°, míří doleva), `krok = 1`.

### Krok 1 — čekání na dokončení otočení
- Čeká, dokud RBCX hlásí `hotovo`.
- Pak: nastav cíl X pro první lajnu, JEĎ 60 %, přechod → **`JEDU_LAJNU`**.

---

## 5. Stav `JEDU_LAJNU`

> Hlavní pracovní stav — robot křižuje arénu horizontálně po aktuální lajně a sbírá puky.

Každý frame (60 fps) se kontroluje v tomto pořadí:

### Priorita A — plný zásobník (≥10 puků)
- `STOP` → **`VRACIM_DOMU`**.

### Priorita B — soupeř v cestě
- Je-li soupeř < 500 mm a ±45° vpředu:
  - `STOP` → **`VYHYBAM_SE`**.

### Priorita C — náraz vpředu (bumper)
- `STOP`, vymaž příznak bumpu.
- Pokud `Y > 580 mm` (není u spodní stěny):
  - → **`PRECHOD_LAJNY`** (bude couvat + přejít na nižší lajnu)
- Pokud je robot u spodní stěny (`Y ≤ 580 mm`):
  - → **`VYKLADAM`** (jsme dole u domova, vylož puky)

### Priorita D — konec lajny
- Robot se dostal na cílové X (`cil_x`):
  - `STOP`.
  - **Dynamický režim?** (druhá jízda po SPACE):
    - Vypočti další nevyčištěný úsek na mapě.
    - Pokud existuje → **`PRESUN_Y`**.
    - Pokud ne → **`VRACIM_DOMU`**.
  - **Normální režim:**
    - Pokud `Y > 580 mm` → **`PRECHOD_LAJNY`** (krok 1, přeskok couvání).
    - Pokud je robot u dna → **`VYKLADAM`**.

> **Poznámka:** Pokud žádná z podmínek nesedí, robot prostě pokračuje v jízdě a sbírá puky (inkrementace mapy pokrytí).

---

## 6. Stav `PRECHOD_LAJNY`

> Manévr pro přejezd z jedné lajny na sousední (o 300 mm dolů).  
> Pořadí kroků: couvni → otoč dolů → jeď dolů → otoč do nového směru → jeď novou lajnou.

### Krok 0 — kontrola před couvnutím
- **Podívej se LiDARem dolů (180°, do 500 mm, ±45°)** — je tam soupeř?
  - Ano → přeotoč se 180° na starou lajnu (krok 10, zpět do původního směru).
  - Ne → **couvni 100 mm**, `krok = 1`.

### Krok 1 — čekání na couvání + otočení dolů
- Po dokončení couvání:
  - Jedeme-li **doprava** → otoč **vpravo 90°** (heading 90° → 180°, míří dolů).
  - Jedeme-li **doleva** → otoč **vlevo 90°** (heading -90° → -180°/180°, míří dolů).
  - `krok = 2`.

### Krok 2 — čekání na otočení + rozjezd dolů
- Po dotočení → JEĎ **40 %** (pomalejší přejezd), zapamatuj čas → `krok = 3`.

### Krok 3 — jedeme dolů
Každý frame:
- **Soupeř v cestě?** → `STOP`, otoč 180° (zpět), smer_doprava invertovat → krok 10 (únik zpět).
- Uplynulo-li >2 s? → zkontroluj, zda je volno v cílovém směru (lajny):
  - Cílový heading: doprava = -90°, doleva = 90°.
  - Pokud volno NEBO jsme u spodní stěny NEBO uplynulo >8 s (pojistka):
    - `STOP` → `krok = 4`.

### Krok 4 — druhé otočení do směru nové lajny
- Jedeme-li **doprava** → otoč vpravo 90° (180° → 90°, míří vpravo).
- Jedeme-li **doleva** → otoč vlevo 90° (180° → -90°/270°, míří doleva).
- `krok = 5`.

### Krok 5 — nastavení nové lajny a odjezd
- `dalsi_lajna()`: číslo lajny +1, invertuj smer_doprava, +1 k počtu dokončených lajn.
- Nastav nový cíl X, JEĎ 60 % → **`JEDU_LAJNU`**.

### Krok 10 — únik zpět (soupeř blokoval přechod)
- Po dotočení 180° zpět → nastav cíl, JEĎ 60 % → **`JEDU_LAJNU`** (pokračuje starou lajnou opačným směrem).

---

## 7. Stav `VYHYBAM_SE`

> Soupeř zablokoval cestu — robot se musí dostat z jeho dosahu bočním pohybem dolů.

### Krok 0 — rozhodnutí
- **U spodní stěny** (`Y ≤ 580 mm`)?
  - Ano: nemůžeme jít dolů → čekáme na místě.
  - Pokud se soupeř vzdálí (> 650 mm nebo mimo úhel) → **obnovit jízdu** → vrátit se do předchozího stavu.
  - Ne → pokračuj níže.
- **Podívej se LiDARem dolů (180°, do 500 mm, ±45°)**:
  - Soupeř pod námi? → otoč 180° (opačný směr) → krok 10.
- Otoč se dolů (stejná logika jako PRECHOD krok 1), krok = 1.

### Krok 1 — čekání na otočení + odjezd dolů
- JEĎ 40 % → zapamatuj čas → `krok = 2`.

### Krok 2 — jedeme dolů (boční úhyb)
- **Soupeř se připletl?** → `STOP`, otoč 180° (zpět) → krok 10.
- Uplynulo-li > 1.5 s?
  - Podívej se v cílovém směru (lajny), je volno do 600 mm?
    - Volno → `STOP` → krok 3.
  - Pojistka: > 6 s → `STOP` → krok 3.

### Krok 3 — vrácení do původního kurzu
- Otoč se zpět do původního směru lajny:
  - Doprava → vlevo 90°.
  - Doleva → vpravo 90°.
  - `krok = 4`.

### Krok 4 — odjezd v původním kurzu
- Po dotočení → JEĎ 60 % → vrátit se do **předchozího stavu** (JEDU nebo NAJEZD, podle toho kde přerušení nastalo).

### Krok 10 — alternativní únik (soupeř blokoval dolů)
- Po dotočení 180° → nastav cíl → JEĎ 60 % → **`JEDU_LAJNU`**.

---

## 8. Stav `VRACIM_DOMU`

> Robot se potřebuje vrátit do HOME zóny — zásobník je plný nebo dostává pokyn.  
> Manévr: **couváme zády** do HOME (přesný výjezd zadní stranou).

### Krok 0 — natočení zády k domovu
- Vypočti relativní úhel k HOME středu (X=2150, Y=350).
- `rel_zacouvani = uhel_k_domovu - 180°` (chceme couvat, ne jet čelem).
- Pokud `|rel_zacouvani| > 10°`:
  - rel ≥ 0 → otoč vpravo.
  - rel < 0 → otoč vlevo.
  - `krok = 1`.
- Jinak rovnou `krok = 2`.

### Krok 1 — čekání na otočení
- Po dokončení → `krok = 2`.

### Krok 2 — couvnutí celé vzdálenosti k HOME
- Vydej příkaz `COUVEJ(vzdálenost k domovu)`.
- `krok = 3`.

### Krok 3 — sledování couvání
Každý frame:
- Pokud `vzdálenost k HOME < 150 mm`:
  - `STOP` → **`VYKLADAM`** (krok 21, přeskočí cestu a rovnou srovnání orientace).
- Pokud drift > 20°:
  - `STOP` → `krok = 0` (znovu zamiř a couver znovu).

---

## 9. Stav `VYKLADAM`

> Robot je v HOME zóně a vykládá puky. Vícekrokový manévr.

### Krok 0 — volba cesty do HOME (odkud přijíždíme)
- **Cesta A** (přijeli jsme DOLEVA, jsme na levé straně arény):
  - Otoč se 180° vpravo (k HOME).
  - `krok = 10`.
- **Cesta B** (přijeli jsme DOPRAVA, jsme blízko HOME):
  - Rovnou `krok = 20`.

### Kroky 10–12 — cesta A: jízda doprava ke HOME
- **Krok 10:** Po dotočení → JEĎ 60 % → `krok = 11`.
- **Krok 11:** Jedeme doprava:
  - Soupeř blokuje? → `STOP` → `krok = 12` (čekání).
  - Dosáhli jsme pravé strany (X ≥ 2250 mm)? → `STOP` → `krok = 20`.
- **Krok 12:** Čekáme na uvolnění:
  - Soupeř se vzdálil → JEĎ 60 % → `krok = 11`.

### Krok 20 — natočení nahoru
- Jsme v HOME a míříme doprava (90°) → otoč vlevo 90° (míříme nahoru, 0°).
- `krok = 21`.

### Krok 21 — přesné srovnání orientace (LiDAR srovnání)
- Po dotočení: spočítej chybu headingu (`h_err = normalize(heading)`).
- Pokud `|h_err| > 0.5°`:
  - Koriguj otočením vlevo/vpravo.
  - `krok = 22`.
- Jinak rovnou `krok = 30`.

### Krok 22 — čekání na korekci
- Po dotočení → `krok = 30`.

### Krok 30 — kontrola, zda jsme opravdu v HOME
- Jsme-li mimo HOME (X < 1800 nebo Y > 700)?
  - Log varování → přechod **`VRACIM_DOMU`**.
- Jinak → **otevři zásobníky** (`VYLOZ`) → `krok = 31`.

### Krok 31 — čekání na otevření zásobníků
- Po otevření (1 s) → JEĎ **40 % vpřed** (pomalá jízda), zapamatuj čas, nastav zbývající čas 1.5 s → `krok = 40`.

### Krok 40 — pomalá jízda vpřed při vykládání
- **Soupeř v cestě?**
  - `STOP`, odečti uplynulý čas od zbývajícího → `krok = 45`.
- Uplynul zbývající čas (1.5 s)?
  - `STOP` → `krok = 50`.

### Krok 45 — čekání na uvolnění soupeře při vykládání
- Soupeř se vzdálil → JEĎ 40 %, znovu zapamatuj čas → `krok = 40`.

### Krok 50 — zavření zásobníků
- `ZAVRI` → `krok = 51`.

### Krok 51 — dokončení výkladu
- Po zavření (1 s):
  - `počet_puků = 0` (reset zásobníku).
  - Zaloguj % pokrytí mapy.
  - `krok = 60`.

### Krok 60 — příprava na další kolo
- Nastav příznak `uz_vylozil = True`.
- Přechod → **`CEKAM_NA_START`** (čeká na druhé SPACE).

---

## 10. Druhá jízda — `PRESUN_Y` + `PRESUN_X`

> Po prvním vyložení čeká robot na SPACE. Po druhém SPACE spustí **dynamický režim**:  
> místo zig-zagu od začátku jde cíleně na největší nevyčištěný úsek mapy.

### Výpočet cíle (`vypocti_dalsi_cil`)
- Prochází celou mapu 10×10 buněk.
- Najde nejdelší **nepokrytou sekvenci buněk** v jednom řádku (Y=const).
- Vrátí: `(start_x, end_x, target_y, row_index)`.
- Souřadnice jsou oříznuty na bezpečný okraj arény.

### Stav `PRESUN_Y` — přesun na správné Y

**Krok 0:**
- Spočítej `dy = cil_Y - aktuální_Y`.
- Pokud `|dy| < 30 mm` → rovnou `krok = 3`.
- Jinak nastav heading na 0° (nahoru) nebo 180° (dolů).
- Pokud heading nesedí (chyba > 3°) → otoč.
- `krok = 1`.

**Krok 1:**
- Po dotočení → JEĎ 60 % → `krok = 2`.

**Krok 2:**
- Každý frame:
  - Soupeř v cestě? → `STOP` → `krok = 20` (čekání).
  - `|dy| ≤ 25 mm`? → `STOP` → `krok = 3`.

**Krok 20 — čekání na uvolnění:**
- Volno → JEĎ 60 % → `krok = 2`.

**Krok 3:**
- Po zastavení → **`PRESUN_X`**.

### Stav `PRESUN_X` — přesun na správné X + spuštění lajny

**Krok 0:**
- Rozhodni, ke kterému konci úseku jsme blíž (start_x nebo end_x).
- Nastav `smer_doprava` a `cil_x` odpovídajícím způsobem.
- Pokud `|dx| < 30 mm` → rovnou `krok = 3`.
- Otoč se na 90° (vpravo) nebo -90° (vlevo), `krok = 1`.

**Krok 1:**
- Po dotočení → JEĎ 60 % → `krok = 2`.

**Krok 2:**
- Každý frame:
  - Soupeř v cestě? → `STOP` → `krok = 21` (čekání).
  - Dosáhli cílového X? → `STOP` → `krok = 3`.

**Krok 21 — čekání:**
- Volno → JEĎ 60 % → `krok = 2`.

**Krok 3:**
- Po zastavení: natočit se do směru lajny (90° nebo -90°), `krok = 4`.

**Krok 4:**
- Po dotočení → JEĎ 60 % → **`JEDU_LAJNU`** (normální čištění, ale mapa-guided).

---

## 11. Stav `NOUZOVY_NAVRAT`

> Aktivuje se automaticky, když zbývá ≤ 10 s do konce zápasu, robot ještě nevyložil.

**Krok 0:**
- Vypočti úhel k HOME (`domov_uhel()`).
- Pokud > 10°:
  - Otoč se správným směrem (L/R) → `krok = 1`.
- Jinak → `krok = 2`.

**Krok 1:**
- Po dotočení → `krok = 2`.

**Krok 2:**
- JEĎ **90 % (maximum!)** → `krok = 3`.

**Krok 3:**
- Každý frame: Pokud vzdálenost k HOME < 150 mm:
  - `STOP` → **`VYKLADAM`** (ihned vyložit).

> ⚠️ Nouzový návrat nepočítá se soupeřem — jede plnou rychlostí přímo k domovu!

---

## 12. Globální přepínač: Nouzový návrat (časovač)

Každý frame, v `update()`, se kontroluje:
```
zbývá < 10 s
AND stav != NOUZOVY
AND stav != VYKLADAM
AND uz_vylozil == False
→ STOP + NOUZOVY_NAVRAT
```
Tato kontrola má **absolutní prioritu** nad vším (kromě aktivního vykládání).

---

## 13. Mapa pokrytí

- **10×10 buněk**, každá 250×250 mm.
- Při každém pohybu vpřed (`_jedu == True`) se označí buňky kolem středu robota (poloměr 150 mm, v rastrech po 125 mm).
- Buňka se označí jako `True` (pokryta).
- Dynamický režim hledá nejdelší sekvenci `False` buněk v jednom řádku Y.

---

## 14. Soupeř — detekce

```
_sup_v_ceste():
  vzdálenost < 500 mm
  AND úhel od čelního směru robota < 45°
  → True (blokuje jízdu)

_sup_volno():
  vzdálenost >= 650 mm
  OR úhel >= 45°
  → True (bezpečné pokračovat)

_sup_vSmeru(target_heading, max_dist, fov):
  teoretický lidar v zadaném směru
  → True, pokud je soupeř v tom sektoru
```

---

## 15. Stručný flowchart celého zápasu

```
START
  │
  └─► CEKAM ──[SPACE]──► NAJEZD_NAHORU
                              │ (dosáhl Y lajny 0)
                              ▼
                         JEDU_LAJNU ◄──────────────────────────┐
                         │  │  │  │                             │
               [plný]    │  │  │  │[konec lajny]               │
                  ▼      │  │  │  └──► PRECHOD_LAJNY ──────────┘
            VRACIM_DOMU  │  │  │
                  │   [bump]│  └──► [dno] VYKLADAM
                  │      │  │                │
                  │      │[soupeř]           └──► CEKAM (krok 60)
                  │      ▼                         │
                  │   VYHYBAM_SE ────────────────► │ [2. SPACE]
                  │                                ▼
                  └─────────────────► VYKLADAM  PRESUN_Y
                                          │        │
                                     [reset]    PRESUN_X
                                      CEKAM        │
                                                JEDU_LAJNU ...

Kdykoli (≤10s): ──────────────────────────────────────────────► NOUZOVY_NAVRAT
                                                                      │
                                                                 VYKLADAM
```

---

## 16. Přehled přechodů stavů

| Ze stavu | Podmínka | Do stavu |
|---|---|---|
| CEKAM | SPACE (první) | NAJEZD_NAHORU |
| CEKAM | SPACE (druhý) | PRESUN_Y |
| NAJEZD_NAHORU | Y ≥ lajna_y[0] | JEDU_LAJNU |
| NAJEZD_NAHORU | soupeř nebo bump | JEDU_LAJNU (rychle) |
| NAJEZD_NAHORU | plný zásobník | VRACIM_DOMU |
| JEDU_LAJNU | plný zásobník | VRACIM_DOMU |
| JEDU_LAJNU | soupeř vpředu | VYHYBAM_SE |
| JEDU_LAJNU | bump + není dno | PRECHOD_LAJNY |
| JEDU_LAJNU | bump + je dno | VYKLADAM |
| JEDU_LAJNU | konec lajny + není dno | PRECHOD_LAJNY |
| JEDU_LAJNU | konec lajny + je dno | VYKLADAM |
| JEDU_LAJNU | konec lajny + dyn. mapa | PRESUN_Y nebo DOMU |
| PRECHOD_LAJNY | dokončen manévr | JEDU_LAJNU |
| PRECHOD_LAJNY | soupeř na přechodu | JEDU_LAJNU (zpět) |
| VYHYBAM_SE | odúhybnutí + volno | předchozí stav |
| VYHYBAM_SE | alternativní únik | JEDU_LAJNU |
| VRACIM_DOMU | vzdálenost < 150 mm | VYKLADAM |
| VYKLADAM | krok 60 | CEKAM |
| VYKLADAM | mimo HOME zónu | VRACIM_DOMU |
| PRESUN_Y | Y dosaženo | PRESUN_X |
| PRESUN_X | X dosaženo | JEDU_LAJNU |
| (kdykoli) | ≤ 10 s do konce | NOUZOVY_NAVRAT |
| NOUZOVY_NAVRAT | vzdálenost < 150 mm | VYKLADAM |
