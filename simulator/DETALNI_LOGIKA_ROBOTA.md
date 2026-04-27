# 🤖 Extrémně podrobný rozbor logiky robota (dle simulator.py)

Tento dokument slouží jako kompletní technický popis chování robota. Popisuje každý stav, každou podmínku "pokud/když" a přesné sekvence kroků, které robot vykonává.

---

## 1. Základní nastavení a orientace

### Souřadný systém
*   **Rozměr arény:** 2500 x 2500 mm.
*   **Osa X:** 0 (levá stěna) až 2500 (pravá stěna).
*   **Osa Y:** 0 (spodní stěna u nás) až 2500 (horní stěna u soupeře).
*   **HOME zóna:** Čtverec 700x700 mm v pravém dolním rohu.
*   **Navigační střed HOME:** X = 2150, Y = 350.

### Heading (Směr)
Robot používá navigační konvenci v stupních:
*   **0°** = Sever (+Y, nahoru)
*   **90°** = Východ (+X, doprava)
*   **-90°** = Západ (-X, doleva)
*   **180° / -180°** = Jih (-Y, dolů)

---

## 2. Globální hlídači (běží neustále)

Každý "frame" (každých cca 16ms) robot kontroluje tyto kritické podmínky, které mají přednost před aktuálním stavem:

1.  **Časová pojistka (Nouzový návrat):**
    *   **POKUD** zbývá do konce zápasu méně než 10 sekund **A ZÁROVEŇ** robot ještě nevykládal puky v tomto kole **A ZÁROVEŇ** už není ve stavu `NOUZOVY` nebo `VYKLADAM`.
    *   **POTOM:** Okamžitě zastaví všechno ostatní a přepne do stavu `NOUZOVY_NAVRAT`.

2.  **Aktualizace mapy:**
    *   **POKUD** se robot pohybuje vpřed.
    *   **POTOM:** Označí čtverec o velikosti robota pod sebou v mřížce 10x10 jako "pokryto".

3.  **Hlídání nárazu (Bumper):**
    *   **POKUD** dojde k sepnutí předního nárazníku.
    *   **POTOM:** Tato informace je uložena a zpracována v rámci stavu `JEDU_LAJNU` nebo `NAJEZD`.

---

## 3. Detailní rozbor stavového automatu

### Stav A: `CEKAM_NA_START`
*   **Co se děje:** Robot stojí na místě.
*   **Kdy končí:** Po stisknutí klávesy SPACE.
*   **Logika přechodu:**
    *   Pokud jde o **úplně první start**: Nastaví pozici na X=2250, Y=350, heading=0°, spustí čas a jde do `NAJEZD_NAHORU`.
    *   Pokud jde o **další start (po vyložení)**: Zapne dynamický režim, najde největší prázdný flek na mapě a jde do `PRESUN_Y`.

---

### Stav B: `NAJEZD_NAHORU`
Robot vyjíždí z domácí zóny směrem k horní stěně, aby začal první lajnu úplně nahoře.

*   **Krok 0:** Jede rovně nahoru (0°).
    *   **POKUD** je zásobník plný (10 puků) → Jdi do `DOMU`.
    *   **POKUD** vidí soupeře přímo před sebou (< 500 mm) → Zastav, otoč se vlevo 90° a začni lajnu předčasně (`JEDU_LAJNU`).
    *   **POKUD** narazí (Bumper) → Zastav, otoč se vlevo 90° a začni lajnu předčasně.
    *   **POKUD** dosáhne Y souřadnice první lajny (cca 2375 mm) → Zastav, otoč se vlevo 90°, `krok 1`.
*   **Krok 1:** Čeká na dotočení o 90°.
    *   **POTOM:** Nastaví cíl na X=250 (levá strana), rozjede se a jde do `JEDU_LAJNU`.

---

### Stav C: `JEDU_LAJNU`
Hlavní stav sbírání. Robot jede horizontálně z jedné strany na druhou.

*   **Pravidelné kontroly (každý krok):**
    1.  **Plný zásobník:** Pokud puky >= 10 → Zastav, jdi do `DOMU`.
    2.  **Soupeř v cestě:** Pokud soupeř < 500 mm před robotem → Zastav, jdi do `VYHYBAM_SE`.
    3.  **Náraz (Bumper):**
        *   Pokud je vysoko (Y > 580) → Jdi do `PRECHOD_LAJNY` (couvne a zkusí nižší lajnu).
        *   Pokud je už úplně dole (Y < 580) → Jdi do `VYKLADAM`.
    4.  **Dosažení konce lajny (Cílové X):**
        *   **Dynamický režim:** Pokud dojel úsek → Najdi další úsek nebo jdi domů.
        *   **Standardní režim:** Pokud je vysoko → Jdi do `PRECHOD_LAJNY`. Pokud je dole → Jdi do `VYKLADAM`.

---

### Stav D: `PRECHOD_LAJNY`
Složitý manévr pro přesun o jednu "řadu" níže.

*   **Krok 0 (Příprava):** Podívá se LiDARem dolů (180°).
    *   **POKUD** je tam soupeř → Otoč se o 180° a jeď zpět po stejné lajně (únik).
    *   **POKUD** je volno → Couvni 100 mm, `krok 1`.
*   **Krok 1 (Otočení dolů):**
    *   Po couvnutí se otočí o 90° směrem k "jihu" (180°). Směr otáčení (vlevo/vpravo) volí tak, aby točil směrem od zdi.
*   **Krok 2 (Sestup):** Rozjede se pomalu (40%) směrem dolů.
*   **Krok 3 (Hlídání sestupu):**
    *   Jede dolů cca 2 sekundy (aby ujel šířku robota 300 mm).
    *   Hlídá soupeře. Pokud se objeví pod ním → Únik (otočka 180° a zpět).
    *   Kontroluje, zda v nové lajně (vlevo/vpravo) není soupeř. Pokud je volno → Zastav, `krok 4`.
*   **Krok 4 (Natočení do nové lajny):** Otočí se o dalších 90° do směru nové lajny.
*   **Krok 5 (Start nové lajny):** Invertuje směr (pokud jel doprava, teď jede doleva), nastaví nové cílové X a jde do `JEDU_LAJNU`.

---

### Stav E: `VYHYBAM_SE`
Reakce na soupeře, který stojí v cestě v lajně.

*   **Strategie:** Robot se pokusí soupeře "podjet" spodem.
*   **Krok 0:**
    *   Pokud je robot u spodní zdi → Nemá kam uhnout, stojí a čeká, až soupeř odjede.
    *   Pokud má místo → Podívá se dolů, jestli tam není soupeř. Pokud volno → Otočí se o 90° dolů, `krok 1`.
*   **Krok 1:** Rozjezd dolů (40%).
*   **Krok 2 (Úhybný manévr):**
    *   Jede dolů cca 1.5 sekundy.
    *   Hlídá, jestli se uvolnil směr v původní lajně.
    *   Pokud je volno → Zastav, `krok 3`.
*   **Krok 3:** Otočí se zpět do původního směru (o 90° zpět).
*   **Krok 4:** Rozjede se a pokračuje v lajně tam, kde přestal.

---

### Stav F: `VRACIM_DOMU` (Couvání)
Robot se vrací do HOME zóny pro vyložení. Používá couvání, aby zásobník (v zadní části) byl blíž k výsypce.

*   **Krok 0:** Vypočítá úhel k středu HOME zóny. Otočí se tak, aby k němu stál **zády**.
*   **Krok 1:** Čeká na dotočení.
*   **Krok 2:** Spustí příkaz `COUVEJ` na celou vzdálenost k HOME.
*   **Krok 3:** Hlídá vzdálenost.
    *   Pokud je < 150 mm od cíle → Zastav, jdi do `VYKLADAM`.
    *   Pokud se cestou příliš vychýlí z kurzu → Zastav a znovu zamiř (zpět na krok 0).

---

### Stav G: `VYKLADAM` (Dump sekvence)
*   **Krok 0:** Pokud přijel z levé strany arény, musí se nejdřív otočit a dojet k pravé zdi k HOME. Pokud už tam je, jde na `krok 20`.
*   **Krok 20:** Otočí se čelem nahoru (0°).
*   **Krok 21-22:** Použije LiDAR k extrémně přesnému srovnání (chyba < 0.5°).
*   **Krok 30:** Kontrola pozice. Pokud je v HOME → Otevře zásobníky (`VYLOZ`).
*   **Krok 31:** Čeká 1 sekundu na otevření.
*   **Krok 40:** Pomalý popojezd vpřed (30 cm), aby se puky vysypaly na plochu HOME.
    *   Hlídá soupeře. Pokud soupeř blokuje popojezd → Čeká.
*   **Krok 50:** Zavře zásobníky (`ZAVRI`).
*   **Krok 51:** Vynuluje počítadlo puku, vypíše statistiku a jde do `CEKAM_NA_START`.

---

### Stav H: `NOUZOVY_NAVRAT`
Aktivuje se v posledních 10s zápasu.
*   Zahodí veškerou logiku lajn a mapy.
*   Zamíří přímo na HOME (čelem).
*   Rozjede se na **90% výkonu** (maximální rychlost).
*   Ignoruje soupeře i nárazy.
*   Jakmile je v HOME → Spustí sekvenci `VYKLADAM`.

---

### Dynamické stavy: `PRESUN_Y` a `PRESUN_X`
Používají se po prvním vyložení, aby robot nejezdil prázdná místa.
*   **`PRESUN_Y`**: Robot se nejdřív přesune na Y souřadnici největšího nepokrytého úseku.
*   **`PRESUN_X`**: Poté se přesune na začátek (X) tohoto úseku, natočí se a spustí standardní `JEDU_LAJNU`.

---

## 4. Logika rozhodování (Summary)

| Situace | Podmínka | Akce |
| :--- | :--- | :--- |
| **Puky** | Počet >= 10 | Přerušit vše, jet couváním domů. |
| **Soupeř** | Vzdálenost < 50 cm | Zastavit, zkusit objet spodem (úhyb). |
| **Konec lajny** | X dosaženo | Sjet o 30 cm níž a jet opačným směrem. |
| **Čas** | < 10 s do konce | Kašlat na puky, jet plnou rychlostí domů. |
| **Náraz** | Bumper sepnut | Couvnout a zkusit jinou cestu (přechod lajny). |
| **Čištění** | Mapa pokrytí | Vybírat úseky s největší hustotou "False" hodnot. |

---

## 5. Navigační triky v kódu

1.  **Hystereze u soupeře:** Robot zastaví při 500 mm, ale rozjede se až při 650 mm. Tím se zabrání "cukání" na hranici detekce.
2.  **Srovnání dle LiDARu:** Před vykládáním se robot srovná s přesností na půl stupně, aby se zásobníky otevíraly přesně rovnoběžně se zdí.
3.  **Inteligentní přechody:** Pokud robot při přejezdu na novou lajnu zjistí, že tam stojí soupeř, raději se vrátí na starou lajnu a zkusí ji jet opačně, místo aby se zasekl.
4.  **Clamping:** Všechny vypočítané cíle jsou prohnány funkcí `max/min`, aby robot nikdy nenavigoval "mimo arénu" nebo příliš blízko ke zdi, kde by mohl drhnout.
