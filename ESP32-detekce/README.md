# Autonomous ESP32 LiDAR Detection System

Tento projekt řeší logiku precizní prostorové orientace robota (vysavače) pomocí 360° laserového skeneru **LD19 (D200)** přímo na mikrokontroleru **ESP32**. Cílem není jen hloupě číst vzdálenost k prázdnému bodu, ale **plně matematicky analyzovat geometrii 2D herního pole** a vytvořit komplexní přehled o světě jen ze samotného shluku paprsků. 

Systém předává řídícímu mozku suverénní data typu: _"Mám tu dominantní stěnu pod úhlem 15 stupňů a přesně 40 cm po pravé straně je cizí valounovitá překážka (SOUPEŘ)."_

## 1. Průběh a Mrazení času (Tick)
Náš LiDAR se otáčí kolem své osy a chrlí stovky bodů za sekundu. Systém na ESP32 má nastaven "časový framer" každých **120 milisekund**. Jakmile pípne 120 ms, přijímané body se seříznou čistě na 180° výhled vpředu a celý bodový mrak (point-cloud) se na zlomek milisekundy pro výpočet zmrazí. Díky tomu máme konzistentní surová data a nedochází k trhání zdi za jízdy.

## 2. Vyhledávání stěn (Globální RANSAC a Kanibalismus)
Staré koncepty, jako dělení obrazu do "Zón Levá/Pravá", systém opustil, protože šikmý roh stolu dokáže prolínat zóny nezávisle a rozhodí algoritmus.
Namísto toho pohlížíme na body globálně pomocí algoritmu **RANSAC** (Random Sample Consensus).

1. **Nahazování sítí**: Ve světě teček si algoritmus náhodně vyberete 2 body a natáhne přes ně fiktivní nekonečnou stěnu (vyhodí přes 200 hypotéz pro jednu jedinou reálnou stěnu). 
2. **Neústupné Skórování**: Ke každé nahozené stěně spočítá, kolik dalších reálných dopadů laseru oné zdi dává za pravdu – stvoří si "tloušťkové pravítko" s odchylkou pouhých `+- 1.5 cm`. A co rozhoduje o výhře? Skóre závisí **absolutně na hustotě** (vysoký počet bodů znamená pevnou silnou zeď). Pokud jsou dvě zdi na milimetr husté, délka stěny funguje jako drobný tie-breaker.
3. **Bodový Kanibalismus**: Čára s nevyšší validací vyhraje a zničí všechny svoje obsažené body! Ty zmizí ze seznamu. Kdybychom to neudělali, křížily by se další nalezené slabší rohy a soupeře přes ty samé tlusté stěny, jako falešné odrazy.

## 3. Matematická Paměť os (Stabilizace a "Flickering")
Největším problémem počítačového vidění je proskakování. Pokud vedle sebe stojí popelnice a za ní stěna obývací stěny, RANSAC bude skákat od kusu stěny k další a robotovi bude souřadnicový kompas jezdit pod rukama. Zavedl se proto systém **Držení Paměti (Hystereze)** a Ortogonální svěrák:

* **Zámek a Učení**: Zcela čerstvý systém změří nejdřív 20x stejnou dominantní stěnu, než jí naplno uvěří a uloží si hluboko do paměti její úhel v prostoru (ZÁMEK!).
* **Přežití rotace**: Kdykoliv robot vytvoří nový Snímek aktuálního světa (Frame), snaží se vždy hledat svou paměťovou Žlutou Stěnu, a bez výčitky zamítne nového "přirozeného tlustého vítěze", pokud nesouhlasí se starou známou dominantou. Toleranci má v širokém úhlu `+- ~30°`, aby robot dokázal na dominantu rotovat za rohlíkem!
* **Ortogonální pravítko**: Tím, že máme Žlutou paměť zatlučenou o hřebík, všechny podružné, následně počítané zdi, spadající na levou a pravou stranu musí s dominantou udržovat striktní Mřížku kolmosti. Mají odchylku brutálních pouhých `+- 10°`, čímž algoritmus eliminuje všechny tupé "zkřížené dlouhé čáry letící přes stoly".
* **Amnézie a Amnestie**: Když robot projede masivní zatáčku a vjede do pole prázdnoty a stará Dominanta už je fyzicky mimo dosah senzoru, chvilku hledá... Pošle do systému "nic"... a po 5 plných výpadcích (`MEM_LIFESPAN >= 5`) praskne Zámek a robot se během zlomku vteřiny adaptuje na celou plynulou novou síť pokoje kolem něj.

## 4. PCA Filtr a izolace Soupeře
Poté co si Stěny sežerou všechny obří rolové tečky, zbyde nám obvykle několik mráčků teček plujících volně v prostředku. Tohle mohou být kusy zapomenutého stínu okna, a samozřejmě onen Soupeř.

1. **Blobování**: Body poblíž sebe se stáhnou do celistvé hmoty (clusteru). Musí jich být víc jak 5.
2. **Přísná metoda PCA (Matice rozptylu komponent)**: Nemůžeme považovat hubenou osamělou čáru za "Soupeře". Robot na blob chumlu uplatní vektorovou matematiku (PCA). Natahuje na chumel jakousi 2D pláštěnku a matematicky izoluje "hlavní vnitřní rozpětí" a "kolmou tloušťku oblaku".
3. Pokud má blob tloušťku menší než `1.3 cm` a víc jak `3 cm` do délky, ESPčko ho chladnokrevně odpraví se slovy: _"Tohle je jen ohlidanej zapomenutej rovný krajní kus zdi, nic jinýho"_ a terč zkrátka nevyhodí! Soupeře tak vždy definuje skutečně olemovaný, buclatý tvar.

## 5. Vizualizér v Pythonu
Vizuální obrazovka z Pythonu nedělá ani výpočet, ani vyhlazování - je to pouze věrný papírový notýsek hlásící to, jak to má ESP32 aktuálně přebrané ve svých registry v RAMce.

- Python používá fintu "Reset Snímku". Ve chvíli, kdy přijede z COMu balíček `Pozice Robota (Typ 2)`, Python s obrovskou radostí kompletně vyzmizíkuje čáry i kruhy na svém plátně a vygumuje ho. Čímž kompletně zničí možnost vizuálních "duchů" starých čar, a proto se vše sypavě překresluje do absolutní live reality stínově do sekundy - podle úhlu.
