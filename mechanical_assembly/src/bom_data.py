"""BOM data for the robot rail base v3 (prices checked 2026-10-17)."""

EUR_RSD = 117.3931            # NBS srednji kurs
EUR_RSD_DATE = "15.10.2026"
VAT = 0.20

TELIX = "Telix (Novi Sad)"
TELIX_PDF = "https://www.telix.rs/wp-content/uploads/katalog.pdf#page={}"
TELIX_ZIP = "https://www.telix.rs/wp-content/uploads/3D%20modeli%20-%20Telix%20ALP%20katalog.zip"
PARAM = "parametarski model (kataloške mere)"

ITEMS = [
    {"section": "A. Aluminijumski profili i usluge – Telix ALP sistem (cene EUR bez PDV, katalog mart 2026)"},
    dict(code="ALP0047", name="Profil 45x90",
         spec="EN AW 6060 T66, slot 10, jezgro M12, 2,96 kg/m. 2 × 3000 mm (cele šipke od 3 m), "
              "postavljeni na kant (90 vertikalno), svetli razmak 500 mm.",
         supplier=TELIX, url=TELIX_PDF.format(20), unit="m", price=19.13, cur="EUR", vat_incl=False,
         qty=6, cad=TELIX_ZIP,
         note="Cena važi za šipke od 3/6 m. Tražiti ravno odsečene krajeve ili sečenje na 3000 ±2 mm (+10% i 0,60 €/rez)."),
    dict(code="ALP0045", name="Profil 45x45",
         spec="EN AW 6060 T66, slot 10, jezgro M12, 1,57 kg/m. Sečeno: 4 × 500 mm (distanceri) "
              "+ 8 × 100 mm (vertikalne noge) = 2,8 m (iz jedne šipke od 3 m).",
         supplier=TELIX, url=TELIX_PDF.format(18), unit="m", price=10.19, cur="EUR", vat_incl=False,
         qty=2.8, cad=TELIX_ZIP),
    dict(code="USL-SEC", name="Sečenje na meru – doplata 10%",
         spec="Telix: kod sečenja na željenu dužinu cena profila se uvećava za 10% (10% × 2,8 m × 10,19 €/m).",
         supplier=TELIX, url=TELIX_PDF.format(18), unit="paušal", price="=0.1*2.8*10.19", cur="EUR",
         vat_incl=False, qty=1, cad="—", in_model="—"),
    dict(code="USL-REZ", name="Sečenje – rez",
         spec="0,60 € po rezu; 12 komada iz šipke 45x45 → 12 rezova (tolerancija ±2 mm).",
         supplier=TELIX, url=TELIX_PDF.format(18), unit="rez", price=0.60, cur="EUR", vat_incl=False,
         qty=12, cad="—", in_model="—"),
    {"section": "B. Spojni elementi – Telix"},
    dict(code="ALP2137", name="Ugaona spojnica 45-visoka (Vis. sp. za 45x45)",
         spec="Aluminijum, krakovi 91 mm, širina 40 mm, 2 ovalne rupe Ø9 po kraku (4 vijka po spojnici). "
              "12 kom: distanceri → 45x90 (srednji 2 po strani, krajnji 1); 16 kom: noge (srednje: 2 na donju "
              "stranu 45x90; krajnje: 1 na donju stranu 45x90 + 1 na donju stranu krajnjeg distancera).",
         supplier=TELIX, url=TELIX_PDF.format(80), unit="kom", price=5.09, cur="EUR", vat_incl=False,
         qty=28, cad=TELIX_ZIP, note="Viša nosivost od klasične ALP2135."),
    dict(code="ALP2138", name="Poklopac 45 (za ugaonu spojnicu 45-visoku)",
         spec="Poliamid, crni; na svaku ALP2137 (distanceri i noge).",
         supplier=TELIX, url=TELIX_PDF.format(80), unit="kom", price=1.46, cur="EUR", vat_incl=False,
         qty=28, cad=TELIX_ZIP),
    dict(code="ALP0175", name="Vijak M8x20",
         spec="8.8, šestougaona glava sa nazubljenom prirubnicom i nanetim lepkom za navoj; "
              "4 kom po ugaonoj spojnici ALP2137 (28 × 4).",
         supplier=TELIX, url=TELIX_PDF.format(30), unit="kom", price=0.19, cur="EUR", vat_incl=False,
         qty=112, cad=TELIX_ZIP),
    dict(code="ALP0643", name="Klizna navrtka M8 T10",
         spec="M8x20x20, čelik pocinkovan (najjača Telix navrtka za slot 10). 112 kom za ugaone spojnice "
              "+ 76 kom za vijke šina (2 × 38) + 8 kom za ručke.",
         supplier=TELIX, url=TELIX_PDF.format(38), unit="kom", price=0.30, cur="EUR", vat_incl=False,
         qty=196, cad=TELIX_ZIP,
         note="Ubacuje se samo sa kraja profila – sve navrtke ubaciti pre montaže čepova. "
              "Alternativa koja se ubacuje naknadno: elastična ALP0529 (0,24 €)."),
    dict(code="ALP4017", name="Poklopac 45x90 (čep)",
         spec="PA, utiskuje se u kraj profila. Samo na zadnjem kraju (Y = 3000); na početku šine "
              "čepovi su izostavljeni.",
         supplier=TELIX, url=TELIX_PDF.format(157), unit="kom", price=0.85, cur="EUR", vat_incl=False,
         qty=2, cad=TELIX_ZIP),

    dict(code="ALP4112", name="Ručka 120",
         spec="Aluminijum, 140 × 40 × 25 mm, razmak rupa 120 mm, rupe Ø8,5. Na spoljnim stranama profila 45x90, "
              "gornji slot, između nogu 1–2 i 3–4 (X = 515 i 2485 mm), 4 kom.",
         supplier=TELIX, url=TELIX_PDF.format(132), unit="kom", price=1.87, cur="EUR", vat_incl=False,
         qty=4, cad=TELIX_ZIP, note="Pričvršćena sa 2 × DIN 912 M8x20 + 2 × ALP0643."),

    {"section": "C. Točkovi i stope – Telix"},
    dict(code="ALP5527", name="Točak sa stopom CD80 (Toč. sa sto. CD80)",
         spec="Točak sa ugrađenom stopom za nivelaciju: točak 60x27, navoj M12×25, H105, nosivost 500 kg. "
              "Na 4 krajnje noge (na samim krajevima profila).",
         supplier=TELIX, url=TELIX_PDF.format(182), unit="kom", price=11.50, cur="EUR", vat_incl=False,
         qty=4, cad=TELIX_ZIP,
         note="Uvrće se u jezgro noge narezano M12 (≥30 mm). Robot radi oslonjen na stope, ne na točkove."),
    dict(code="ALP2512", name="Stopa fi39 Al (gumena podloga)",
         spec="Aluminijumsko telo Ø39 sa gumenom podlogom (trenje pri kretanju robota), sferni zglob, "
              "nosivost 1500 kg (najveća među Telix stopama sa gumom). Na 4 srednje noge.",
         supplier=TELIX, url=TELIX_PDF.format(170), unit="kom", price=2.55, cur="EUR", vat_incl=False,
         qty=4, cad=TELIX_ZIP,
         note="Nabija se gumenim čekićem na brezon ALP2613. Alternativa: ALP2511 Stopa fi75 (3000 kg, 4,84 €) – "
              "u Telix STEP-u ima gumenu podlogu, ali katalog to ne navodi; potvrditi sa Telix-om."),
    dict(code="ALP2613", name="Brezon M12x100",
         spec="Navojni stub M12×100 sa kuglom SR7,5 i kontra navrtkom, za stopu ALP2512; uvrće se u jezgro "
              "noge (≈18 mm navoja), nivelacija odvrtanjem.",
         supplier=TELIX, url=TELIX_PDF.format(171), unit="kom", price=2.61, cur="EUR", vat_incl=False,
         qty=4, cad=TELIX_ZIP),

    {"section": "D. Linearno vođenje – Mehatron (cene RSD sa PDV)"},
    dict(code="550010", name="Profilisana šina GD35 (Saier), dužina 300 cm",
         spec="Šina 34×29 mm, rupe Ø9/Ø14×12 za M8, korak 80 mm, 38 rupa na 3000 mm (E = 20 mm). "
              "Ekvivalent HIWIN HGR35R. Montirana na gornju stranu (45 mm) oba profila 45x90.",
         supplier="Mehatron (Kragujevac)", url="https://mehatron.rs/profilisana-sina-gd35", unit="kom",
         price=25200, cur="RSD", vat_incl=True, qty=2, cad=PARAM + " – Saier SER-GD / HIWIN HGR35",
         note="8.400 RSD/m. Tražiti simetrično sečenje E = 20 mm i informaciju o klasi prednaprezanja/tačnosti."),
    dict(code="550085", name="Kolica za profilisanu šinu GD35WA (Saier) – široka, sa prirubnicom",
         spec="W100 × L110, H = 48 mm od dna šine, 4 × M10 na 82 × 62 (interfejs = HIWIN HGW35CC). "
              "2 kolica po šini.",
         supplier="Mehatron (Kragujevac)", url="https://mehatron.rs/kolica-za-profilisanu-sinu-gd35wa",
         unit="kom", price=5280, cur="RSD", vat_incl=True, qty=4, cad=PARAM + " – Saier SER-GD / HIWIN HGW35CC",
         note="Ne mešati brendove šine i kolica."),
    dict(code="HG35-LIM", name="Stoper (graničnik) za šinu veličine 35 – 'HG35 limit block' sa gumenim odbojnicima",
         spec="U-blok 60 × 20 × 35 mm koji se steže na šinu, 2 gumena odbojnika ka kolicima. Postavljen "
              "iznad poslednje rupe šine (20 mm od kraja) – vijak za stezanje ulazi u upust te rupe.",
         supplier="AliExpress (uvoz)", url="https://www.aliexpress.com/item/1005013023414852.html",
         unit="kom", price=983, cur="RSD", vat_incl=False, qty=4, cad=PARAM,
         note="U Srbiji nema stopera za veličinu 35 (Vallder HGR-Lim samo 15–30, Mehatron nema). "
              "Okvirna cena bez dostave; PDV pri uvozu uračunat kroz formulu."),

    {"section": "E. Ploča kolica (nosač robota FR20)"},
    dict(code="PL-5083-25", name="Aluminijumska brušena ploča EN AW-5083, 25 mm – isečena na 700 × 600 mm",
         spec="Materijal za ploču kolica: 700 (poprečno) × 600 (duž hoda) × 25 mm, ≈ 27,9 kg pre obrade. "
              "Brušena ploča zbog ravnosti (4 kolica).",
         supplier="Prodaja Metala (Novi Sad)",
         url="https://prodajametala.rs/proizvod/aluminijum-brusena-ploca-1500x3000mm-t25-en-aw-5083",
         unit="kg", price=780, cur="RSD", vat_incl=False, qty=27.9, cad="parametarski model (vlastiti dizajn)",
         note="Cena po kg bez PDV; sečenje na format – cena sečenja nije objavljena. Proveriti telefonom 021/641-2285."),
    dict(code="CNC-PLOCA", name="CNC obrada ploče kolica (po crtežu)",
         spec="16 × Ø11/Ø17,5×10,5 upust za M10 (kolica, 82 × 62, osni razmak kolica 400 mm, šine 545 mm); "
              "6 × M10 navoj kroz na PCD 210 (FR20, 30° + k·60°); 2 × Ø8 H7 prolazne (čivije); gravirana strelica +X.",
         supplier="po ponudi (npr. Delta Tehnik Marold, Novi Sad)", url="https://deltatehnik.rs/usluga/usluga-glodanja/",
         unit="kom", price=0, cur="RSD", vat_incl=False, qty=1, cad="—", in_model="da",
         note="NIJE uračunato u ukupnu cenu – tražiti ponudu (Delta Tehnik Marold ili MS Technologies) sa STEP crtežom."),
    dict(code="00841030", name="Vijak cilindrične glave imbus DIN 912 M10x30, 8.8, pocinkovan",
         spec="Ploča → kolica GD35WA (4 × M10 po kolicima, 4 kolica).",
         supplier="Würth Srbija",
         url="https://eshop.wurth.rs/vijak-cilind-gl-imbus-din-912-c-poc-8-8-m10x30-vijci-i-vijcane-veze",
         unit="kom", price=48, cur="RSD", vat_incl=True, qty=16, cad=PARAM, note="Min. pakovanje 25 kom (1.200 RSD)."),
    dict(code="00841050", name="Vijak cilindrične glave imbus DIN 912 M10x50, 8.8, pocinkovan",
         spec="Robot FR20 → ploča: 6 kom kroz prirubnicu baze (25 mm) u navoj ploče (25 mm).",
         supplier="Würth Srbija",
         url="https://eshop.wurth.rs/vijak-cilind-gl-imbus-din-912-c-poc-8-8-m10x50-vijci-i-vijcane-veze",
         unit="kom", price=41, cur="RSD", vat_incl=True, qty=6, cad="—", in_model="ne (robot nije u modelu)",
         note="Min. pakovanje 100 kom. M10x45 nema u Würth eshopu. Proveriti debljinu prirubnice FR20 na crtežu."),
    dict(code="11980024", name="Cilindrična čivija DIN 6325 m6 Ø8 × 24",
         spec="Kaljena brušena čivija za tačno pozicioniranje baze robota: utiskuje se u prolaznu rupu Ø8 H7 u ploči, "
              "viri 9 mm i ulazi u rupu/žleb Ø8 na dnu baze FR20.",
         supplier="AMI d.o.o.", url="https://ami.rs/proizvod/cilindricna-civija-din-6325-m6-8x24",
         unit="kom", price=41.62, cur="RSD", vat_incl=True, qty=2, cad=PARAM),

    {"section": "F. Vijci za šine i ručke"},
    dict(code="0084830", name="Vijak cilindrične glave imbus DIN 912 M8x30, 8.8, pocinkovan",
         spec="Šina → klizna navrtka ALP0643 u gornjem slotu 45x90: 17 mm šina + 6 mm zid slota + navrtka; "
              "38 kom po šini.",
         supplier="Würth Srbija",
         url="https://eshop.wurth.rs/vijak-cilind-gl-imbus-din-912-c-poc-8-8-m8x30-vijci-i-vijcane-veze",
         unit="kom", price=30, cur="RSD", vat_incl=True, qty=76, cad=PARAM,
         note="Minimalno pakovanje 100 kom (3.000 RSD). Ne M8x35 (udara u dno slota, dubina 14,5 mm); "
              "bez podloški (Ø16 ne staje u upust Ø14)."),
    dict(code="0084820", name="Vijak cilindrične glave imbus DIN 912 M8x20, 8.8, pocinkovan",
         spec="Ručka ALP4112 → klizna navrtka (2 po ručki).",
         supplier="Würth Srbija",
         url="https://eshop.wurth.rs/vijak-cilind-gl-imbus-din-912-c-poc-8-8-m8x20-vijci-i-vijcane-veze",
         unit="kom", price=23, cur="RSD", vat_incl=True, qty=8, cad=PARAM, note="Min. pakovanje 100 kom."),
]

NOTES = [
    "Izvori cena (provereno 17.10.2026): Telix katalog-cenovnik mart 2026 (EUR, bez PDV); Mehatron, Würth, AMI – cene sa sajta "
    "sa PDV; Prodaja Metala – RSD/kg bez PDV; AliExpress – okvirno. CNC obrada ploče nije uračunata (po ponudi).",
    "Telix nema web-shop: porudžbine na office@telix.rs / +381 21 489 0 489; preko 50 € bez manipulativnih troškova. "
    "Linkovi vode na stranu kataloga.",
    "Koordinatni sistem modela: X = pravac hoda (X = 0 početak bez čepova), Y poprečno, Z gore, Z = 0 pod. "
    "X ose robota poklapa se sa X osom raila (strelica +X ugravirana na ploči).",
    "Profili 45x90 na kant, svetli razmak 500 mm (osni 545 mm); distanceri 45x45 × 500 mm u donjih 45 mm, osni razmak 985 mm.",
    "Noge su ispod distancera (osni razmak 985 mm, krajnje na samim krajevima); svaka ima 2 ugaone spojnice ALP2137 – "
    "krajnje: jedna ka sredini na donju stranu 45x90 i jedna na donju stranu krajnjeg distancera.",
    "Ručke u gornjem slotu: između vrha ručke i dna ploče kolica ostaje ≈ 57 mm – pazi na prste kada ploča prolazi.",
    "Visine od poda: donja ivica noge 112 mm, vrh profila 302 mm, vrh šine 331 mm, vrh kolica 350 mm, vrh ploče 375 mm.",
    "Stoperi iznad prve/poslednje rupe šine: hod centra ploče ≈ 291 … 2709 mm (≈ 2418 mm).",
    "Baza FR20 (izmereno na meshu fairino_description): prirubnica Ø240, 6 × Ø10,5 na PCD 210, na dnu okrugla rupa "
    "Ø8 i žleb Ø8 za čivije – prečnik i položaj čivija potvrditi sa zvaničnim crtežom FAIRINO pre obrade ploče.",
    "Jezgro nogu 45x45 narezati M12 (≥25 mm za stope, ≥30 mm za točkove); ureznica M12 (npr. Würth 0639712, 3.990 RSD) "
    "nije uračunata.",
    "Stoperi tipa 'limit block' nisu udarni graničnici; za robota predvideti i softverske limite / tvrdi graničnik sa "
    "odbojnikom. Domaća alternativa: AMI Niš HK 3501 A stega (75 € bez PDV/kom).",
    "Besplatni STEP za GD35/GD35WA/stoper ne postoji bez logovanja, pa su ti delovi parametarski modelovani.",
    "Nije obuhvaćeno: pogon (zupčasta letva, motor), kablovski lanac, sidrenje u pod, robot.",
]
