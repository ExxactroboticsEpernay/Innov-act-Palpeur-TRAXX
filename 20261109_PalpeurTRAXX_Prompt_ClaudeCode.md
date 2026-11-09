# Projet : palpeurs de guidage pour le TRAXX (Exxact Robotics)

> À coller dans Claude Code à la racine d'un dossier vide `palpeur-traxx/`.
> Langue de travail : français (UI, doc, commentaires). Identifiants de code en anglais.

---

## 0. Façon de travailler (à respecter)

1. **Commence en mode plan.** Avant d'écrire du code : relis tout ce document, reformule en 10 lignes max ce que tu as compris, dessine la géométrie en schéma ASCII (§3) et liste tes questions. Attends ma validation.
2. Ne devine jamais un brochage, un GPIO, une tension. Si l'info n'est pas dans ce document, lis la doc officielle (Industrial Shields, Honeywell) ou demande-moi.
3. Travaille par phases (§6). À la fin de chaque phase : build OK, tests OK, commit, résumé de 5 lignes de ce qui change.
4. Code simple, lisible, commenté en français là où le « pourquoi » n'est pas évident. Pas de sur-ingénierie.
5. Aucune dépendance Internet à l'exécution (le PLC n'aura pas de connexion) : zéro CDN, tous les assets de la page web sont embarqués dans le firmware (LittleFS).
6. Périmètre : **lecture seule**. Pas d'actionneur, pas de boucle de commande. Les sorties du PLC ne sont pas utilisées.

---

## 1. Contexte et objectif

Le TRAXX est un robot viticole autonome. Je veux connaître sa **position dans le rang de vigne** avec des palpeurs mécaniques (bras à contact) :

- **Décalage latéral** de la machine par rapport à l'axe du rang.
- **Cap** (angle de lacet) de la machine par rapport au rang, avec 2 paires de palpeurs.
- Affichage en temps réel sur une **page web via Wi-Fi** (téléphone/tablette/PC).
- **À terme** : liaison **CAN** entre le PLC et l'ordinateur du TRAXX pour enregistrer ces données dans les **rosbag** du TRAXX.

Configuration cible : **4 palpeurs = 2 paires** (1 paire avant, 1 paire arrière), un palpeur de chaque côté du rang par paire. **Pour les premiers essais : 1 seule paire** (avant OU arrière, au choix dans la config). Le code doit gérer 1 ou 2 paires dès le départ.

Objets palpés : pieds de vigne, piquets, feuillage. Diamètre **1 à 30 cm**. Contact avec des objets de nature très variable → la mesure sera bruitée, avec des transitoires (piquet tous les quelques mètres, feuilles).

---

## 2. Matériel (faits à respecter)

### Capteurs : Honeywell **RTY180HVNAA** (×2 pour les essais, ×4 à terme)
- Capteur rotatif à effet Hall, **180° (±90°)**, version **HV**, pinout **N** (nord-américain), levier A.
- **Alimentation 10 à 30 Vdc** (version HV : PAS 5 V).
- **Sortie analogique 0,5 V (butée gauche) → 4,5 V (butée droite), NON ratiométrique** : la sortie ne dépend pas de la tension d'alimentation. Ce n'est donc pas du 0-5 V : plage utile = 4 V pour 180° soit ≈ 22 mV/°.
- Précision annoncée ±1 % de la plage (≈ ±1,8°, soit ≈ ±1 cm en bout de bras de 35 cm) : la calibration corrige offset et gain mais pas la non-linéarité résiduelle. À documenter.
- Conso max ≈ 32 mA par capteur. Connecteur 3 points (type AMP Superseal). IP69K.
- Alimentés en direct sur le **bus du TRAXX** (en parallèle du PLC), via fusible. **Pas d'alimentation depuis le PLC ni de DC/DC dédié** tant que le bus est dans 10–30 V. Si le bus est plus élevé (ex. 48 V), un DC/DC unique alimente PLC + capteurs. **Tension du bus TRAXX : à confirmer par moi.**
- Masse commune capteurs/PLC obligatoire. Câble blindé conseillé, blindage reprise côté PLC uniquement.

### Contrôleur : **Industrial Shields ESP32 PLC 14**
- ESP32, Wi-Fi, Ethernet, RS485, I2C, programmation Arduino.
- Alimentation PLC 12–24 V (à vérifier dans la datasheet).
- Entrées analogiques : 2 entrées 0-10 V / 4-20 mA (choix usine, pas modifiable par logiciel) + 2 entrées 0-10 V (aussi utilisables en digital). **Il faut la version 0-10 V**. Il y a exactement 4 entrées analogiques utilisables → 4 capteurs. **Version exacte de mon PLC : à confirmer par moi.**
- Mon capteur sort 0,5–4,5 V sur une entrée 0–10 V : on n'utilise qu'environ 40 % de la plage ADC (≈ 1600 points sur 180° en 12 bits → ≈ 0,11°/point au mieux). Suffisant, mais le bruit ADC de l'ESP32 est réel : prévoir sur-échantillonnage + filtrage + calibration par canal.
- **Piège ESP32 à vérifier en premier** : les broches ADC2 sont inutilisables quand le Wi-Fi est actif. Vérifie sur la doc Industrial Shields à quels GPIO sont reliées les 4 entrées analogiques. Si l'une est sur ADC2, propose une solution (ADC externe I2C type ADS1115, etc.) avant d'aller plus loin.
- **CAN** : la variante « w/ CAN Bus » = carte de communication optionnelle. **Je ne sais pas si mon PLC l'a** : prépare le code CAN derrière une abstraction, désactivée par défaut (§7).
- Brochage : utilise la bibliothèque et la doc officielles Industrial Shields (noms d'entrées type `I0.x`). Ne mets pas de numéros de GPIO « de mémoire ».

### Chaîne de build
- Cible par défaut : **PlatformIO + framework Arduino**. Vérifie d'abord que la carte Industrial Shields ESP32 PLC est utilisable proprement avec PlatformIO. Si ce n'est pas le cas, fallback : `arduino-cli` avec le package de cartes Industrial Shields. Dis-moi lequel tu retiens et pourquoi.
- Tests de la logique de calcul en **natif sur PC** (`pio test -e native`, Unity ou équivalent), sans matériel.

---

## 3. Modèle géométrique

### 3.1 Repère et conventions
- Repère machine : origine **O = centre de l'essieu arrière**, **x vers l'avant**, **y vers la gauche**, lacet ψ positif anti-horaire (vu de dessus).
- Une paire p ∈ {AV, AR} a ses deux pivots à la même abscisse `x_p` : pivot gauche en `(x_p, +b/2)`, pivot droit en `(x_p, −b/2)`, avec `b` = entraxe des pivots (**≈ 50 à 55 cm**).
- Abscisses des paires : `x_AR = d_AR` (décalage du pivot arrière par rapport à l'essieu arrière, signé, positif vers l'avant) et `x_AV = W + d_AV` (`W` = empattement essieu AV/AR, **≈ 3,7 à 4,0 m**, `d_AV` = décalage du pivot avant par rapport à l'essieu avant).
- Angle de bras `θ_i` : angle entre le bras et l'axe x, **positif vers l'intérieur** (vers l'axe de la machine). Direction du bras `s = +1` (vers l'avant) ou `s = −1` (vers l'arrière), paramétrable (`arm_direction`).
  - Bras gauche : `u_G = (s·cosθ, −sinθ)` ; bras droit : `u_D = (s·cosθ, +sinθ)`.
- Longueur de bras `L` = **35 cm**. Les deux bras d'une paire **se croisent** au repos (X devant les pivots).

### 3.2 Estimateur v1 (point de contact effectif)
- Point de contact de chaque bras : `C_i = P_i + c·u_i`, avec `c` = distance de contact effective (paramètre, défaut `c = L`, à affiner).
- Centre du rang à la station de la paire : `y_row,p = (y_CG + y_CD) / 2`.
- Largeur apparente de l'objet : `w_p = |y_CG − y_CD|` (diagnostic : trop grande ou trop petite = contact douteux).

### 3.3 Estimateur v2 (apex / bissectrice), à comparer
Les deux bras forment un V autour de l'objet. Le centre d'un objet rond tangent aux deux bras est sur la **bissectrice** du V, **quel que soit son diamètre**. Le point de contact exact sur chaque bras est inconnu (objet de 1 à 30 cm), donc le v1 a une erreur dépendant du diamètre. Implémente v1 et v2 derrière la même interface (`RowEstimator`) et **chiffre dans le simulateur l'erreur de chaque estimateur en fonction du diamètre (1 → 30 cm), du décalage et du lacet**. Recommande-moi celui à garder. C'est le principal risque technique du projet.

### 3.4 Décalage et cap
- 1 paire valide : décalage latéral à la station de la paire uniquement, `e = −y_row,p` (positif = machine décalée à gauche du rang). **Cap : indisponible** (flag explicite, pas de valeur inventée).
- 2 paires valides : points `Q_AV = (x_AV, y_row,AV)` et `Q_AR = (x_AR, y_row,AR)`.
  - Direction du rang dans le repère machine : `α = atan2(y_row,AV − y_row,AR, x_AV − x_AR)`.
  - **Cap de la machine par rapport au rang : `ψ = −α`.**
  - Décalage latéral = distance signée entre un point de référence `(x_ref, 0)` et la droite `Q_AR–Q_AV`. Sortir pour 3 références : essieu arrière, centre d'empattement, essieu avant.
- Avec 3,8 m entre les paires, une erreur de ±1,5 cm par paire donne ≈ ±0,45° sur le cap : documente cet ordre de grandeur.

### 3.5 Détection de contact et validité
- Un bras est « en contact » si `|θ − θ_repos| > seuil_contact` (avec hystérésis). `θ_repos` vient de la calibration (§5).
- Paire valide = les deux bras en contact. Sinon : mesure invalide + raison (bras gauche libre, bras droit libre, capteur hors plage, tension aberrante).
- Une tension capteur hors [0,3 ; 4,7 V] = défaut capteur (câble coupé, court-circuit), distinct de « pas de contact ».
- Option phase 2 : mode dégradé à un seul bras en s'appuyant sur la dernière largeur valide, avec flag « estimé ».
- Les piquets créent des échelons rapides : filtre robuste (médiane glissante + lissage + limite de pente), paramétrable, avec valeurs brutes **et** filtrées exposées.

**Point à valider avec moi avant de coder** : « les palpeurs se recroisent de x cm ». Je veux dire que le recouvrement/croisement au repos vaut x cm. Dis-moi comment tu l'interprètes. De toute façon, la valeur réelle d'angle au repos vient de la calibration du neutre.

---

## 4. Arborescence proposée

```
palpeur-traxx/
├─ README.md                 # install, build, flash, utilisation
├─ platformio.ini            # envs : plc (cible), native (tests)
├─ src/
│  ├─ main.cpp               # init, boucle, tâches
│  ├─ config/               # paramètres + stockage NVS (Preferences)
│  ├─ acquisition/          # lecture ADC, sur-échantillonnage, volts → angle
│  ├─ calibration/          # procédure + stockage
│  ├─ geometry/             # RowEstimator v1/v2, calcul décalage/cap (PURE, testable en natif)
│  ├─ filters/              # médiane, lissage, limite de pente
│  ├─ web/                  # serveur HTTP + WebSocket + API REST
│  ├─ can/                  # interface CanBus + implémentation désactivée + stub
│  └─ sim/                  # source de données simulée (rang, piquets, feuillage)
├─ data/                    # assets web embarqués (LittleFS) : html/js/css, sans CDN
├─ test/                    # tests natifs (geometry, filters, calibration)
├─ tools/mock_server.py     # sert la page web + données simulées sur PC
└─ docs/
   ├─ geometrie.md          # schémas, formules, conventions de signe
   ├─ cablage.md            # bornier, fusible, masse, blindage, nomenclature
   ├─ can_protocol.md       # spec CAN (§7)
   ├─ traxx_can.dbc         # fichier DBC
   └─ ros2_bridge/          # squelette nœud ROS 2 (spec uniquement, non compilé)
```

Le module `geometry/` ne doit dépendre ni de l'Arduino ni de l'ESP32 : fonctions pures, testables sur PC.

---

## 5. Fonctionnalités

### 5.1 Acquisition
- Échantillonnage rapide en tâche dédiée (FreeRTOS), sur-échantillonnage et moyenne, publication des mesures à **50 Hz** par défaut (paramétrable). Page web rafraîchie à ~20 Hz.
- Conversion tension → angle : `θ = (V − V_neutre_cal) · gain_cal`, avec nominal `V = 0,5 + 4,0·(angle+90°)/180°`. Inversion de sens paramétrable par canal.
- Horodatage (millis) sur chaque mesure et compteur de trames.

### 5.2 Calibration (depuis la page de paramétrage, valeurs en NVS)
Le neutre n'est pas forcément à mi-course : on calibre.
1. **Neutre** : bras au repos sans contact → bouton « Capturer neutre » (moyenne sur ~2 s, avec contrôle de stabilité) par capteur.
2. **Butées** : amener chaque bras à fond dans chaque sens → capture min/max de tension, vérifie que la plage ≈ 0,5–4,5 V, alerte sinon.
3. **Centre latéral** : machine centrée dans un rang de référence → « Définir le centre » : enregistre le biais de décalage de bout en bout.
- Assistant pas à pas à l'écran, avec valeur live. Bouton « Réinitialiser » (retour aux valeurs nominales). Export/import de la config en JSON.

### 5.3 Page web (en français, mobile first, sans framework lourd)
- **Vue Live** : vue de dessus (canvas) avec la machine, ses essieux, les palpeurs et leurs angles, le rang, le décalage latéral et le cap ; gros chiffres lisibles (décalage en cm, cap en °) ; voyants de validité par paire/bras ; courbes courtes (30 s) du décalage et du cap.
- **Vue Capteurs** : tension brute, angle brut/filtré, état de contact, défaut, par capteur.
- **Vue Paramètres** : géométrie (§3.1, tableau ci-dessous), filtres, seuils, Wi-Fi, sélection des paires actives, estimateur v1/v2, mode simulation. Validation des plages, valeurs par défaut, bouton « Enregistrer » (pas d'effet avant validation).
- **Vue Calibration** : assistant §5.2.
- **Vue Diagnostic** : version firmware, uptime, taux de trames, état Wi-Fi/CAN, export CSV des dernières minutes (enregistrement côté navigateur, pas de dépendance à une carte SD).
- API REST JSON documentée (`/api/state`, `/api/config`, `/api/calibration/*`) + WebSocket `/ws` pour le flux live. Ce JSON est la même source que celle des trames CAN futures.

### 5.4 Wi-Fi
- Par défaut : **point d'accès** du PLC (`SSID: PALPEUR-TRAXX`, mot de passe WPA2 configurable), nom `palpeur.local` en mDNS. Mode client (STA) optionnel dans les paramètres. Pas de dépendance au réseau du TRAXX.
- La page de paramétrage ne doit pas être modifiable par n'importe quel client sans une confirmation explicite (code PIN simple ou bouton « Déverrouiller »).

### 5.5 Mode simulation
- Générateur de données réalistes : rang droit puis courbe légère, objets de 1 à 30 cm, piquets, feuillage (bruit + traînée), pertes de contact, défaut capteur, vitesse d'avance réglable.
- Activable dans le firmware (sans capteurs branchés) **et** via `tools/mock_server.py` pour développer la page web sur PC.

---

## 6. Phases et critères d'acceptation

**Phase 0 : cadrage.** Plan, schéma ASCII, vérification ADC1/ADC2, choix PlatformIO vs arduino-cli, questions. *Livrable : réponse validée par moi.*

**Phase 1 : cœur de calcul + simulation (sans matériel).**
- `geometry/`, `filters/`, `calibration/` avec tests natifs : cas nominal, décalages ±, lacets ±, 1 paire / 2 paires, bras perdu, défaut capteur, diamètres 1/10/30 cm.
- Étude de sensibilité v1 vs v2 (tableau + courbes dans `docs/geometrie.md`).
- *Critère : `pio test -e native` vert ; recommandation d'estimateur argumentée.*

**Phase 2 : firmware + page web.**
- Acquisition réelle, calibration, page web complète, config en NVS, Wi-Fi AP, mode simulation.
- *Critères : build OK pour la cible ; page web fonctionnelle en mode simulation sur le PLC et via `mock_server.py` ; calibration assistée utilisable ; rafraîchissement live fluide (≥ 20 Hz) sur un téléphone ; la config survit à un redémarrage.*

**Phase 3 : préparation CAN (sans dépendre du matériel CAN).**
- Interface `CanBus`, implémentation désactivée par défaut, spec + DBC, encodage/décodage testés en natif, page diagnostic CAN.
- *Critère : les trames encodées à partir d'un état simulé se décodent à l'identique (test aller-retour).*

**Phase 4 : rédaction uniquement.** Squelette de nœud ROS 2 côté ordinateur TRAXX (SocketCAN → topic `/palpeur/row_pose`), dans `docs/ros2_bridge/`, non compilé.

**Livrables finaux** : code, tests, README (flash, première mise en service pas à pas), `docs/cablage.md` (bornier, fusible 0,5 A, diode TVS conseillée, masse commune, blindage), `docs/geometrie.md`.

---

## 7. Interface CAN future (à concevoir dès maintenant, proposition à valider)

- CAN 2.0A, 11 bits, **500 kbit/s** par défaut (paramétrable), émission à 50 Hz par défaut.
- Trames proposées (entiers signés little-endian, facteurs fixes, pas de float) :
  - `0x3A0 POSE` : décalage ref. (int16, 0,1 mm), cap ψ (int16, 0,01°), flags de validité (uint8), compteur (uint8), + décalage avant/arrière selon place restante.
  - `0x3A1 ARMS_FRONT` et `0x3A2 ARMS_REAR` : angles des 2 bras (int16, 0,01°), état contact, défaut.
  - `0x3A3 STATUS` : mode (réel/simulation), paires actives, estimateur, qualité.
  - `0x3A4 HEARTBEAT` : compteur + version firmware.
- Écris la spec dans `docs/can_protocol.md` + un fichier DBC cohérent. Dis-moi si les IDs entrent en conflit avec ceux déjà utilisés sur le bus du TRAXX (je te dirai lesquels sont occupés).
- Les données doivent être enregistrables dans un rosbag : message ROS 2 de type pose du rang (décalage, cap, covariances/qualité, validité) avec horodatage côté ordinateur.

---

## 8. Paramètres de configuration (valeurs par défaut)

| Paramètre | Défaut | Remarque |
|---|---|---|
| `pivot_spacing_m` (b) | 0,525 | 0,50–0,55 selon montage |
| `arm_length_m` (L) | 0,35 | |
| `contact_length_m` (c) | = L | à affiner |
| `arm_direction` | forward | forward / backward |
| `wheelbase_m` (W) | 3,85 | 3,7–4,0 |
| `front_pair_offset_m` (d_AV) | 0,0 | signé, par rapport à l'essieu avant |
| `rear_pair_offset_m` (d_AR) | 0,0 | signé, par rapport à l'essieu arrière |
| `front_pair_enabled` / `rear_pair_enabled` | true / false | 1 paire pour les essais |
| `row_object_diameter_m` (info) | 0,01–0,30 | sert au simulateur |
| `track_width_m` (voie), `inter_row_m` | non définis | optionnels, affichage uniquement |
| par capteur : `v_neutral`, `v_min`, `v_max`, `invert` | cf. calibration | |
| `contact_threshold_deg` / hystérésis | 3° / 1° | |
| `median_window`, `smoothing`, `max_slew_deg_s` | 5 / paramétrable / paramétrable | |
| `publish_rate_hz` / `ui_rate_hz` | 50 / 20 | |
| `estimator` | v1 (puis selon phase 1) | v1 / v2 |
| `wifi_mode`, `ssid`, `password`, `channel` | AP, PALPEUR-TRAXX | |
| `can_enabled`, `can_bitrate` | false, 500000 | |

---

## 9. Points ouverts : à me demander, ne pas trancher seul

1. Tension du bus TRAXX (12 / 24 / 48 V) et confirmation de l'alim PLC.
2. Version exacte du PLC (0-10 V ou 4-20 mA ; carte CAN présente ?).
3. Interprétation de « se recroisent de x cm » et position exacte des pivots sur la machine (devant/derrière les essieux, côté de croisement).
4. Quelle paire (avant ou arrière) pour les premiers essais.
5. IDs CAN déjà occupés sur le TRAXX, bitrate réel.
6. Voie du TRAXX et largeur d'inter-rang (affichage).
