# Géométrie et estimateurs

Ce document fixe les conventions de signe et les équations de référence. Les valeurs d'installation sont configurables; les estimations de contact restent à comparer au simulateur avant de choisir l'estimateur de production.

## Périmètre du POC

Le POC démarre avec la paire avant uniquement, sans cap. Le logiciel implémente aussi le cas à deux paires ci-dessous, activable par le paramètre `activePairs`. Côté matériel, l'ESP32 PLC 19R est le candidat probable, sous réserve de confirmer qu'une variante adaptée prend en charge les quatre capteurs.

Le biais de centrage (calibration « centre ») est enregistré **par paire** et retranché à `y_row,p` avant le calcul de la pose. Avec deux paires, la capture se fait machine centrée **et alignée** dans un rang de référence : elle remet donc aussi le cap à zéro.

## Repère machine

- Origine `O` : centre de l'essieu arrière.
- Axe `x` : vers l'avant.
- Axe `y` : vers la gauche.
- Lacet `ψ` : positif dans le sens antihoraire, vu de dessus.
- Essieu arrière : `x = 0`.
- Essieu avant : `x = W`, où `W` est l'empattement.

Pour une paire `p`, les pivots sont symétriques autour de l'axe longitudinal :

```text
P_G = (x_p, +b/2)       pivot gauche
P_D = (x_p, -b/2)       pivot droit
```

Les abscisses sont :

```text
x_AR = d_AR
x_AV = W + d_AV
```

`d_AR` et `d_AV` sont signés par rapport aux essieux correspondants. Valeurs retenues pour le cadrage : `d_AR = -0,40 m` et `d_AV = +0,40 m`. Avec `W = 2,14 m` (fiche constructeur TRAXX vignes étroites), la distance entre paires vaut `2,94 m`.

```text
                         x vers l'avant
       arrière  <------------------------------------>  avant

   paire AR             essieu AR        essieu AV             paire AV
   x = -0,40 m             x = 0           x = W                x = W+0,40 m
      pivots                  O                                  pivots
```

La voie communiquée est de 1,16 m. L'entraxe des pivots `b` est un paramètre distinct; elle vaut 0,515 m (mesurée sur table). La voie ne doit donc pas être substituée à `b`. Largeur intérieure du jambage : 0,59 m, soit environ 3,7 cm de marge par pivot.

## Angles et points de contact

`θ` est l'angle mesuré du bras par rapport à l'axe `x`, positif vers l'intérieur de la machine. `s = +1` pour les bras orientés vers l'avant et `s = -1` pour ceux orientés vers l'arrière.

```text
u_G = (s cos(θ_G), -sin(θ_G))
u_D = (s cos(θ_D), +sin(θ_D))
C_i = P_i + c u_i
```

`c` est la distance effective entre le pivot et le point de contact. Valeur initiale : `c = L = 0,395 m`; en pratique le contact glisse le long du bras. Le neutre (bras au repos, perpendiculaires) est issu de la calibration; le croisement de 28 cm en découle (2L − b = 27,5 cm).

## Estimateur v1 : points de contact effectifs

Pour les contacts gauche et droit calculés par la formule précédente :

```text
y_row,p = (y_CG + y_CD) / 2
w_p     = |y_CG - y_CD|
```

`y_row,p` est l'ordonnée estimée du centre du rang à la station de la paire. `w_p` est une largeur apparente de diagnostic; ce n'est pas nécessairement le diamètre de l'objet touché.

Cet estimateur dépend de la distance effective `c` et de la position réelle du contact sur chaque bras. Sa sensibilité au diamètre des objets doit être quantifiée par simulation pour les diamètres de 1 à 30 cm.

## Estimateur v2 : bissectrice

Un cercle tangent aux deux bras a son centre situé sur une bissectrice des droites support des bras. Cette propriété définit une direction géométrique, mais ne suffit pas à elle seule à localiser le centre : sa position le long de la bissectrice dépend notamment du rayon et du choix de la bissectrice pertinente.

Le v2 reste un estimateur candidat, pas une solution validée. Dans le code actuel, `BisectorEstimator` construit l'intersection des droites support des bras, forme la bissectrice opposée aux directions des bras et l'intersecte avec la station `x = x_p`. Ce choix fournit une ordonnée de rang à la station de la paire sans supposer le diamètre; il ne prétend pas retrouver le centre exact d'un objet dans les géométries asymétriques. Les deux estimateurs implémentent la même interface `RowEstimator`.

### Simulateur et étude de sensibilité

`src/sim/geometry_simulation.cpp` calcule les deux tangentes depuis les pivots vers un cercle de diamètre connu. `tools/geometry_study.cpp` balaie les diamètres de 1 à 30 cm, les décalages de -20 à +20 cm par pas de 10 cm et les lacets de -5° à +5° par pas de 5°. Il écrit les erreurs v1/v2 en CSV sur la sortie standard :

```powershell
.\build\palpeur_geometry_study.exe > geometry-study.csv
```

Ce modèle suppose des bras rigides, des tangences parfaites et un objet circulaire isolé. Il ne modélise ni feuillage, ni forme irrégulière des pieds/piquets, ni jeu mécanique, ni bruit capteur. Les résultats servent à comparer les estimateurs, pas à annoncer une précision terrain. La recommandation finale attend l'exécution du binaire et des essais mécaniques.

## Décalage latéral et cap

Convention de sortie : `e = -y_row`. Ainsi, `e > 0` signifie que la machine est décalée à gauche du rang.

### Une paire valide

La mesure de décalage est disponible uniquement à la station de la paire :

```text
e_p = -y_row,p
```

Le cap est indisponible. Il doit être représenté par un état de validité explicite, jamais par un zéro implicite.

### Évolution éventuelle : deux paires valides

```text
Q_AR = (x_AR, y_row,AR)
Q_AV = (x_AV, y_row,AV)
α = atan2(y_row,AV - y_row,AR, x_AV - x_AR)
ψ = -α
```

Pour chaque abscisse de référence `x_ref`, l'ordonnée de la droite du rang est :

```text
y_row(x_ref) = y_row,AR
             + (y_row,AV - y_row,AR) * (x_ref - x_AR) / (x_AV - x_AR)
e(x_ref)     = -y_row(x_ref)
```

Dans une évolution à deux paires, les trois références sont l'essieu arrière (`x_ref = 0`), le centre d'empattement (`x_ref = W/2`) et l'essieu avant (`x_ref = W`). Si une paire est invalide, le cap est indisponible.

### Ordre de grandeur de l'erreur angulaire

Avec une séparation de 2,94 m et des erreurs de position opposées de 1,5 cm par paire, l’erreur différentielle maximale est 3 cm : `atan(0,03 / 2,94) ≈ 0,58°`. L’empattement court du TRAXX rend le cap plus sensible aux erreurs de chaque paire. Ce calcul est un ordre de grandeur géométrique, pas une garantie de précision.

## Montage mesuré : bras perpendiculaires au repos (à intégrer)

Mesures sur table : `b = 0,515 m`, `L = 0,395 m`, croisement au repos 28,0 cm. Comme `2L − b = 27,5 cm`, les bras sont **perpendiculaires à la machine au repos** (`θ_repos ≈ 90°`) et se croisent au milieu. En avançant, un cep repousse les deux bras vers l'arrière.

Ordres de grandeur (objet fin, contact en bout de bras) :

- Plage où les deux bras touchent : `|y| ≤ L − b/2 ≈ 13,7 cm` (plus large pour un gros objet).
- Rang centré : chaque bras est à environ 41° de l'axe x, soit environ 49° de déflexion depuis le repos.
- Sensibilité au centre : environ 1,9°/cm, soit environ 42 mV/cm sur le capteur.

### Angle de repos et balayage (implémenté)

- Le capteur donne un **balayage** `φ` relatif au neutre calibré (bras au repos), avec le gain nominal de la fiche Honeywell (4,0 V / 180°, soit 22,2 mV/°). Les butées capturées ne servent plus au gain, seulement au contrôle de cohérence (au moins 1 V balayé).
- Le sens de chaque capteur est appris à l'étape de calibration « Sens » : bras tenus repoussés vers l'arrière, `φ > 0`. Les capteurs gauche et droit sont montés en miroir.
- Angle géométrique utilisé par les estimateurs, mesuré depuis l'axe `x`, positif vers l'intérieur : `θ = θ0 + φ`, avec `θ0` = angle de repos par paire (90° par défaut). `θ > 90°` : bras repoussé vers l'arrière (marche avant). `θ < 90°` : bras basculé vers l'avant (marche arrière). La même équation couvre les deux cas, avec `s = +1`.
- L'API publie `angle_raw_deg` / `angle_filtered_deg` = `φ` et `arm_angle_deg` = `θ`.

### Encore ouvert : point de contact glissant

Le cep glisse le long du bras pendant son passage, donc la distance de contact `c` varie. L'estimateur v1, qui suppose `c` fixe, est biaisé. L'estimateur v2 actuel prolonge la bissectrice jusqu'à la station `x = x_p`. Or le cep se trouve en arrière de la station, et la bissectrice n'est pas parallèle au rang quand la machine est décalée : ce prolongement introduit aussi un biais, de l'ordre de 2 cm pour 3 cm de décalage selon une estimation à la main. Une variante prenant le **sommet du V** (point de croisement des deux bras), proche du centre du cep, semble plus robuste. À comparer dans le simulateur avec un cep qui défile.

## Comparaison G/D : l'indicateur prioritaire

Pour une paire, l'écart `ΔGD = φG − φD` (balayages filtrés des bras gauche et droit) est la mesure la plus directe du centrage :

- **Zéro robuste** : machine centrée sur le cep, les deux bras sont repoussés de la même façon par symétrie, quels que soient le diamètre du cep et le point de contact. `ΔGD = 0` ne dépend donc ni de `c` ni de l'estimateur.
- **Signe** : `ΔGD > 0` = bras gauche plus repoussé = cep plus près du pivot gauche = **rang à gauche, machine décalée à droite** (décalage `e < 0`).
- **Amplitude** : le passage de `ΔGD` à des centimètres dépend du point de contact, donc de l'estimateur. C'est là que portent les incertitudes v1/v2.
- **Simultanéité** : `ΔGD` n'est publié que si les deux bras touchent en même temps (même cep, même échantillon). Sinon il vaut `null`.
- **Zéro calibré** : la capture « centre » enregistre aussi `ΔGD` machine centrée. Ce zéro compense les différences de montage entre gauche et droite.

Publié par paire dans l'API (`lr_diff_deg`), affiché en tête de l'état des paires et enregistré dans le CSV (`gd_avant_deg`, `gd_arriere_deg`).

## Claquement des ressorts

Quand le cep libère un bras, les deux ressorts le ramènent brutalement au repos. Il dépasse le repos (balayage négatif), puis rebondit autour. Les bras se croisent de 28 cm au repos, ce qui peut aussi les faire se heurter. Traitement dans `SensorChannel` :

- **Décision sur la médiane**, pas sur la valeur lissée : la libération coupe le contact tout de suite, sans traîner l'ancien angle. Lissage et limite de pente repartent de la valeur courante.
- **Contact lié au côté** : un contact pris vers l'arrière tombe dès que le bras repasse de l'autre côté du repos. Le dépassement n'est jamais pris pour un contact en marche arrière.
- **Confirmation** : un contact doit durer au moins 30 ms (environ 8 échantillons à 250 Hz). Un choc isolé ou un pic est ignoré.
- **Fenêtre anti-rebond** : pendant 80 ms après une libération, aucun nouveau contact. Le capteur publie `rebound = true`, affiché « rebond ». Cette durée est plafonnée par le cas cep + piquet (voir ci-dessous).
- **Limite de pente à 1 000°/s** : à 6 km/h, le bras balaie jusqu'à environ 370°/s à l'impact. La limite ne sert plus qu'à écrêter les aberrations.

Ces durées sont à régler sur les premiers essais : vitesse d'avance, raideur des ressorts, espacement des ceps. Pour cela, enregistrer un passage à 250 Hz et mesurer la durée réelle du retour et des rebonds. Côté mécanique, un amortissement (butée caoutchouc, frein visqueux) réduirait les chocs sur les capteurs et les bras.

## Rang réel : cadence et mesure par cep

Données terrain : un cep tous les 1 m, de **3 à 25 cm** de diamètre ; un piquet tous les 5 à 10 m, entre deux ceps ; TRAXX jusqu'à **6 km/h** (1,67 m/s).

| Cas (6 km/h, montage b = 0,515 m, L = 0,395 m) | Ordre de grandeur |
|---|---|
| Balayage au relâchement, cep fin centré | ~49° |
| Balayage au relâchement, cep de 25 cm centré | ~70° |
| Balayage maxi, cep de 25 cm décalé de 10 cm | ~85–90° (bras presque parallèle à la machine) |
| Trajet en double contact | ~0,30 m (cep fin) à ~0,5 m (cep de 25 cm) |
| Durée du double contact | ~180 ms (cep fin) à ~300 ms |
| Bras libre entre deux ceps espacés de 1 m | ~0,4 s |
| Bras libre entre un cep et un piquet espacés de 0,5 m | ~0,09 s pour un cep fin, ~0,05 s pour 10 cm, moins de 30 ms au-delà de ~15 cm (les passages peuvent se confondre). Si la fenêtre anti-rebond couvre ce trou, le piquet est vu un peu plus tard, pas perdu. |

Conséquences dans le code :

- **Acquisition à 250 Hz**, environ 45 échantillons par passage. La publication (page, futur CAN) garde sa propre cadence.
- **Mesure par cep** (`VineEventTracker`) : chaque double contact est résumé à sa fin (moyennes de G−D et de `y_row`, durée, balayage maxi). La mesure est conservée jusqu'au passage suivant. L'API publie `last_vine` avec `age_s` et `count`.
- **Pose à partir des derniers ceps** : le décalage, et le cap avec 2 paires, utilisent le dernier passage de chaque paire s'il date de moins de 5 s (marge pour rouler lentement). Les paires AV et AR touchent rarement un cep au même instant : sans cette tenue, le cap ne serait presque jamais disponible.
- **Fusion cep + piquet** : si les deux passages se confondent, la mesure est une moyenne des deux objets. Le zéro G/D reste juste ; seule l'amplitude est mélangée.

**Calage mécanique du capteur.** Avec des ceps jusqu'à 25 cm, un bras doit pouvoir être repoussé d'environ 90° vers l'arrière, et basculer d'environ 45° vers l'avant en marche arrière. Il faut donc caler chaque capteur pour que son neutre soit entre 1,5 et 2,5 V si la tension monte vers l'arrière, ou entre 2,5 et 3,5 V dans le montage miroir. L'étape de calibration « Sens » affiche la course disponible vers l'arrière et alerte en dessous de 90°.

## Détection et qualité

- Le contact d'un bras est détecté par écart à son neutre calibré, avec seuil et hystérésis configurables.
- Une paire n'est valide que si ses deux bras sont en contact et que leurs mesures capteurs sont valides.
- Une tension hors de `[0,3 V; 4,7 V]` est un défaut capteur, distinct d'une absence de contact.
- Les raisons d'invalidité sont exposées séparément : bras gauche libre, bras droit libre, défaut capteur, tension aberrante, calibration invalide ou paire désactivée.
- Les valeurs brutes et filtrées sont conservées. Médiane glissante, lissage et limite de pente sont paramétrables.
- Le mode à un seul bras avec largeur précédente est désactivé pour la phase 1. S'il est ajouté ultérieurement, le résultat sera explicitement estimé et limité dans le temps.

## Valeurs initiales

| Paramètre | Valeur de cadrage | État |
|---|---:|---|
| `pivot_spacing_m` | 0,515 m | Mesuré sur table (2026-11-09) |
| `arm_length_m` | 0,395 m | Mesuré sur table (2026-11-09) |
| `contact_length_m` | 0,395 m | = L par défaut; le contact glisse le long du bras, voir ci-dessous |
| `rest_angle_deg` (θ0) | 90° par paire | Bras perpendiculaires au repos, rappelés par 2 ressorts (confirmé) |
| `wheelbase_m` | 2,14 m | Fiche constructeur TRAXX vignes étroites |
| `front_pair_offset_m` | +0,40 m | Choix communiqué |
| `rear_pair_offset_m` | -0,40 m | Position choisie pour une éventuelle deuxième paire; non utilisée dans le POC |
| `track_width_m` | 1,16 m | Valeur communiquée |
| `contact_threshold_deg` | 3° | Valeur initiale à régler |
| `contact_hysteresis_deg` | 1° | Valeur initiale à régler |
| Fréquence calcul / IHM | 50 Hz / 20 Hz | Cibles initiales |