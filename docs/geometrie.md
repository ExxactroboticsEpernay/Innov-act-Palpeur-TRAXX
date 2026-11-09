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

`d_AR` et `d_AV` sont signés par rapport aux essieux correspondants. Valeurs retenues pour le cadrage : `d_AR = -0,40 m` et `d_AV = +0,40 m`. Avec `W = 3,85 m` provisoire, la distance entre paires vaut `4,65 m`.

```text
                         x vers l'avant
       arrière  <------------------------------------>  avant

   paire AR             essieu AR        essieu AV             paire AV
   x = -0,40 m             x = 0           x = W                x = W+0,40 m
      pivots                  O                                  pivots
```

La voie communiquée est de 1,16 m. L'entraxe des pivots `b` est un paramètre distinct; sa valeur provisoire est 0,525 m. La voie ne doit donc pas être substituée à `b`.

## Angles et points de contact

`θ` est l'angle mesuré du bras par rapport à l'axe `x`, positif vers l'intérieur de la machine. `s = +1` pour les bras orientés vers l'avant et `s = -1` pour ceux orientés vers l'arrière.

```text
u_G = (s cos(θ_G), -sin(θ_G))
u_D = (s cos(θ_D), +sin(θ_D))
C_i = P_i + c u_i
```

`c` est la distance effective entre le pivot et le point de contact. Valeur initiale : `c = L = 0,35 m`, à vérifier mécaniquement. Le neutre angulaire est issu de la calibration; le croisement estimé à environ 15 cm ne sert pas à calculer cet angle.

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

Avec une séparation provisoire de 4,65 m et des erreurs de position opposées de 1,5 cm par paire, l'erreur différentielle maximale est 3 cm : `atan(0,03 / 4,65) ≈ 0,37°`. Ce calcul est un ordre de grandeur géométrique, pas une garantie de précision.

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
| `pivot_spacing_m` | 0,525 m | À mesurer sur le montage |
| `arm_length_m` | 0,35 m | Valeur nominale |
| `contact_length_m` | 0,35 m | Hypothèse initiale, à affiner |
| `arm_direction` | avant | À vérifier sur le montage |
| `wheelbase_m` | 3,85 m | Provisoire, plage attendue 3,7–4,0 m |
| `front_pair_offset_m` | +0,40 m | Choix communiqué |
| `rear_pair_offset_m` | -0,40 m | Position choisie pour une éventuelle deuxième paire; non utilisée dans le POC |
| `track_width_m` | 1,16 m | Valeur communiquée |
| `contact_threshold_deg` | 3° | Valeur initiale à régler |
| `contact_hysteresis_deg` | 1° | Valeur initiale à régler |
| Fréquence calcul / IHM | 50 Hz / 20 Hz | Cibles initiales |