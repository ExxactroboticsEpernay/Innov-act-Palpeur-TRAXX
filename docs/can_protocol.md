# Proposition de protocole CAN

> **Proposition non validée.** Le CAN est désactivé par défaut. Les IDs, le bitrate, la présence du matériel CAN et l'allocation de ces IDs sur le bus TRAXX restent à confirmer. Ne pas connecter ni émettre sur le bus machine avec cette proposition sans validation explicite.

Le POC utilise une seule paire avant et ne fournit pas de cap. Les trames arrière et les trois références de décalage décrites plus bas sont réservées à une éventuelle évolution à deux paires; elles ne font pas partie du contrat du POC. Si le cap est requis, le choix du PLC sera réévalué.

## Paramètres proposés

- CAN 2.0A, identifiants standard 11 bits.
- Bitrate proposé : 500 kbit/s.
- Émission proposée : 50 Hz pour les mesures, heartbeat à 1 Hz.
- Payload : 8 octets maximum, entiers signés/non signés little-endian, sans flottants.
- Toutes les mesures sont invalides si le flag correspondant est absent; les champs invalides ne doivent pas être interprétés comme des zéros mesurés.
- Le compteur de trame est modulo 256.

## IDs proposés

| ID | Nom | Période proposée | Contenu |
|---:|---|---:|---|
| `0x3A0` | `PALPEUR_POSE` | 20 ms | Décalage de référence, cap, validités et compteur |
| `0x3A1` | `PALPEUR_ARMS_FRONT` | 20 ms | Angles et états des bras avant |
| `0x3A2` | `PALPEUR_ARMS_REAR` | 20 ms | Angles et états des bras arrière |
| `0x3A3` | `PALPEUR_STATUS` | 100 ms | Mode, paires actives, estimateur, qualité |
| `0x3A4` | `PALPEUR_HEARTBEAT` | 1 s | Compteur et version firmware |
| `0x3A5` | `PALPEUR_OFFSETS` | 20 ms | Décalages aux essieux arrière, milieu et avant |

`0x3A2` et `0x3A5` ne sont utiles que pour une future configuration à deux paires. Leur ajout doit être validé avec le propriétaire du bus; ils ne sont pas utilisés par le POC.

## Signaux

Les numéros de bits dans ce tableau suivent la convention DBC : bit 0 = bit de poids faible de l'octet 0; `@1` signifie little-endian. Les facteurs indiquent la résolution physique.

### `0x3A0 PALPEUR_POSE`

| Bits | Signal | Type / facteur | Unité | Description |
|---:|---|---|---|---|
| 0–15 | `LateralOffset` | `int16`, 0,1 | mm | Décalage à la référence choisie par la configuration |
| 16–31 | `Heading` | `int16`, 0,01 | deg | `ψ`, cap machine par rapport au rang |
| 32–39 | `ValidityFlags` | `uint8` | bitfield | Voir ci-dessous |
| 40–47 | `Counter` | `uint8` | — | Compteur modulo 256 |
| 48–63 | `Reserved` | `uint16` | — | Émis à zéro, réservé |

Bits `ValidityFlags` : bit 0 pose latérale valide; bit 1 cap valide (toujours à zéro pour le POC); bit 2 paire avant valide; bit 3 paire arrière valide (toujours à zéro pour le POC); bit 4 résultat estimé; bit 5 défaut capteur présent; bit 6 calibration valide; bit 7 mode simulation.

### `0x3A1` et `0x3A2` bras avant/arrière

`0x3A1` est la seule trame bras du POC. `0x3A2` est réservée à l'éventuelle paire arrière.

| Bits | Signal | Type / facteur | Unité | Description |
|---:|---|---|---|---|
| 0–15 | `AngleLeft` | `int16`, 0,01 | deg | Angle filtré bras gauche |
| 16–31 | `AngleRight` | `int16`, 0,01 | deg | Angle filtré bras droit |
| 32 | `LeftContact` | `uint1` | — | Contact gauche détecté |
| 33 | `RightContact` | `uint1` | — | Contact droit détecté |
| 34 | `PairValid` | `uint1` | — | Paire exploitable |
| 35 | `LeftFault` | `uint1` | — | Défaut capteur gauche |
| 36 | `RightFault` | `uint1` | — | Défaut capteur droit |
| 37 | `Estimated` | `uint1` | — | Valeur de paire estimée |
| 38–41 | `InvalidReason` | `uint4` | enum | 0 aucun; 1 gauche libre; 2 droite libre; 3 deux bras libres; 4 défaut capteur; 5 hors plage; 6 calibration invalide; 7 paire désactivée; 8–15 réservés |
| 42–47 | `ReservedFlags` | `uint6` | — | Émis à zéro |
| 48–55 | `Counter` | `uint8` | — | Compteur modulo 256 |
| 56–63 | `Reserved` | `uint8` | — | Émis à zéro |

Les angles bruts restent disponibles dans l'API web et le diagnostic; cette trame transporte les angles filtrés. Ce choix pourra être revu si le débit ou le diagnostic l'exige.

### `0x3A3 PALPEUR_STATUS`

| Bits | Signal | Type | Description |
|---:|---|---|---|
| 0 | `SimulationMode` | `uint1` | 1 en simulation, 0 en acquisition réelle |
| 1–2 | `EnabledPairs` | `uint2` | bit 0 avant; bit 1 arrière |
| 3–4 | `Estimator` | `uint2` | 0 aucun; 1 v1; 2 v2; 3 réservé |
| 5–6 | `QualityCode` | `uint2` | 0 invalide; 1 dégradé/estimé; 2 valide; 3 réservé |
| 7 | `CalibrationValid` | `uint1` | Calibration utilisable |
| 8–63 | `Reserved` | `uint56` | Émis à zéro |

### `0x3A4 PALPEUR_HEARTBEAT`

| Bits | Signal | Type | Description |
|---:|---|---|---|
| 0–7 | `Counter` | `uint8` | Compteur modulo 256 |
| 8–15 | `FirmwareMajor` | `uint8` | Version majeure |
| 16–23 | `FirmwareMinor` | `uint8` | Version mineure |
| 24–31 | `FirmwarePatch` | `uint8` | Correctif |
| 32–63 | `Reserved` | `uint32` | Émis à zéro |

### `0x3A5 PALPEUR_OFFSETS` (évolution à deux paires)

| Bits | Signal | Type / facteur | Unité | Description |
|---:|---|---|---|---|
| 0–15 | `OffsetRearAxle` | `int16`, 0,1 | mm | Décalage au niveau de l'essieu arrière |
| 16–31 | `OffsetWheelbaseCenter` | `int16`, 0,1 | mm | Décalage au centre de l'empattement |
| 32–47 | `OffsetFrontAxle` | `int16`, 0,1 | mm | Décalage au niveau de l'essieu avant |
| 48–55 | `Counter` | `uint8` | — | Compteur modulo 256 |
| 56–63 | `Reserved` | `uint8` | — | Émis à zéro |

## Validité, perte et démarrage

- CAN désactivé au démarrage tant que `can_enabled` n'est pas activé dans une configuration validée.
- Une paire désactivée ou invalide n'est pas présentée comme une mesure valide.
- Pour le POC à une paire avant, seul le décalage à la station avant peut être valide; le cap reste invalide.
- Les trames arrière et les décalages aux trois références ne sont activés que si une deuxième paire est décidée et si le PLC est adapté.
- Le heartbeat permet de détecter la présence du nœud; il ne vaut pas validation d'une mesure.
- Le compteur est indépendant pour chaque ID.
- Les valeurs hors plage du type entier doivent être saturées ou déclarées invalides; aucun wrap silencieux n'est permis.

## Validation requise avant intégration

1. Confirmer le matériel CAN du PLC et son transceiver.
2. Demander les IDs déjà utilisés et le bitrate réel du bus TRAXX.
3. Faire valider les IDs `0x3A0` à `0x3A5` et les périodes par le responsable du bus.
4. Valider le signal de référence latérale dans `0x3A0` et l'ajout proposé de `0x3A5`.
5. Tester en natif l'aller-retour encodeur/décodeur contre le DBC avant tout essai matériel.