# Points à confirmer

Ce registre évite de transformer des hypothèses en caractéristiques matérielles. Les points marqués bloquants doivent être résolus avant le câblage réel ou l'activation du CAN.

| Sujet | État actuel | Conséquence / action requise |
|---|---|---|
| Tension du bus TRAXX | Inconnue | Mesurer tension nominale et maximale. Capteurs RTY180HVNAA prévus pour 10–30 V; vérifier aussi l'alimentation admissible du PLC. |
| Référence exacte du PLC TRAXX | ESP32 PLC 19R envisagé | Achat probable. Confirmer référence complète, révision, variante analogique, alimentation et options avant commande/raccordement. |
| Nombre d'entrées analogiques | À vérifier sur le PLC du TRAXX | Le projet joystick pour PLC 14 `013000010001` documente deux entrées analogiques utilisables, suffisantes pour le POC à une paire. Vérifier la référence et la fiche officielle. |
| Entrées du 19R | À vérifier avant achat | Confirmer quatre voies analogiques 0–10 V utilisables simultanément avec le Wi-Fi, leur brochage et si elles sont sur ADC1. Ne pas déduire ces propriétés du seul nom « 19R ». |
| Angle de repos des bras | 90° (perpendiculaires, 2 ressorts de rappel) — confirmé | Repoussés vers l'arrière en marche avant, libres vers l'avant en marche arrière. Réglable par paire (`frontRestAngle`, `rearRestAngle`). |
| Entraxe des pivots arrière | Supposé identique à l'avant | Le paramètre `pivotSpacing` est commun aux deux paires; le séparer si le montage arrière diffère. |
| Évolution à deux paires | Logiciel prêt (1 ou 2 paires au choix); matériel 19R candidat | Si les essais montrent qu'un cap est nécessaire, vérifier que le 19R commandé accepte quatre capteurs; sinon choisir une autre référence de PLC. Pas d'extension analogique prévue à ce stade. |
| Chaîne Arduino CLI pour 19R | À vérifier | Confirmer que le paquet Industrial Shields utilisé par le joystick inclut la cible exacte du 19R et ses options de partition OTA. |
| Calibration firmware | Routes implémentées, non testées sur matériel | Neutre stable sur 2 s, balayage des butées sur 10 s et biais de centrage sont persistés en NVS; tester les seuils de stabilité et la procédure sur le PLC/capteurs réels. |
| Brochage RTY180HVNAA | Non relevé dans ce dépôt | Utiliser le plan officiel Honeywell correspondant exactement au connecteur et à la variante N; ne pas déduire les couleurs de fils. |
| Empattement | 2,14 m (fiche constructeur vignes étroites) | Version vignes larges : même empattement, voie 1,40 m et jambage intérieur 0,85 m. |
| Encombrement palpeurs | b = 0,515 m dans un jambage de 0,59 m | Environ 3,7 cm de marge de chaque côté entre pivot et jambage : vérifier au montage que bras et capteurs ne touchent pas les jambes. |
| Entraxe des pivots | 0,515 m mesuré sur table | Revérifier une fois monté sur le TRAXX. La voie de 1,16 m n'est pas l'entraxe des pivots. |
| Longueur des bras | 0,395 m mesurée sur table | Dimension modifiable si l'étude montre un meilleur compromis. |
| Croisement mécanique | 28,0 cm mesuré sur table | 2L − b = 27,5 cm : bras perpendiculaires à la machine au repos. Pris en compte : angle de repos θ0 = 90° et sens appris à la calibration. Vérifier que le neutre de chaque capteur laisse au moins 60° de course vers l'arrière (affiché à l'étape « Sens »). |
| Claquement des ressorts | Anti-rebond logiciel : confirmation 40 ms, fenêtre 250 ms, contact lié au côté | Régler ces durées aux premiers essais (vitesse, raideur, espacement des ceps). Vérifier que les bras croisés ne se heurtent pas au retour; envisager un amortissement (butée caoutchouc). |
| Voie | 1,16 m communiquée | Confirmer les points de mesure si la valeur est utilisée dans l'affichage. |
| Bus CAN TRAXX | IDs et bitrate inconnus | Obtenir la liste d'IDs occupés, le bitrate et l'accord du propriétaire du bus. CAN reste désactivé. |
| Carte CAN sur le PLC | Inconnue | Confirmer matériel/transceiver avant de choisir une implémentation physique. |
| Réseau Wi-Fi | AP local retenu | Mot de passe configurable; pas d'authentification OTA additionnelle prévue pour le prototype R&D local. |
| CAN 0x3A0–0x3A5 | Proposition seulement | Vérifier les conflits et faire valider l'ajout `0x3A5` avant publication de la DBC comme contrat. |

## Décisions déjà communiquées

- Première paire utilisée : avant.
- POC limité à une paire (deux capteurs); le cap n'est pas mesuré.
- Une deuxième paire pourra être étudiée après les essais; l'ESP32 PLC 19R est le candidat d'achat probable pour la permettre.
- Pivots choisis : 40 cm devant l'essieu avant et 40 cm derrière l'essieu arrière.
- Voie communiquée : 116 cm.
- Palpeurs mesurés sur table : b = 0,515 m, L = 0,395 m, croisement 28 cm au repos; bras perpendiculaires, 2 ressorts de rappel, repoussés vers l'arrière en marche avant, libres vers l'avant en marche arrière.
- TRAXX vignes étroites (fiche constructeur) : empattement 2,14 m, voie 1,16 m, jambage intérieur 0,59 m, longueur 3,18 m.
- Indicateur prioritaire : la comparaison G/D des deux bras d'une paire.
- OTA par Wi-Fi, dans l'esprit du projet joystick.
- Chaîne de build basée sur Arduino CLI et le paquet Industrial Shields du projet joystick.
- Validité : paire exigeant deux contacts; cap indisponible dans le POC; toute estimation dégradée doit porter un indicateur explicite.