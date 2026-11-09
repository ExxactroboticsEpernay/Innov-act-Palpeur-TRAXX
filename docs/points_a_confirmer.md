# Points à confirmer

Ce registre évite de transformer des hypothèses en caractéristiques matérielles. Les points marqués bloquants doivent être résolus avant le câblage réel ou l'activation du CAN.

| Sujet | État actuel | Conséquence / action requise |
|---|---|---|
| Tension du bus TRAXX | Inconnue | Mesurer tension nominale et maximale. Capteurs RTY180HVNAA prévus pour 10–30 V; vérifier aussi l'alimentation admissible du PLC. |
| Référence exacte du PLC TRAXX | ESP32 PLC 19R envisagé | Achat probable. Confirmer référence complète, révision, variante analogique, alimentation et options avant commande/raccordement. |
| Nombre d'entrées analogiques | À vérifier sur le PLC du TRAXX | Le projet joystick pour PLC 14 `013000010001` documente deux entrées analogiques utilisables, suffisantes pour le POC à une paire. Vérifier la référence et la fiche officielle. |
| Entrées du 19R | À vérifier avant achat | Confirmer quatre voies analogiques 0–10 V utilisables simultanément avec le Wi-Fi, leur brochage et si elles sont sur ADC1. Ne pas déduire ces propriétés du seul nom « 19R ». |
| Évolution à deux paires | Hors périmètre du POC; 19R candidat | Si les essais montrent qu'un cap est nécessaire, vérifier que le 19R commandé accepte quatre capteurs; sinon choisir une autre référence de PLC. Pas d'extension analogique prévue à ce stade. |
| Chaîne Arduino CLI pour 19R | À vérifier | Confirmer que le paquet Industrial Shields utilisé par le joystick inclut la cible exacte du 19R et ses options de partition OTA. |
| Calibration firmware | Routes implémentées, non testées sur matériel | Neutre stable sur 2 s, balayage des butées sur 10 s et biais de centrage sont persistés en NVS; tester les seuils de stabilité et la procédure sur le PLC/capteurs réels. |
| Brochage RTY180HVNAA | Non relevé dans ce dépôt | Utiliser le plan officiel Honeywell correspondant exactement au connecteur et à la variante N; ne pas déduire les couleurs de fils. |
| Empattement | 3,85 m provisoire | Mesurer sur le TRAXX; la distance entre paires calculée à 4,65 m changera avec cette mesure. |
| Entraxe des pivots | 0,525 m provisoire | Mesurer après montage. La voie de 1,16 m n'est pas l'entraxe des pivots. |
| Croisement mécanique | Environ 15 cm, estimation utilisateur | Préciser comment la mesure est prise. L'angle de repos vient de la calibration. |
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
- Environ 15 cm de croisement au repos, à confirmer.
- OTA par Wi-Fi, dans l'esprit du projet joystick.
- Chaîne de build basée sur Arduino CLI et le paquet Industrial Shields du projet joystick.
- Validité : paire exigeant deux contacts; cap indisponible dans le POC; toute estimation dégradée doit porter un indicateur explicite.