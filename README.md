# Palpeurs de guidage du TRAXX

Projet de mesure en lecture seule du décalage latéral et du cap du TRAXX dans un rang de vigne, à partir de palpeurs mécaniques et d'un PLC Industrial Shields ESP32.

> **État du dépôt : cœur natif et maquette en développement.** Les estimateurs, filtres, conversion capteur, étude de géométrie, page web locale, serveur mock et routes de calibration firmware sont présents. Les tests natifs et les routes matérielles ne sont pas encore exécutés localement; l'acquisition réelle et le build sur PLC 19R attendent le FQBN et son brochage officiel.

## Objectifs du POC

- Mesurer et afficher le décalage latéral du TRAXX au niveau de la paire avant.
- Ne pas estimer le cap avec une seule paire : le cap reste explicitement indisponible.
- Afficher les mesures et diagnostics sur une page web locale, utilisable hors connexion Internet.
- Préparer une interface CAN désactivée par défaut pour une future intégration aux rosbag du TRAXX.
- Comparer par simulation les estimateurs géométriques v1 et v2 avant de retenir celui qui convient.

Le PLC ne commande aucun actionneur. Ses sorties ne sont pas utilisées.

Le POC utilise une seule paire avant (deux capteurs). Une deuxième paire pourra être étudiée si les essais montrent qu'une mesure de cap est nécessaire. Cette évolution pourra justifier le remplacement du PLC; elle n'est pas une exigence de la première version.

L'**ESP32 PLC 19R** est maintenant le candidat d'achat probable pour garder cette évolution ouverte. La référence complète et la variante analogique restent à confirmer; le nom du modèle seul ne garantit pas quatre entrées 0–10 V utilisables avec le Wi-Fi.

## Configuration retenue pour le cadrage

| Élément | Valeur de travail | État |
|---|---:|---|
| Première paire | Avant | Choix confirmé |
| Pivot avant | 0,40 m devant l'essieu avant | Choix confirmé |
| Pivot arrière | 0,40 m derrière l'essieu arrière | Choix confirmé |
| Empattement | 3,85 m | Valeur provisoire à mesurer |
| Distance entre paires | 4,65 m avec l'empattement provisoire | Calculée pour une éventuelle évolution, non utilisée dans le POC |
| Voie de la machine | 1,16 m | Valeur communiquée, définition des points de mesure à confirmer |
| Croisement des bras au repos | Environ 15 cm | Estimation à confirmer; ne remplace pas la calibration du neutre |
| PLC candidat | Industrial Shields ESP32 PLC 19R | Achat probable; référence et variante à confirmer |
| Acquisition initiale | Deux capteurs pour une paire | Compatible avec les deux voies analogiques documentées pour le PLC du projet joystick |
| Évolution éventuelle | Deuxième paire | À décider après essais; le 19R est le candidat, sous réserve de vérifier ses voies analogiques |
| Build firmware | Arduino CLI et paquet de cartes Industrial Shields | Retenu pour reprendre la chaîne éprouvée du projet joystick |
| Mise à jour firmware | Téléversement OTA depuis l'IHM Wi-Fi | Retenue; première installation par USB |
| CAN | Désactivé par défaut; IDs proposés dans la documentation | Bitrate et IDs à valider sur le bus TRAXX |

### Entrées analogiques

Le README du projet joystick documente, pour son PLC ESP32 PLC 14 référence `013000010001` en version 0–10 V, deux voies analogiques utilisables (`I0.7` et `I0.8`), reliées à ADC1 et utilisables avec le Wi-Fi. Ces deux voies suffisent au POC à une paire. Le candidat ESP32 PLC 19R pourrait permettre l'évolution à deux paires, mais il faut confirmer sur la documentation officielle sa variante 0–10 V, le nombre de voies réellement disponibles avec le Wi-Fi et l'option CAN éventuelle. Si une deuxième paire devient nécessaire, le 19R sera évalué avant d'envisager une extension analogique externe.

Ne jamais raccorder directement un signal au PLC sur la seule base de ce README. Voir [câblage et sécurité électrique](docs/cablage.md) et [points à confirmer](docs/points_a_confirmer.md).

## Documentation

- [Maquette web interactive](data/index.html) (simulation locale, sans connexion au PLC)
- [Géométrie et estimateurs](docs/geometrie.md)
- [Câblage et sécurité électrique](docs/cablage.md)
- [Mise à jour OTA](docs/mise_a_jour.md)
- [Proposition de protocole CAN](docs/can_protocol.md)
- [DBC proposé](docs/traxx_can.dbc)
- [Points à confirmer](docs/points_a_confirmer.md)
- [Mapping matériel PLC 19R](docs/board_19r_map.md)
- [Spécification de la passerelle ROS 2](docs/ros2_bridge/README.md)
- [Prompt de cadrage initial](20261109_PalpeurTRAXX_Prompt_ClaudeCode.md)

## Chaîne de développement prévue

La chaîne retenue pour la cible est Arduino CLI avec le paquet de cartes Industrial Shields, comme dans le projet joystick. La compilation exacte dépend toutefois de la référence du PLC et de ses options d'usine. Le cœur géométrique, les filtres et la calibration devront rester indépendants d'Arduino afin d'être testables sur PC.

Les tests natifs utilisent CMake et un compilateur C++17, sans bibliothèque tierce :

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

L'étude de sensibilité génère un CSV sur la sortie standard :

```powershell
.\build\palpeur_geometry_study.exe > geometry-study.csv
```

Le mock web local utilise Python 3.10+ et la bibliothèque standard uniquement :

```powershell
py -3 tools/mock_server.py
```

Ouvrir ensuite `http://127.0.0.1:8765/`. La page reste également ouvrable directement via [data/index.html](data/index.html); dans ce mode, elle utilise son simulateur intégré et conserve les paramètres dans le navigateur.

Le sketch firmware se prépare dans un dossier de staging avec :

```powershell
arduino-cli lib install ArduinoJson@6.21.5
.\tools\build_arduino.ps1 -Fqbn "<fqbn-confirme-du-19R>"
```

Le build cible reste bloqué tant que le FQBN exact et le mapping des entrées du 19R ne sont pas confirmés. Le firmware compile en simulation quand la cible Arduino est disponible; `-RealInputs` est volontairement bloqué sans `board_19r_map.h`. Les outils de transfert LittleFS dépendent du paquet Industrial Shields retenu et restent à confirmer.

## Phases prévues

1. Cœur de calcul, filtres, calibration et simulation; tests natifs et étude v1/v2.
2. Acquisition PLC, configuration persistante, page web embarquée, Wi-Fi et OTA après validation du 19R.
3. Encodage/décodage CAN natifs; émission désactivée par défaut et IDs à valider.
4. Spécification de la passerelle ROS 2, sans nœud compilé dans ce dépôt.

Les éléments qui dépendent d'une mesure ou d'une validation matérielle restent explicitement marqués comme provisoires dans les documents.