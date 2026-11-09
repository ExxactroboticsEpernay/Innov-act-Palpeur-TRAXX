# Mise à jour du firmware par Wi-Fi

La mise à jour OTA depuis un navigateur est retenue, en reprenant le principe utilisé par le projet joystick Industrial Shields. Elle sera implémentée avec le firmware; elle n'est pas encore disponible dans ce dépôt.

## Fonctionnement prévu

1. La première installation du firmware se fait par USB avec Arduino CLI et le paquet officiel de cartes Industrial Shields.
2. Le firmware démarre son point d'accès Wi-Fi et sert l'IHM embarquée, sans CDN ni ressource Internet.
3. Depuis la vue diagnostic/maintenance, l'opérateur sélectionne un binaire d'application `.bin` et confirme son envoi.
4. Le PLC vérifie l'image et l'écrit dans l'emplacement OTA inactif. En cas d'image incomplète ou invalide, l'image courante doit rester démarrable.
5. Après succès, le PLC redémarre; l'IHM vérifie que la nouvelle version est revenue.

Le sketch expose `POST /api/ota` en `multipart/form-data` avec l'API `Update` du cœur ESP32. L'implémentation doit être compilée et testée sur le FQBN Industrial Shields exact avant toute utilisation. Le serveur mock de PC refuse cette route (HTTP 501).

## Pré-requis

- Un schéma de partitions avec deux emplacements d'application OTA. Le schéma exact doit être vérifié pour la carte et la taille du firmware.
- Les assets web embarqués dans le firmware ou LittleFS; aucune dépendance au réseau Internet à l'exécution.
- Une procédure de récupération USB documentée si le firmware ne redémarre pas.
- Un contrôle d'intégrité du binaire et un contrôle de taille avant de commencer l'écriture.

## Accès en contexte de prototypage

Pour le POC de R&D, aucune authentification OTA additionnelle ni code PIN ne sont requis : l'OTA est accessible depuis le réseau local du PLC, comme sur le joystick. Le point d'accès reste protégé par son mot de passe Wi-Fi configurable et n'est pas relié à Internet. L'IHM demande une confirmation avant l'envoi. Ce choix correspond au prototype et pourra être revu avant un déploiement opérationnel.

Les contrôles d'intégrité du binaire, de taille et de conservation de l'ancien firmware restent nécessaires au bon fonctionnement, indépendamment de l'authentification.

## À tester avant livraison

- Première installation USB avec partitions OTA.
- Mise à jour par Wi-Fi depuis un téléphone et un ordinateur.
- Refus d'un fichier incorrect, trop gros, tronqué ou d'un autre type de partition.
- Coupure Wi-Fi et redémarrage pendant la réception : l'ancien firmware doit rester récupérable.
- Conservation des calibrations et paramètres stockés en NVS.
- Retour à l'IHM après redémarrage et affichage de la version courante.