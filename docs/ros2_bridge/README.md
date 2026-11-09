# Spécification de la passerelle ROS 2

Cette phase est documentaire uniquement. Aucun nœud ROS 2 n'est compilé dans le projet firmware.

## But

Recevoir les trames CAN du PLC via SocketCAN sur l'ordinateur du TRAXX, vérifier leur validité et publier une mesure du rang horodatée côté ordinateur afin qu'elle puisse être enregistrée dans un rosbag.

Le POC ne comporte qu'une paire avant. Il fournit un décalage latéral à la station de cette paire, mais pas de cap ni de pose complète de la droite du rang. Le cap et la pose orientée ne seront publiés que si une évolution à deux paires est décidée et implémentée.

## Interface proposée

- Entrée : interface SocketCAN, bitrate et IDs à confirmer avec le bus réel.
- Topic envisagé après validation CAN : `/palpeur/row_pose`.
- Message géométrique proposé : `geometry_msgs/msg/PoseWithCovarianceStamped`.
- `header.stamp` est l'heure de réception côté ordinateur; le PLC ne fournit pas une horloge synchronisée.
- `header.frame_id` est le repère machine convenu avec l'équipe ROS (proposition : `base_link`).
- Convention de pose : position du rang exprimée dans le repère machine; `position.y = y_row = -e`. Le lacet de la droite du rang dans ce repère est `α = -ψ`.
- Pour le POC, publier uniquement le décalage latéral à la station avant dans le champ documenté par le consommateur; ne pas écrire un cap nul comme s'il était mesuré.
- Les covariances doivent provenir d'une caractérisation expérimentale ou être marquées inconnues; ne pas inventer une précision.

`PoseWithCovarianceStamped` ne transporte pas à lui seul tous les états (paire valide, bras libres, mesure estimée, défaut capteur, mode simulation). Proposition : publier en parallèle un `diagnostic_msgs/msg/DiagnosticArray` sur `/diagnostics`, avec le statut, les causes d'invalidité et la qualité. Si les consommateurs ont besoin de champs typés stables, définir ultérieurement un message dédié dans un paquet ROS 2 versionné.

## Conversion CAN proposée

Le DBC décrit le décodage des trames, mais la table CAN est encore à valider. Le pont devra :

1. Rejeter les IDs, longueurs et compteurs incohérents.
2. Convertir `LateralOffset` et `Heading` avec les facteurs du DBC.
3. Utiliser les flags de `PALPEUR_POSE` pour déterminer les champs valides.
4. Pour le POC, garder le cap indisponible; une évolution à deux paires pourra publier le cap si `HeadingValid` est actif et les deux paires sont valides.
5. Ne décoder `PALPEUR_OFFSETS` que si l'évolution à deux paires et l'ID correspondant sont approuvés.
6. Déclarer la perte de données si les trames attendues deviennent périmées; le délai devra être déterminé avec la fréquence CAN retenue.

## Hors périmètre

- Pas de commande envoyée au PLC ou au TRAXX.
- Pas de contrôle d'actionneur.
- Pas d'horodatage synchronisé PLC tant qu'un mécanisme dédié n'est pas défini.
- Pas de package ROS 2 ou de nœud implémenté dans cette phase.