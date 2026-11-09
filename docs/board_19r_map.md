# Mapping matériel du PLC 19R

L'Industrial Shields ESP32 PLC 19R est le candidat matériel probable, mais sa référence complète et sa variante analogique ne sont pas encore confirmées. Aucun numéro de GPIO ni canal analogique n'est codé en dur dans le firmware.

## Garde de compilation

Le firmware est compilé en simulation par défaut. Pour demander l'acquisition réelle, le build définit `PALPEUR_SIMULATION=0`; il échoue alors volontairement tant que `src/firmware/board_19r_map.h` n'existe pas.

Ce fichier, à créer après consultation de la documentation officielle correspondant à la référence achetée, doit fournir :

```cpp
void palpeur19rBegin();
// channel : 0 avant gauche, 1 avant droit, 2 arrière gauche, 3 arrière droit.
float palpeur19rReadVoltage(int channel);
```

La fonction de lecture retourne des volts côté entrée analogique après conversion effectuée par l'API officielle Industrial Shields. Elle doit retourner `NAN` pour une voie non câblée ou indisponible : la voie apparaît alors en défaut capteur, jamais comme une mesure. Le firmware ne lit que les voies des paires actives (paramètre `activePairs`). Ne pas copier les GPIO ou macros du PLC 14 du joystick sans preuve qu'ils sont identiques au 19R.

## Vérifications avant le mapping

1. Référence complète et révision matérielle du PLC.
2. Variante d'entrées 0–10 V, quatre voies disponibles simultanément et bornier exact.
3. Nom des entrées (`I0.x` ou autre) dans le paquet officiel de cartes.
4. Correspondance des voies avec ADC1/ADC2 et comportement Wi-Fi actif.
5. Conversion brute vers volts, résolution, impédance et seuils électriques admissibles.
6. FQBN Arduino CLI et options de partition contenant deux slots OTA.
7. Présence et référence du module CAN, si cette option est commandée.

Après validation, vérifier chaque voie au multimètre et avec une tension connue avant de brancher les capteurs. Le mode réel ne doit être activé qu'après ces contrôles.