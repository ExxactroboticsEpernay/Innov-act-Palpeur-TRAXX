# Câblage et sécurité électrique

> **Document de préparation uniquement. Ne pas câbler sur la base de suppositions.** La tension du bus TRAXX, la référence exacte du PLC, son option analogique et le brochage du connecteur Honeywell doivent être vérifiés avant raccordement.

## Schéma fonctionnel prévu

```text
Bus TRAXX (+V) ---- protection/fusible ----+---- PLC (alimentation conforme à sa fiche technique)
                                           +---- capteurs Honeywell (uniquement si +V est dans leur plage)

0 V TRAXX -------------------------------+---- GND PLC
                                           +---- GND capteurs

Sortie analogique capteur 1 -------------------- entrée analogique validée
Sortie analogique capteur 2 -------------------- entrée analogique validée

Blindage câble --------------------------------- reprise côté PLC uniquement
```

Ce schéma ne définit ni les broches du connecteur, ni les couleurs de fils. Ils doivent provenir de la fiche officielle du RTY180HVNAA et de la documentation de la variante exacte du PLC.

## Capteurs

Le modèle prévu est Honeywell RTY180HVNAA, version HV, sortie nominale 0,5–4,5 V non ratiométrique, alimentation 10–30 Vdc, courant maximal annoncé d'environ 32 mA par capteur. Ces données viennent du cadrage et doivent être confirmées sur la documentation officielle correspondant à la référence commandée.

- Ne pas alimenter le capteur en 5 V.
- Ne pas connecter le bus TRAXX au capteur avant d'avoir mesuré sa tension maximale et confirmé qu'elle reste dans la plage 10–30 V.
- Si le bus dépasse 30 V, une alimentation DC/DC adaptée sera nécessaire. La tension d'entrée admissible du PLC doit être vérifiée séparément; le DC/DC devra convenir aux deux équipements.
- La masse des capteurs et celle du PLC doivent être communes.
- Reprendre le blindage côté PLC uniquement, sauf indication contraire des documents officiels.
- Le fusible de branche proposé dans le cadrage est de 0,5 A. Sa sélection finale doit tenir compte du câblage, du courant disponible et des protections amont.
- Une diode TVS est conseillée dans le cadrage; sa référence et son dimensionnement restent à définir à partir du bus mesuré et des composants retenus.

## Entrées analogiques : point à résoudre

Le projet joystick documente, pour son PLC ESP32 PLC 14 `013000010001` en version 0–10 V, deux entrées analogiques utilisées (`I0.7` et `I0.8`), sur ADC1 et compatibles Wi-Fi. Ces deux voies suffisent au POC. L'ESP32 PLC 19R est le candidat probable pour le matériel du projet et une éventuelle deuxième paire; vérifier dans sa documentation officielle la référence exacte, la variante d'entrées analogiques, le nombre de voies et leur compatibilité ADC1/Wi-Fi. Ne pas transposer au 19R le brochage du PLC 14.

La paire avant du POC demande deux voies et correspond au nombre documenté pour le PLC du joystick. Aucune extension analogique n'est prévue pour ce POC. Si une deuxième paire devient nécessaire, valider que la variante du 19R retenue dispose bien de quatre voies adaptées; sinon, choisir une autre référence de PLC. Ne pas appliquer directement une tension 0–10 V à une entrée d'un ADS1115 ou à un GPIO ESP32 sans circuit d'adaptation dont la plage et la protection sont établies.

La sortie nominale du capteur (0,5–4,5 V) est dans la plage d'une entrée 0–10 V, mais la compatibilité électrique effective, l'impédance, la résolution et les seuils de défaut doivent être confirmés sur la documentation de la carte exacte. Wi-Fi actif : les voies utilisées doivent être sur ADC1 ou sur un convertisseur externe approprié.

## CAN futur

Le CAN est hors du câblage de la phase d'essai. La présence d'une carte CAN, le connecteur, l'isolation, le bitrate, la terminaison, la masse de référence et les IDs occupés sur le bus TRAXX sont inconnus. Le logiciel CAN restera désactivé par défaut.

Ne pas connecter l'interface au bus de la machine avant validation des IDs et du bitrate par le responsable du bus. La proposition de protocole dans [can_protocol.md](can_protocol.md) ne constitue pas une autorisation d'émission.

## Contrôles avant mise sous tension

1. Lire les références exactes du PLC et des capteurs.
2. Mesurer la tension du bus TRAXX, y compris ses variations maximales.
3. Vérifier la tension admissible et les borniers d'alimentation du PLC dans sa fiche officielle.
4. Valider le brochage du connecteur capteur avec le dessin officiel de la référence RTY180HVNAA.
5. Vérifier que les entrées analogiques sont bien la version 0–10 V et relever leur affectation officielle.
6. Vérifier la continuité de la masse commune et l'absence de court-circuit entre alimentation, signal et masse.
7. Tester chaque capteur et chaque entrée séparément, sans connexion au bus CAN du TRAXX.