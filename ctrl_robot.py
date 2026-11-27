#!/usr/bin/env python3
"""Faire parcourir un rectangle au robot dans le repère du GPS intérieur."""

import argparse
from contextlib import ExitStack
import sys
import time

from gps import GPS
from materiel import Capteurs, creer_robot
from navigation import NavigationRectangle


def surveiller(gps, capteurs, navigation=None):
    dernier_etat = None
    prochain_affichage = 0.0
    coins_affiches = False
    while True:
        mesure = gps.mesure()
        cap = capteurs.cap()
        libre = capteurs.voie_libre()
        erreur = gps.erreur or capteurs.erreur
        if erreur:
            raise RuntimeError(erreur)
        maintenant = time.monotonic()
        if navigation is None:
            if maintenant >= prochain_affichage:
                xy = "en attente" if mesure is None else f"({mesure[1]:.2f}, {mesure[2]:.2f}) m"
                angle = "en attente" if cap is None else f"{cap:+.1f}°"
                passage = capteurs.etat_lidar
                print(f"GPS {xy} | cap relatif {angle} | lidar {passage}", flush=True)
                prochain_affichage = maintenant + 0.5
        else:
            navigation.actualiser(maintenant, mesure, cap, libre)
            if navigation.sommets and not coins_affiches:
                coins = " → ".join(f"({x:.2f}, {y:.2f})" for x, y in navigation.sommets)
                print(f"Sommets du rectangle en mètres — {coins}", flush=True)
                coins_affiches = True
            detail = navigation.attente or navigation.etat.lower()
            if detail == "scan lidar indisponible":
                detail = f"lidar — {capteurs.etat_lidar}"
            etat = (navigation.etat, detail, navigation.cote)
            if etat != dernier_etat:
                if navigation.termine:
                    print("Rectangle terminé. Robot arrêté au point de départ.", flush=True)
                    return
                angle = ""
                if navigation.etat == "ROTATION" and cap is not None:
                    angle = f" (cap {round(cap, 1) % 360:.1f} deg, cible {round(navigation.cap_cible, 1) % 360:.1f} deg)"
                print(f"Côté {navigation.cote + 1}/4 — {detail}{angle}", flush=True)
                dernier_etat = etat
        time.sleep(0.02)


def main(arguments=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--longueur", type=float, default=2.0, help="Premier côté en mètres.")
    p.add_argument("--largeur", type=float, default=1.2, help="Deuxième côté, à gauche, en mètres.")
    p.add_argument("--tolerance", type=float, default=0.15, help="Rayon d'arrivée à un coin en mètres.")
    p.add_argument("--port-gps", default="/dev/ttyACM0")
    p.add_argument("--qualite-min", type=int, default=50, help="Qualité Qorvo minimale sur 100.")
    p.add_argument("--age-gps", type=float, default=1.0, help="Âge maximal d'une position en secondes.")
    p.add_argument("--port-lidar", default="/dev/ttyUSB0")
    p.add_argument("--modele-lidar", choices=("x2", "x4"), default="x4")
    p.add_argument("--avant-lidar", type=float, default=180, help="Angle du devant dans le repère lidar en degrés.")
    p.add_argument("--distance-obstacle", type=float, default=0.50, help="Profondeur surveillée depuis le lidar en mètres.")
    p.add_argument("--largeur-robot", type=float, default=0.24, help="Largeur du passage surveillé en mètres.")
    p.add_argument("--sens-gyro", type=int, choices=(-1, 1), default=1, help="Signe de l'axe X pour qu'un pivot gauche augmente le cap.")
    p.add_argument("--diagnostic", action="store_true", help="Afficher les trois capteurs, moteurs désactivés.")
    options = p.parse_args(arguments)
    try:
        gps = GPS(options.port_gps, options.qualite_min, options.age_gps)
        capteurs = Capteurs(options.port_lidar, options.modele_lidar, options.avant_lidar,
                            options.distance_obstacle, options.sens_gyro, options.largeur_robot)
        with ExitStack() as sorties:
            sorties.callback(gps.arreter)
            sorties.callback(capteurs.arreter)
            navigation = None
            if not options.diagnostic:
                robot = creer_robot()
                sorties.callback(robot.fermer)
                navigation = NavigationRectangle(robot, options.longueur, options.largeur, options.tolerance, options.age_gps)
            print("Le robot doit rester immobile pendant l’étalonnage du gyroscope.", flush=True)
            gps.demarrer()
            capteurs.demarrer()
            surveiller(gps, capteurs, navigation)
    except KeyboardInterrupt:
        print("Arrêt demandé.")
    except (OSError, RuntimeError, ValueError, ImportError) as erreur:
        print(f"Démarrage ou navigation interrompus — {erreur}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
