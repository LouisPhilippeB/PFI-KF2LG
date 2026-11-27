"""Pilotage du châssis, mesure du lacet et surveillance du passage devant le robot."""

from contextlib import ExitStack
import math
import multiprocessing
import threading
import time

from capteur_lidar import Lidar
from moteur import Moteur
import param
from robot import Robot


PERIODE_IMU = 0.01
AGE_MAX_IMU = 0.25
DUREE_BIAIS = 1.0
AGE_MAX_LIDAR = 0.50
DELAI_FERMETURE_LIDAR = 6.0


def creer_robot():
    from gpiozero import DigitalOutputDevice, PWMOutputDevice

    with ExitStack() as sorties:
        def moteur(pwm, avant, arriere):
            return Moteur(
                sorties.enter_context(PWMOutputDevice(pwm)),
                sorties.enter_context(DigitalOutputDevice(avant)),
                sorties.enter_context(DigitalOutputDevice(arriere)),
            )

        gauche = moteur(param.MOTEUR_GAUCHE_PWM, param.MOTEUR_GAUCHE_IN2, param.MOTEUR_GAUCHE_IN1)
        droit = moteur(param.MOTEUR_DROIT_PWM, param.MOTEUR_DROIT_IN1, param.MOTEUR_DROIT_IN2)
        robot = Robot(gauche, droit)
        sorties.pop_all()
        return robot


def analyser_passage(points, avant, profondeur, largeur, distance_max):
    angles_avant = set()
    for distance, angle in points:
        if not (math.isfinite(angle) and 0 < distance <= distance_max):
            continue
        ecart = math.remainder(angle - avant, math.tau)
        longitudinal = distance * math.cos(ecart)
        lateral = distance * math.sin(ecart)
        if 0 < longitudinal <= profondeur and abs(lateral) <= largeur / 2:
            return False, "obstacle"
        if abs(ecart) <= math.radians(14):
            angles_avant.add(math.degrees(ecart))
    if not angles_avant:
        return None, "aucun retour frontal valide"
    bas, haut = min(angles_avant), max(angles_avant)
    if bas < -3 and haut > 3 and any(bas < a < haut and abs(a) <= 4 for a in angles_avant):
        return True, "libre"
    return None, "couverture frontale incomplète"


def passage_libre(points, avant, profondeur, largeur, distance_max):
    return analyser_passage(points, avant, profondeur, largeur, distance_max)[0]


def _acquerir_lidar(canal, fin, port, modele, avant, profondeur, largeur):
    lidar = Lidar(port, modele)
    try:
        lidar.demarrer()
        echecs = 0
        while not fin.is_set():
            if not lidar.est_actif():
                raise RuntimeError("Le lidar a cessé de tourner.")
            debut = time.monotonic()
            try:
                points = lidar.obtenir_donnees()
            except RuntimeError:
                echecs += 1
                if echecs >= 3:
                    raise
                canal.send((debut, None, "échec de lecture du scan", None))
                fin.wait(1 / lidar.MODELES[modele]["frequence"])
                continue
            echecs = 0
            libre, motif = analyser_passage(points, avant, profondeur, largeur, lidar.MODELES[modele]["distance_max"])
            canal.send((debut, libre, motif, None))
    except Exception as erreur:
        canal.send((time.monotonic(), None, "défaut lidar", f"Lidar : {erreur}"))
    finally:
        try:
            lidar.arreter()
        finally:
            canal.close()


class Capteurs:

    def __init__(self, port_lidar="/dev/ttyUSB0", modele_lidar="x4", avant_lidar=180.0,
                 distance_obstacle=0.50, sens_gyro=1, largeur_robot=0.24):
        modele = modele_lidar.lower()
        if not port_lidar or modele not in Lidar.MODELES:
            raise ValueError("Préciser le port série et le modèle de lidar (x2 ou x4).")
        if not math.isfinite(avant_lidar) or sens_gyro not in (-1, 1):
            raise ValueError("L'angle avant doit être fini et le sens du gyro doit valoir -1 ou +1.")
        if not all(math.isfinite(v) and v > 0 for v in (distance_obstacle, largeur_robot)):
            raise ValueError("La profondeur de détection et la largeur doivent être positives.")
        self._options_lidar = (port_lidar, modele, math.radians(avant_lidar), distance_obstacle, largeur_robot)
        self._sens_gyro = sens_gyro
        self._fin = threading.Event()
        self._verrou = threading.Lock()
        self._cap = self._instant_cap = None
        self._erreur_imu = self._erreur_lidar = None
        self._instant_scan = self._libre = None
        self.etat_lidar = "en attente du premier scan"
        self._imu = self._lidar = self._canal = self._fin_lidar = None

    def demarrer(self):
        if self._fin.is_set() or self._imu is not None:
            raise RuntimeError("Les capteurs ont déjà été démarrés ou fermés.")
        contexte = multiprocessing.get_context("spawn")
        self._fin_lidar = contexte.Event()
        self._canal, emetteur = contexte.Pipe(duplex=False)
        processus = contexte.Process(target=_acquerir_lidar, daemon=True, args=(emetteur, self._fin_lidar, *self._options_lidar))
        try:
            processus.start()
            self._lidar = processus
            self._imu = threading.Thread(target=self._acquerir_imu, name="gyroscope", daemon=True)
            self._imu.start()
        except BaseException:
            self.arreter()
            raise
        finally:
            emetteur.close()

    def _acquerir_imu(self):
        try:
            from icm20948 import ICM20948
            from smbus2 import SMBus

            with SMBus(1) as bus:
                capteur = ICM20948(i2c_bus=bus)
                capteur.set_gyro_sample_rate(200)
                capteur.set_gyro_full_scale(500)
                capteur.set_gyro_low_pass(enabled=True, mode=4)
                self._integrer_lacet(capteur)
        except Exception as erreur:
            with self._verrou:
                self._cap = None
                self._erreur_imu = f"IMU : {erreur}"

    def _integrer_lacet(self, capteur):
        debut_repos = instant_precedent = None
        somme = 0.0
        nombre = 0
        biais = None
        angle = vitesse_precedente = 0.0
        while not self._fin.is_set():
            debut = time.monotonic()
            gx = float(capteur.read_accelerometer_gyro_data()[3])
            instant = time.monotonic()
            if not math.isfinite(gx):
                raise ValueError("Lecture gyroscopique non finie.")
            interruption = instant_precedent is not None and instant - instant_precedent > AGE_MAX_IMU
            if biais is None:
                if debut_repos is None or interruption:
                    debut_repos, somme, nombre = instant, 0.0, 0
                somme += gx
                nombre += 1
                if instant - debut_repos >= DUREE_BIAIS and nombre >= 20:
                    biais = somme / nombre
                    vitesse_precedente = (gx - biais) * self._sens_gyro
            else:
                if interruption:
                    raise RuntimeError("Échantillonnage interrompu, le cap doit être réinitialisé au départ.")
                vitesse = (gx - biais) * self._sens_gyro
                angle = (angle + (vitesse_precedente + vitesse) * (instant - instant_precedent) / 2) % 360.0
                vitesse_precedente = vitesse
            if biais is not None:
                with self._verrou:
                    self._cap, self._instant_cap = angle, instant
            instant_precedent = instant
            self._fin.wait(max(0.0, PERIODE_IMU - (time.monotonic() - debut)))

    def cap(self):
        with self._verrou:
            if self._fin.is_set() or self._instant_cap is None or time.monotonic() - self._instant_cap > AGE_MAX_IMU:
                return None
            return self._cap

    def voie_libre(self):
        if self._fin.is_set() or self._canal is None or self._erreur_lidar:
            return None
        try:
            while self._canal.poll():
                self._instant_scan, self._libre, self.etat_lidar, self._erreur_lidar = self._canal.recv()
                if self._erreur_lidar:
                    return None
        except (OSError, EOFError) as erreur:
            self._erreur_lidar = f"Connexion au lidar perdue ({type(erreur).__name__})."
            return None
        if self._instant_scan is None:
            return None
        if time.monotonic() - self._instant_scan > AGE_MAX_LIDAR:
            self.etat_lidar = "scan périmé (> 0,50 s)"
            return None
        return self._libre

    @property
    def erreur(self):
        with self._verrou:
            return self._erreur_imu or self._erreur_lidar

    def arreter(self):
        self._fin.set()
        if self._fin_lidar is not None:
            self._fin_lidar.set()
        if self._imu is not None and self._imu.ident is not None:
            self._imu.join(timeout=AGE_MAX_IMU)
        if self._lidar is not None:
            # Une lecture en cours, turnOff et disconnecting peuvent dépasser 1 s.
            self._lidar.join(timeout=DELAI_FERMETURE_LIDAR)
            if self._lidar.is_alive():
                self._lidar.kill()
                self._lidar.join(timeout=1.0)
            if not self._lidar.is_alive():
                self._lidar.close()
                self._lidar = None
        if self._canal is not None:
            self._canal.close()
            self._canal = None
