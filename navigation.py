"""Parcours rectangulaire en mètres dans le repère horizontal du tag Qorvo."""

import math
from collections import deque
from statistics import median


def ecart_cap(cible, cap):
    return (cible - cap + 180.0) % 360.0 - 180.0


def position_mediane(points):
    return median(p[0] for p in points), median(p[1] for p in points)


class NavigationRectangle:
    def __init__(self, robot, longueur=2.0, largeur=1.2, tolerance=0.15, age_gps=1.0):
        if not all(math.isfinite(v) and v > 0 for v in (longueur, largeur, tolerance)):
            raise ValueError("Les dimensions et la tolérance doivent être positives et finies.")
        self.amorce = min(0.75, longueur / 2)
        if self.amorce < max(0.35, 3 * tolerance) or largeur < 4 * tolerance:
            raise ValueError("Rectangle trop petit pour mesurer le cap et distinguer ses coins.")
        if not math.isfinite(age_gps) or age_gps <= 0:
            raise ValueError("L'âge maximal d'une mesure GPS doit être positif et fini.")
        self.robot = robot
        self.longueur = longueur
        self.largeur = largeur
        self.tolerance = tolerance
        self.age_gps = age_gps
        self.etat = "DÉPART"
        self.attente = ""
        self.sommets = ()
        self.cote = 0
        self._points = deque(maxlen=3)
        self._fixes_repos = deque(maxlen=3)
        self._dernier_fix = -math.inf
        self._precedent = None
        self._libre_depuis = None
        self._temps_cote = 0.0
        self._temps_etat = 0.0
        self._repere_avance = None
        self._temps_repere = 0.0
        self._impulsion_fin = None
        self._repos_fin = 0.0
        self._stable_depuis = None

    @property
    def termine(self):
        return self.etat == "TERMINÉ"

    def _changer_etat(self, etat):
        self.etat = etat
        self._temps_etat = 0.0
        self._repere_avance = None
        self._fixes_repos.clear()
        self._impulsion_fin = self._stable_depuis = None
        self._repos_fin = 0.0
        self.robot.arreter()

    def _position_au_repos(self, nouveau, point):
        if self._temps_etat < 0.25 or not nouveau:
            return None
        self._fixes_repos.append(point)
        if len(self._fixes_repos) == 3:
            centre = position_mediane(self._fixes_repos)
            if all(math.dist(centre, p) <= self.tolerance for p in self._fixes_repos):
                return centre
        return None

    def _definir_rectangle(self, point, cap):
        dx, dy = point[0] - self.depart[0], point[1] - self.depart[1]
        distance = math.hypot(dx, dy)
        if distance < 0.85 * self.amorce:
            self._changer_etat("AMORÇAGE")
            return
        ux, uy = dx / distance, dy / distance
        cap_radio = math.degrees(math.atan2(dy, dx))
        cap_moyen_imu = self.cap_depart + ecart_cap(cap, self.cap_depart) / 2
        self._radio_moins_imu = cap_radio - cap_moyen_imu
        x, y = self.depart
        p1 = x + self.longueur * ux, y + self.longueur * uy
        p3 = x - self.largeur * uy, y + self.largeur * ux
        p2 = p1[0] - self.largeur * uy, p1[1] + self.largeur * ux
        self.sommets = (self.depart, p1, p2, p3)
        self._orienter(point)

    def _cap_de_route(self, point):
        debut = self.sommets[self.cote]
        fin = self.sommets[(self.cote + 1) % 4]
        longueur = math.dist(debut, fin)
        ux, uy = (fin[0] - debut[0]) / longueur, (fin[1] - debut[1]) / longueur
        projection = (point[0] - debut[0]) * ux + (point[1] - debut[1]) * uy
        visee = min(longueur, max(0.0, projection) + max(0.45, 3 * self.tolerance))
        dx = debut[0] + visee * ux - point[0]
        dy = debut[1] + visee * uy - point[1]
        return math.degrees(math.atan2(dy, dx)) - self._radio_moins_imu

    def _orienter(self, point):
        self._cap_cible = self._cap_de_route(point)
        self._changer_etat("ROTATION")

    def _rotation(self, maintenant, cap):
        erreur = ecart_cap(self._cap_cible, cap)
        if abs(erreur) <= 3.0:
            self.robot.arreter()
            self._impulsion_fin = None
            if self._stable_depuis is None:
                self._stable_depuis = maintenant
            if maintenant - self._stable_depuis >= 0.20:
                self._changer_etat("AVANCE")
            return
        self._stable_depuis = None
        if self._impulsion_fin is not None and maintenant >= self._impulsion_fin:
            self._impulsion_fin = None
            self._repos_fin = maintenant + 0.12
        if maintenant < self._repos_fin:
            self.robot.arreter()
            return
        if abs(erreur) < 20 and self._impulsion_fin is None:
            self._impulsion_fin = maintenant + max(0.025, min(0.12, abs(erreur) / 150))
        if erreur > 0:
            self.robot.pivoter_gauche()
        else:
            self.robot.pivoter_droite()

    def _avancer(self, erreur, point):
        if self._repere_avance is None or math.dist(point, self._repere_avance) >= 0.10:
            self._repere_avance = point
            self._temps_repere = self._temps_etat
        elif self._temps_etat - self._temps_repere > 6.0:
            raise RuntimeError("Avance commandée, mais le GPS ne mesure pas de déplacement de 10 cm en 6 s.")
        self.robot.avancer_corrige(max(-0.05, min(0.05, erreur * 0.004)))

    def _etape(self, maintenant, nouveau, point, filtre, cap):
        if self.etat == "DÉPART":
            depart = self._position_au_repos(nouveau, point)
            if depart is not None:
                self.depart, self.cap_depart = depart, cap
                self._changer_etat("AMORÇAGE")
        elif self.etat == "AMORÇAGE":
            erreur = ecart_cap(self.cap_depart, cap)
            if abs(erreur) > 20:
                raise RuntimeError("Le robot a dévié pendant la mesure de son cap initial.")
            if math.dist(self.depart, filtre) >= self.amorce:
                self._changer_etat("MESURE DU CAP")
            else:
                self._avancer(erreur, filtre)
        elif self.etat == "MESURE DU CAP":
            repos = self._position_au_repos(nouveau, point)
            if repos is not None:
                self._definir_rectangle(repos, cap)
        elif self.etat == "ROTATION":
            self._rotation(maintenant, cap)
        elif self.etat == "AVANCE":
            cible = self.sommets[(self.cote + 1) % 4]
            if math.dist(filtre, cible) <= self.tolerance:
                self._changer_etat("CONFIRMATION DU COIN")
            else:
                erreur = ecart_cap(self._cap_de_route(filtre), cap)
                if abs(erreur) > 20:
                    self._orienter(filtre)
                else:
                    self._avancer(erreur, filtre)
        elif self.etat == "CONFIRMATION DU COIN":
            repos = self._position_au_repos(nouveau, point)
            if repos is None:
                return
            if math.dist(repos, self.sommets[(self.cote + 1) % 4]) > self.tolerance:
                self._orienter(repos)
                return
            self.cote += 1
            self._temps_cote = 0.0
            if self.cote == 4:
                self._changer_etat("TERMINÉ")
            else:
                self._orienter(repos)

    def actualiser(self, maintenant, mesure, cap, voie_libre):
        if self.termine:
            return
        dt = 0 if self._precedent is None else maintenant - self._precedent
        self._precedent = maintenant
        radio_valide = (mesure is not None and all(map(math.isfinite, mesure))
                        and 0 <= maintenant - mesure[0] <= self.age_gps
                        and mesure[0] >= self._dernier_fix)
        if not radio_valide:
            self.attente = "position Qorvo indisponible"
        elif cap is None or not math.isfinite(cap):
            self.attente = "gyroscope indisponible ou en étalonnage"
        elif voie_libre is not True:
            self.attente = "obstacle devant le robot" if voie_libre is False else "scan lidar indisponible"
        else:
            self.attente = ""
        if self.attente:
            self.robot.arreter()
            self._libre_depuis = self._stable_depuis = self._impulsion_fin = None
            self._fixes_repos.clear()
            self._points.clear()
            return

        horodatage, x, y = mesure
        nouveau = horodatage > self._dernier_fix
        if nouveau:
            self._dernier_fix = horodatage
            self._points.append((x, y))
        if self._libre_depuis is None:
            self._libre_depuis = maintenant
        if maintenant - self._libre_depuis < 0.20 or len(self._points) < 2:
            self.robot.arreter()
            return

        self._temps_cote += dt
        self._temps_etat += dt
        longueur_cote = self.largeur if self.cote % 2 else self.longueur
        if (self._temps_cote > 25 + longueur_cote / 0.04
                or (self.etat == "ROTATION" and self._temps_etat > 20)):
            self.robot.arreter()
            raise RuntimeError(f"Le robot ne termine pas l’étape « {self.etat} » du côté {self.cote + 1}.")
        try:
            self._etape(maintenant, nouveau, (x, y), position_mediane(self._points), cap)
        except Exception:
            self.robot.arreter()
            raise
