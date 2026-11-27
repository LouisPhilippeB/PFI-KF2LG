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
        self.coins_mesures = []
        self.cote = 0
        self._points = deque(maxlen=3)
        self._fixes_repos = deque(maxlen=5)
        self._dernier_fix_repos = -math.inf
        self._dispersion = 0.0
        self._intervalles_gps = deque(maxlen=5)
        self._duree_amorce = 0.0
        self._dernier_fix = -math.inf
        self._precedent = None
        self._libre_depuis = None
        self._temps_cote = 0.0
        self._temps_etat = 0.0
        self._repere_avance = None
        self._temps_repere = 0.0
        self._impulsion_fin = None
        self._sens_impulsion = 0
        self._vitesse_pivot = 180.0
        self._repos_fin = 0.0
        self._stable_depuis = None
        self._apres_rotation = "AVANCE"

    @property
    def termine(self):
        return self.etat == "TERMINÉ"

    def _changer_etat(self, etat):
        self.etat = etat
        self._temps_etat = 0.0
        self._repere_avance = None
        self._fixes_repos.clear()
        self._dernier_fix_repos = -math.inf
        self._reference_vitesse = None
        self._fin_approche = None
        self._impulsion_fin = self._stable_depuis = None
        self._repos_fin = 0.0
        self.robot.arreter()

    def _position_au_repos(self, nouveau, point):
        if self._temps_etat < 0.25 or not nouveau or self._dernier_fix - self._dernier_fix_repos < 0.24:
            return None
        self._dernier_fix_repos = self._dernier_fix
        self._fixes_repos.append(point)
        if len(self._fixes_repos) == self._fixes_repos.maxlen:
            centre = position_mediane(self._fixes_repos)
            self._dispersion = median(math.dist(centre, p) for p in self._fixes_repos)
            if self._dispersion <= 1.5 * self.tolerance:
                return centre
            if not self.attente and self._temps_etat > max(7, 8 * median(self._intervalles_gps)):
                raise RuntimeError(f"Position GPS trop dispersée à l'arrêt ({self._dispersion:.2f} m).")
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
        self.coins_mesures = [self.depart]
        self._vitesse_avance = distance / max(0.01, self._duree_amorce)
        self._orienter()

    def _position_sur_cote(self, point):
        debut = self.sommets[self.cote]
        fin = self.sommets[(self.cote + 1) % 4]
        longueur = math.dist(debut, fin)
        ux, uy = (fin[0] - debut[0]) / longueur, (fin[1] - debut[1]) / longueur
        dx, dy = point[0] - debut[0], point[1] - debut[1]
        return longueur - (dx * ux + dy * uy), ux * dy - uy * dx

    def _cap_du_cote(self):
        debut = self.sommets[self.cote]
        fin = self.sommets[(self.cote + 1) % 4]
        direction = math.degrees(math.atan2(fin[1] - debut[1], fin[0] - debut[0]))
        return (direction - self._radio_moins_imu) % 360.0

    def _orienter(self):
        self.cap_cible = self._cap_du_cote()
        self._apres_rotation = "AVANCE"
        self._changer_etat("ROTATION")

    def _rayon_coin(self):
        if self.cote == 3:
            return self.tolerance
        return min(2 * self.tolerance, min(self.longueur, self.largeur) / 4)

    def _coin_atteint(self, restant, lateral):
        # Viser le bout du côté même avec un GPS précis. Accepter un passage
        # légèrement au-delà si la médiane reste dans le voisinage du sommet.
        avance_manquante = min(self.tolerance, max(self.tolerance / 3, self._dispersion))
        return restant <= avance_manquante and math.hypot(restant, lateral) <= self._rayon_coin()

    def _approcher_coin(self, repos, cap, restant, lateral):
        cible = self.sommets[(self.cote + 1) % 4]
        cap_cote = self._cap_du_cote()
        distance = math.hypot(restant, lateral)
        # Une seule trame a pu faire freiner très tôt. Reprendre la ligne
        # si les mesures à l'arrêt placent le coin au-delà de la zone de freinage.
        seuil_reprise = 2 * self.tolerance + self._vitesse_avance * (2 * median(self._intervalles_gps) + 0.18)
        if restant > seuil_reprise:
            self._orienter()
            return
        if restant > 0 and abs(lateral) <= self._rayon_coin() * 0.9:
            # Compléter la longueur du côté sans tourner vers un seul point bruité.
            self.cap_cible = cap_cote
            distance = restant
        else:
            direction = math.degrees(math.atan2(cible[1] - repos[1], cible[0] - repos[0]))
            self.cap_cible = (direction - self._radio_moins_imu) % 360
        self._duree_approche = max(0.06, min(0.35, 0.75 * distance / self._vitesse_avance))
        self._apres_rotation = "APPROCHE DU COIN"
        self._changer_etat("ROTATION" if abs(ecart_cap(self.cap_cible, cap)) > 8 else self._apres_rotation)

    def _rotation(self, maintenant, cap):
        erreur = ecart_cap(self.cap_cible, cap)
        if self._impulsion_fin is not None:
            if maintenant < self._impulsion_fin and erreur * self._sens_impulsion > 4:
                return
            self.robot.arreter()
            self._duree_impulsion = max(0.001, maintenant - self._debut_impulsion)
            self._impulsion_fin = None
            self._repos_fin = maintenant + 0.22
            return
        if maintenant < self._repos_fin:
            return
        if self._sens_impulsion:
            progres = self._sens_impulsion * ecart_cap(cap, self._cap_impulsion)
            self._sens_impulsion = 0
            if progres < -6:
                raise RuntimeError("Pivot mesuré en sens inverse. Vérifier le signe avec --diagnostic et --sens-gyro.")
            if progres > 1:
                self._vitesse_pivot = max(35, min(500, progres / self._duree_impulsion))
        if abs(erreur) <= 4:
            if self._stable_depuis is None:
                self._stable_depuis = maintenant
            if maintenant - self._stable_depuis >= 0.20:
                self._changer_etat(self._apres_rotation)
            return
        self._stable_depuis = None
        self._cap_impulsion = cap
        self._debut_impulsion = maintenant
        self._sens_impulsion = 1 if erreur > 0 else -1
        self._impulsion_fin = maintenant + max(0.020, min(0.25, 0.70 * abs(erreur) / self._vitesse_pivot))
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

    def _distance_freinage(self, maintenant, nouveau, restant):
        # Réestimer la vitesse sur une portion de ligne droite, entre deux
        # positions filtrées assez éloignées dans le temps pour limiter le bruit.
        if nouveau:
            if self._reference_vitesse is None:
                self._reference_vitesse = self._dernier_fix, restant
            instant, restant_avant = self._reference_vitesse
            duree = self._dernier_fix - instant
            if duree >= 0.8:
                vitesse = (restant_avant - restant) / duree
                if 0.4 * self._vitesse_avance <= vitesse <= 2.5 * self._vitesse_avance:
                    self._vitesse_avance = 0.6 * self._vitesse_avance + 0.4 * vitesse
                self._reference_vitesse = self._dernier_fix, restant
        # Retard de la médiane mobile, âge du dernier fix et roue libre.
        delai = median(self._intervalles_gps) + (maintenant - self._dernier_fix) + 0.18
        return self.tolerance + self._vitesse_avance * delai

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
                self._duree_amorce += self._temps_etat
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
            restant, lateral = self._position_sur_cote(filtre)
            freinage = self._distance_freinage(maintenant, nouveau, restant)
            if restant <= freinage or (nouveau and self._position_sur_cote(point)[0] <= 0):
                self._changer_etat("CONFIRMATION DU COIN")
            else:
                correction = math.degrees(math.atan2(-lateral, max(0.45, 3 * self.tolerance)))
                consigne = self.cap_cible + max(-18, min(18, correction))
                erreur = ecart_cap(consigne, cap)
                if abs(ecart_cap(self.cap_cible, cap)) > 35:
                    self._orienter()
                else:
                    self._avancer(erreur, filtre)
        elif self.etat == "APPROCHE DU COIN":
            # Une trame aberrante ne doit pas annuler chaque petite avance.
            if self._fin_approche is None:
                self._fin_approche = maintenant + self._duree_approche
            if maintenant >= self._fin_approche:
                self._changer_etat("CONFIRMATION DU COIN")
            else:
                self._avancer(ecart_cap(self.cap_cible, cap), filtre)
        elif self.etat == "CONFIRMATION DU COIN":
            repos = self._position_au_repos(nouveau, point)
            if repos is None:
                return
            restant, lateral = self._position_sur_cote(repos)
            if not self._coin_atteint(restant, lateral):
                self._approcher_coin(repos, cap, restant, lateral)
                return
            self.coins_mesures.append(repos)
            self.cote += 1
            self._temps_cote = 0.0
            if self.cote == 4:
                self._changer_etat("TERMINÉ")
            else:
                self._orienter()

    def actualiser(self, maintenant, mesure, cap, voie_libre):
        if self.termine:
            return
        dt = 0 if self._precedent is None else maintenant - self._precedent
        self._precedent = maintenant
        radio_valide = (mesure is not None and all(map(math.isfinite, mesure))
                        and 0 <= maintenant - mesure[0] <= self.age_gps
                        and mesure[0] >= self._dernier_fix)
        nouveau = radio_valide and mesure[0] > self._dernier_fix
        if nouveau:
            intervalle = mesure[0] - self._dernier_fix
            if math.isfinite(intervalle):
                self._intervalles_gps.append(min(intervalle, self.age_gps))
            self._dernier_fix, x, y = mesure
            self._points.append((x, y))
        if not radio_valide:
            self._points.clear()
            self.attente = "position Qorvo indisponible"
        elif cap is None or not math.isfinite(cap):
            self.attente = "gyroscope indisponible ou en étalonnage"
        elif voie_libre is not True:
            self.attente = "obstacle devant le robot" if voie_libre is False else "scan lidar indisponible"
        else:
            self.attente = ""
        if self.attente:
            self.robot.arreter()
            self._reference_vitesse = None
            cap_valide = cap is not None and math.isfinite(cap)
            if voie_libre is False or not radio_valide or not cap_valide:
                self._libre_depuis = None
            if self._impulsion_fin is not None:
                self._duree_impulsion = max(0.001, maintenant - self._debut_impulsion)
                self._impulsion_fin = None
                self._repos_fin = maintenant + 0.22
            if not cap_valide:
                self._sens_impulsion = 0
            if self._stable_depuis is not None and (not cap_valide or abs(ecart_cap(self.cap_cible, cap)) > 4):
                self._stable_depuis = None
            if not radio_valide:
                self._fixes_repos.clear()
            elif nouveau and self.etat in ("DÉPART", "MESURE DU CAP", "CONFIRMATION DU COIN"):
                self._position_au_repos(True, (x, y))
            return

        _, x, y = mesure
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
