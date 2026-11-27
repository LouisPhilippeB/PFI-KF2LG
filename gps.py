"""GPS intérieur Qorvo DWM1001. Le tag transmet lignes POS,x,y,z."""

import math
import threading
import time


class GPS:
    def __init__(self, port="/dev/ttyACM0", qualite_min=50, age_max=1.0):
        if not isinstance(qualite_min, int) or not 0 <= qualite_min <= 100:
            raise ValueError("La qualité minimale doit être un entier entre 0 et 100.")
        if not math.isfinite(age_max) or age_max <= 0:
            raise ValueError("L'âge maximal doit être positif et fini.")
        self.port = port
        self.qualite_min = qualite_min
        self.age_max = age_max
        self._verrou = threading.Lock()
        self._cycle = threading.Lock()
        self._fin = threading.Event()
        self._fil = None
        self._derniere = None
        self._erreur = None

    def demarrer(self):
        import serial

        with self._cycle:
            if self._fil is not None and self._fil.is_alive():
                return
            self._fin.clear()
            with self._verrou:
                self._derniere = self._erreur = None
            liaison = serial.Serial(
                self.port, 115200, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=min(0.05, self.age_max / 2), write_timeout=0.5,
            )
            try:
                liaison.reset_input_buffer()
                fil = threading.Thread(target=self._ecouter, args=(liaison,), name="GPS")
                fil.start()
                self._fil = fil
            except BaseException:
                liaison.close()
                raise

    def _traiter_ligne(self, ligne, instant):
        ligne = ligne.strip()
        if ligne.startswith(b"dwm>"):
            ligne = ligne[4:].lstrip()
        if not ligne.startswith(b"POS,"):
            return False
        mesure = None
        try:
            _, x, y, z, qualite = ligne.split(b",")
            x, y, z = float(x), float(y), float(z)
            if all(map(math.isfinite, (x, y, z))) and self.qualite_min <= int(qualite) <= 100:
                mesure = instant, x, y
        except ValueError:
            pass
        with self._verrou:
            self._derniere = mesure
        return True

    def _ecouter(self, liaison):
        commande = b"\r\r"
        echeance = time.monotonic() + max(1.2, self.age_max + 0.2)
        fragment = bytearray()
        debut_fragment = None
        ignorer_ligne = False
        erreur = None
        try:
            while not self._fin.is_set():
                bloc = liaison.read_until(b"\n", size=256)
                maintenant = time.monotonic()
                if bloc:
                    if debut_fragment is None:
                        debut_fragment = maintenant
                    if len(fragment) + len(bloc) > 256 or maintenant - debut_fragment > self.age_max:
                        ignorer_ligne = True
                    if not ignorer_ligne:
                        fragment.extend(bloc)
                    if bloc.endswith(b"\n"):
                        if not ignorer_ligne and self._traiter_ligne(bytes(fragment), debut_fragment):
                            commande = None
                        fragment.clear()
                        debut_fragment = None
                        ignorer_ligne = False
                if commande and maintenant >= echeance and not self._fin.is_set():
                    liaison.write(commande)
                    commande = b"lep\n" if commande == b"\r\r" else None
                    echeance = maintenant + 1.0
        except (OSError, ValueError) as exc:
            erreur = str(exc)
        finally:
            try:
                liaison.close()
            finally:
                with self._verrou:
                    self._derniere = None
                    self._erreur = erreur

    def mesure(self):
        with self._verrou:
            mesure = self._derniere
            if mesure is not None and time.monotonic() - mesure[0] <= self.age_max:
                return mesure
            return None

    def position(self):
        mesure = self.mesure()
        return None if mesure is None else mesure[1:]

    @property
    def erreur(self):
        with self._verrou:
            return self._erreur

    def arreter(self):
        with self._cycle:
            self._fin.set()
            if self._fil is not None:
                self._fil.join()
                self._fil = None


if __name__ == "__main__":
    import argparse

    analyseur = argparse.ArgumentParser(description=__doc__)
    analyseur.add_argument("--port", default="/dev/ttyACM0")
    analyseur.add_argument("--qualite-min", type=int, default=50)
    options = analyseur.parse_args()
    try:
        gps = GPS(options.port, options.qualite_min)
    except ValueError as exc:
        analyseur.error(str(exc))
    try:
        gps.demarrer()
        print("En attente du tag Qorvo.", flush=True)
        precedente = None
        while gps.erreur is None:
            mesure = gps.mesure()
            if mesure != precedente:
                if mesure is None:
                    print("Position indisponible.", flush=True)
                else:
                    print(f"x = {mesure[1]:.2f} m, y = {mesure[2]:.2f} m", flush=True)
                precedente = mesure
            time.sleep(0.05)
        raise SystemExit(gps.erreur)
    except KeyboardInterrupt:
        pass
    except ImportError as exc:
        raise SystemExit("Le diagnostic GPS nécessite pyserial.") from exc
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Impossible d'ouvrir le tag sur {options.port} ({exc}).") from exc
    finally:
        gps.arreter()
