#!/usr/bin/env python3
# main.py - Programme principal de navigation autonome pour voilier robotisé

"""
Programme principal pour la navigation autonome d'un voilier robotisé.
Ce système intègre:
- Lecture des données GPS et capteurs
- Calcul de trajectoire
- Contrôle des actionneurs (voile et safran)
- Communication via UDP et WebSocket
- Gestion des waypoints
- Sécurité
"""

import asyncio
import json
import threading
import time
import numpy as np
from queue import Queue
from datetime import datetime
import socket
import serial
import RPi.GPIO as GPIO
from pynmeagps import NMEAReader, NMEAMessage
from navV5_T import PureSailboatActuators
from prop_saf_e_gv_V2 import control_gv, control_saf, cleanup
from tools import udp_forwarder
import websockets
import os

# --- Configuration globale ---
CONFIG = {
    "control_interval": 0.5,          # Intervalle de contrôle en secondes
    "waypoint_tolerance": 10.0,       # Tolérance pour atteindre un waypoint (mètres)
    "log_file": "navigation_log.csv", # Fichier de logs
    "gps_port": "/dev/ttyUSB0",       # Port série du GPS
    "gps_baudrate": 4800,             # Baudrate du GPS
    "airmar_port": "/dev/ttyACM0",    # Port série du capteur de vent Airmar
    "airmar_baudrate": 4800,          # Baudrate du capteur Airmar
    "udp_ip": "0.0.0.0",              # IP pour la communication UDP
    "udp_port": 4000,                 # Port pour la communication UDP
    "websocket_port": 8765,           # Port pour le serveur WebSocket
    "gps_points_file": "gps_points.json"  # Fichier de stockage des waypoints
}

class AutonomousBoat:
    """
    Classe principale gérant la navigation autonome du voilier.
    Gère les capteurs, les actionneurs, la communication et la logique de navigation.
    """
    
    def __init__(self):
        """Initialise tous les composants du système"""
        
        # --- Initialisation des états ---
        self.boat_position = None      # Position actuelle [lat, lon]
        self.boat_heading = 0.0       # Cap actuel en degrés
        self.boat_speed = 0.0         # Vitesse actuelle en km/h
        self.wind_direction = 0.0     # Direction du vent en degrés
        self.wind_speed = 0.0         # Vitesse du vent en noeuds
        self.current_tack = 1         # Bord actuel (1 = tribord, -1 = bâbord)
        
        # --- Navigation ---
        self.waypoints = []           # Liste des waypoints [ [lat1,lon1], ... ]
        self.current_waypoint_index = 0  # Index du waypoint courant
        self.navigation_active = False  # Si la navigation est en cours
        self.safety_triggered = False  # Si le mode sécurité est activé
        
        # --- Communication ---
        self.connected_clients = set()  # Clients WebSocket connectés
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((CONFIG["udp_ip"], CONFIG["udp_port"]))
        
        # --- Initialisation des composants ---
        self.actuator_calculator = PureSailboatActuators()  # Pour calculer les angles des actionneurs
        self.stop_event = threading.Event()  # Pour arrêter proprement les threads
        
        # Initialisation GPIO et PWM
        self.init_gpio()
        
        # Chargement des waypoints depuis fichier
        self.load_waypoints()
        
        # Démarrer les threads pour les capteurs et communication
        self.start_threads()

    def init_gpio(self):
        """Initialise les GPIO et PWM pour contrôler les actionneurs"""
        GPIO.setmode(GPIO.BOARD)  # Utilise la numérotation physique des broches
        GPIO.setwarnings(False)   # Désactive les avertissements
        
        # Configuration des pins GPIO
        self.ENA_GV = 32      # PWM Grand Voile (Enable)
        self.SENS_GV = 29     # Sens de rotation Grand Voile
        self.ENA_SAF = 33     # PWM Safran (Enable)
        self.SENS_SAF = 31    # Sens de rotation Safran
        
        # Configuration des broches en sortie
        GPIO.setup(self.ENA_GV, GPIO.OUT)
        GPIO.setup(self.SENS_GV, GPIO.OUT)
        GPIO.setup(self.ENA_SAF, GPIO.OUT)
        GPIO.setup(self.SENS_SAF, GPIO.OUT)
        
        # Initialisation des signaux PWM (100Hz)
        self.PWM_GV = GPIO.PWM(self.ENA_GV, 100)  # Fréquence 100Hz
        self.PWM_GV.start(0)  # Démarre avec duty cycle à 0%
        self.PWM_SAF = GPIO.PWM(self.ENA_SAF, 100)
        self.PWM_SAF.start(0)

    def load_waypoints(self):
        """
        Charge les waypoints depuis un fichier JSON.
        Format attendu: {"points": [{"lat": x, "lng": y}, ...]}
        """
        try:
            if os.path.exists(CONFIG["gps_points_file"]):
                with open(CONFIG["gps_points_file"], 'r') as f:
                    data = json.load(f)
                    # Extraction des coordonnées depuis le fichier JSON
                    self.waypoints = [(p['lat'], p['lng']) for p in data.get("points", [])]
                    print(f"Waypoints chargés: {len(self.waypoints)} points")
        except Exception as e:
            print(f"Erreur lors du chargement des waypoints: {e}")
            self.waypoints = []  # Liste vide en cas d'erreur

    def start_threads(self):
        """Démarre tous les threads nécessaires pour le fonctionnement du système"""
        
        # Thread pour lire les données GPS
        self.gps_thread = threading.Thread(
            target=self.read_gps,
            daemon=True  # Le thread s'arrête quand le programme principal s'arrête
        )
        self.gps_thread.start()
        
        # Thread pour lire le capteur de vent Airmar
        self.airmar_thread = threading.Thread(
            target=self.read_airmar,
            daemon=True
        )
        self.airmar_thread.start()
        
        # Thread pour traiter les messages UDP entrants
        self.udp_thread = threading.Thread(
            target=self.process_udp_messages,
            daemon=True
        )
        self.udp_thread.start()
        
        # Thread pour le serveur WebSocket (communication avec interface web)
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server,
            daemon=True
        )
        self.websocket_thread.start()

    def read_gps(self):
        """
        Lit en continu les données du récepteur GPS via le port série.
        Traite les messages NMEA GGA (position) et VTG (cap et vitesse).
        """
        try:
            with serial.Serial(CONFIG["gps_port"], CONFIG["gps_baudrate"], timeout=1) as ser:
                print(f"GPS connecté sur {CONFIG['gps_port']}")
                nmr = NMEAReader(ser)  # Lecteur de messages NMEA
                
                while not self.stop_event.is_set():
                    try:
                        raw_data, parsed_data = nmr.read()
                        
                        if isinstance(parsed_data, NMEAMessage):
                            # Message GGA - Position GPS
                            if parsed_data.msgID == "GGA":
                                self.boat_position = (parsed_data.lat, parsed_data.lon)
                                # Envoi des données GPS via UDP
                                udp_forwarder({
                                    "type": "gps_update",
                                    "data": {
                                        "lat": parsed_data.lat,
                                        "lon": parsed_data.lon,
                                        "alt": parsed_data.alt,
                                        "quality": parsed_data.quality
                                    }
                                }, [(CONFIG["udp_ip"], CONFIG["udp_port"])])
                            
                            # Message VTG - Cap et vitesse
                            elif parsed_data.msgID == "VTG":
                                self.boat_heading = parsed_data.cogt
                                self.boat_speed = parsed_data.spd_over_grnd_kmph
                                # Envoi des données via UDP
                                udp_forwarder({
                                    "type": "attitude_update",
                                    "data": {
                                        "yaw": parsed_data.cogt,
                                        "speed": parsed_data.spd_over_grnd_kmph
                                    }
                                }, [(CONFIG["udp_ip"], CONFIG["udp_port"])])
                                
                    except Exception as e:
                        print(f"Erreur lecture GPS: {e}")
                        time.sleep(1)
                        
        except Exception as e:
            print(f"Erreur connexion GPS: {e}")
        finally:
            print("GPS arrêté")

    def read_airmar(self):
        """
        Lit en continu les données du capteur de vent Airmar.
        Traite les messages NMEA MWV (direction et vitesse du vent).
        """
        try:
            with serial.Serial(CONFIG["airmar_port"], CONFIG["airmar_baudrate"], timeout=1) as ser:
                print(f"Capteur Airmar connecté sur {CONFIG['airmar_port']}")
                nmr = NMEAReader(ser)
                
                while not self.stop_event.is_set():
                    try:
                        raw_data, parsed_data = nmr.read()
                        
                        if isinstance(parsed_data, NMEAMessage):
                            # Message MWV - Vent relatif
                            if parsed_data.msgID == "MWV":
                                if parsed_data.ref == "R":  # 'R' = vent relatif au bateau
                                    self.wind_direction = parsed_data.wind_angle
                                    self.wind_speed = parsed_data.wind_speed
                                    # Envoi des données via UDP
                                    udp_forwarder({
                                        "type": "wind_update",
                                        "data": {
                                            "direction": parsed_data.wind_angle,
                                            "speed": parsed_data.wind_speed
                                        }
                                    }, [(CONFIG["udp_ip"], CONFIG["udp_port"])])
                                
                    except Exception as e:
                        print(f"Erreur lecture Airmar: {e}")
                        time.sleep(1)
                        
        except Exception as e:
            print(f"Erreur connexion Airmar: {e}")
        finally:
            print("Airmar arrêté")

    def process_udp_messages(self):
        """
        Traite les messages UDP entrants.
        Permet de recevoir des commandes externes (début/arrêt navigation).
        """
        while not self.stop_event.is_set():
            try:
                # Réception des messages (bloquant avec timeout)
                data, _ = self.udp_socket.recvfrom(1024)
                message = json.loads(data.decode('utf-8'))
                
                # Traitement selon le type de message
                if message.get('type') == 'command':
                    if message['command'] == 'start_navigation':
                        self.start_navigation()
                    elif message['command'] == 'stop_navigation':
                        self.stop_navigation()
                        
            except socket.timeout:
                pass  # Pas de message reçu
            except Exception as e:
                print(f"Erreur traitement message UDP: {e}")

    async def websocket_handler(self, websocket, path):
        """
        Gère les connexions WebSocket.
        - Envoie l'état initial au nouveau client
        - Reçoit les messages des clients
        - Gère la déconnexion
        """
        self.connected_clients.add(websocket)
        try:
            print(f"New connection from {websocket.remote_address}")
            
            # Envoi de l'état initial au client
            await websocket.send(json.dumps({
                "type": "status",
                "position": self.boat_position,
                "heading": self.boat_heading,
                "speed": self.boat_speed,
                "wind": {
                    "direction": self.wind_direction,
                    "speed": self.wind_speed
                },
                "navigation_active": self.navigation_active,
                "current_waypoint": self.current_waypoint_index,
                "waypoints": self.waypoints
            }))
            
            # Boucle de réception des messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    print(f"Received message: {data}")  # Log des messages
                    
                    # Ici on pourrait traiter des commandes spécifiques
                    # par exemple changer de waypoint, etc.
                    
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": "Invalid JSON format"
                    }))
                    
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected")
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            self.connected_clients.discard(websocket)
            
    def run_websocket_server(self):
        """
        Lance le serveur WebSocket dans un thread séparé.
        Utilise asyncio pour gérer les connexions asynchrones.
        """
        # Crée une nouvelle event loop pour asyncio
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)

        # Handler qui accepte les paramètres standards de websockets
        async def handler(websocket, path):
            await self.websocket_handler(websocket, path)

        async def server_main():
            try:
                # Création du serveur WebSocket
                self.ws_server = await websockets.serve(
                    handler,
                    "0.0.0.0",  # Écoute sur toutes les interfaces
                    CONFIG["websocket_port"],
                    ping_interval=None,  # Désactive les ping automatiques
                    ping_timeout=None    # Désactive le timeout des ping
                )
                print(f"WebSocket server started on port {CONFIG['websocket_port']}")
                await self.ws_server.wait_closed()
            except Exception as e:
                print(f"WebSocket server error: {e}")
                self.ws_server = None

        try:
            self.ws_loop.run_until_complete(server_main())
        except Exception as e:
            print(f"Failed to start WebSocket server: {e}")
            self.ws_server = None
        finally:
            if not self.ws_loop.is_closed():
                self.ws_loop.close()
                                        
    def start_navigation(self):
        """
        Active la navigation autonome vers les waypoints chargés.
        Réinitialise l'index du waypoint courant et les flags.
        """
        if not self.waypoints:
            print("Aucun waypoint défini!")
            return
            
        self.navigation_active = True
        self.safety_triggered = False
        self.current_waypoint_index = 0
        print(f"Début de navigation vers {len(self.waypoints)} waypoints")
        
        # Notification aux clients WebSocket
        self.broadcast_websocket({
            "type": "navigation_started",
            "waypoints": len(self.waypoints)
        })

    def stop_navigation(self):
        """
        Arrête la navigation autonome.
        Coupe les actionneurs et notifie les clients.
        """
        self.navigation_active = False
        print("Navigation arrêtée")
        
        # Arrêt des actionneurs (duty cycle à 0%)
        self.PWM_SAF.ChangeDutyCycle(0)
        self.PWM_GV.ChangeDutyCycle(0)
        
        # Notification aux clients
        self.broadcast_websocket({
            "type": "navigation_stopped"
        })

    def broadcast_websocket(self, message):
        """
        Envoie un message à tous les clients WebSocket connectés.
        Crée une nouvelle event loop si nécessaire.
        """
        if self.connected_clients:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            async def send_messages():
                # Envoi asynchrone à tous les clients
                await asyncio.wait([client.send(json.dumps(message)) for client in self.connected_clients])
            
            loop.run_until_complete(send_messages())
            loop.close()

    def calculate_distance(self, pos1, pos2):
        """
        Calcule la distance entre deux positions en mètres (formule haversine).
        Retourne la distance en mètres ou infinity si positions invalides.
        """
        if None in pos1 or None in pos2:
            return float('inf')
            
        # Conversion des degrés en radians
        lat1, lon1 = np.radians(pos1)
        lat2, lon2 = np.radians(pos2)
        
        # Différences de coordonnées
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Formule haversine
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        return 6371000 * c  # Rayon de la Terre en mètres

    def navigate(self):
        """
        Exécute une itération de navigation autonome:
        1. Vérifie les conditions de sécurité
        2. Calcule la distance au waypoint courant
        3. Si waypoint atteint, passe au suivant
        4. Calcule les angles des actionneurs
        5. Contrôle les actionneurs
        6. Log les données
        """
        if not self.navigation_active or not self.waypoints:
            return
            
        # Vérification des conditions de sécurité
        if self.check_safety_conditions():
            return
            
        current_waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.calculate_distance(self.boat_position, current_waypoint)
        
        # Vérification si le waypoint est atteint
        if distance < CONFIG["waypoint_tolerance"]:
            print(f"Waypoint {self.current_waypoint_index + 1} atteint!")
            self.current_waypoint_index += 1
            
            # Vérification si tous les waypoints sont atteints
            if self.current_waypoint_index >= len(self.waypoints):
                print("Tous les waypoints atteints!")
                self.stop_navigation()
                return
                
        # Calcul des angles optimaux pour les actionneurs
        rudder_angle, sail_angle = self.actuator_calculator.compute_actuators(
            boat_pos=np.array(self.boat_position) if self.boat_position else np.array([0, 0]),
            boat_heading=np.radians(self.boat_heading),
            wind_dir=np.radians(self.wind_direction),
            line_start=np.array(self.boat_position) if self.boat_position else np.array([0, 0]),
            line_end=np.array(self.waypoints[self.current_waypoint_index]),
            current_tack=self.current_tack
        )
        
        # Contrôle de la grand-voile
        current_gv, pwm_gv = control_gv(
            sail_angle,
            self.PWM_GV,
            self.SENS_GV
        )
        
        # Contrôle du safran
        current_saf, pwm_saf = control_saf(
            rudder_angle,
            self.PWM_SAF,
            self.SENS_SAF
        )
        
        # Enregistrement des données de navigation
        self.log_navigation(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)
        
        # Affichage du statut dans la console
        self.display_status(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)

    def check_safety_conditions(self):
        """
        Vérifie les conditions pouvant déclencher le mode sécurité.
        Retourne True si une condition de sécurité est détectée.
        """
        if self.boat_position is None:
            self.trigger_safety("Position GPS invalide")
            return True
            
        if self.wind_speed > 20:  # Vent trop fort (20 noeuds)
            self.trigger_safety(f"Vent trop fort ({self.wind_speed} noeuds)")
            return True
            
        return False

    def trigger_safety(self, reason):
        """
        Active le mode sécurité avec une raison donnée.
        Arrête la navigation et coupe les actionneurs.
        """
        if not self.safety_triggered:
            print(f"Sécurité activée: {reason}")
            self.safety_triggered = True
            self.navigation_active = False
            # Arrêt des actionneurs
            self.PWM_SAF.ChangeDutyCycle(0)
            self.PWM_GV.ChangeDutyCycle(0)
            
            # Notification aux clients
            self.broadcast_websocket({
                "type": "safety_triggered",
                "reason": reason
            })

    def log_navigation(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """
        Enregistre les données de navigation dans un fichier CSV.
        Format: timestamp,position,cap,vitesse,vent,waypoint,actionneurs,etc.
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return
            
        current_wp = self.waypoints[self.current_waypoint_index]
        
        # Formatage de la ligne de log
        log_entry = (
            f"{datetime.now().isoformat()},",
            f"{self.boat_position[0] if self.boat_position else 'NaN'},",
            f"{self.boat_position[1] if self.boat_position else 'NaN'},",
            f"{self.boat_heading},",
            f"{self.boat_speed},",
            f"{self.wind_direction},",
            f"{self.wind_speed},",
            f"{self.current_waypoint_index},",
            f"{current_wp[0]},",
            f"{current_wp[1]},",
            f"{distance},",
            f"{np.degrees(rudder_angle)},",
            f"{np.degrees(sail_angle)},",
            f"{pwm_gv},",
            f"{pwm_saf}\n"
        )
        
        # Écriture dans le fichier
        with open(CONFIG["log_file"], "a") as f:
            f.write("".join(log_entry))

    def display_status(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """
        Affiche le statut de navigation dans la console.
        Efface l'écran et affiche les informations mises à jour.
        """
        os.system('clear' if os.name == 'posix' else 'cls')  # Efface la console
        
        print("=== NAVIGATION AUTONOME ===")
        print(f"Position: {self.boat_position}")
        print(f"Cap: {self.boat_heading:.1f}° | Vitesse: {self.boat_speed:.1f} km/h")
        print(f"Vent: {self.wind_direction:.1f}° ({self.wind_speed:.1f} kt) | Bord: {'Tribord' if self.current_tack == 1 else 'Bâbord'}")
        print(f"\nWaypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        print(f"Distance: {distance:.1f}m | Tolérance: {CONFIG['waypoint_tolerance']}m")
        print(f"\nCommandes:")
        print(f"GV: {np.degrees(sail_angle):.1f}° (PWM: {pwm_gv}%)")
        print(f"SAF: {np.degrees(rudder_angle):.1f}° (PWM: {pwm_saf}%)")
        print(f"\nSécurité: {'ACTIVE' if self.safety_triggered else 'OK'}")

    def cleanup(self):
        """
        Nettoyage des ressources avant l'arrêt du programme.
        - Arrêt des threads
        - Arrêt des PWM
        - Nettoyage GPIO
        - Fermeture des sockets
        """
        # Signal d'arrêt pour les threads
        self.stop_event.set()
        
        # Arrêt des PWM
        self.PWM_GV.stop()
        self.PWM_SAF.stop()
        
        # Nettoyage de l'ADC et GPIO
        cleanup()
        GPIO.cleanup()
        
        # Fermeture du socket UDP
        self.udp_socket.close()

        # Fermeture propre du serveur WebSocket
        if hasattr(self, 'ws_server') and self.ws_server is not None:
            if hasattr(self, 'ws_loop') and self.ws_loop is not None:
                async def close_server():
                    try:
                        self.ws_server.close()
                        await self.ws_server.wait_closed()
                    except Exception as e:
                        print(f"Error closing WebSocket server: {e}")

                if not self.ws_loop.is_closed():
                    try:
                        if self.ws_loop.is_running():
                            asyncio.run_coroutine_threadsafe(close_server(), self.ws_loop)
                            time.sleep(1)  # Laisse le temps de fermer
                        else:
                            self.ws_loop.run_until_complete(close_server())
                    except Exception as e:
                        print(f"Error during WebSocket server shutdown: {e}")
                    finally:
                        if not self.ws_loop.is_closed():
                            self.ws_loop.close()                    
def main():
    """
    Fonction principale.
    Initialise le système et lance la boucle de navigation.
    """
    # Initialisation du système
    boat = AutonomousBoat()
    
    try:
        # Boucle principale
        while not boat.stop_event.is_set():
            boat.navigate()  # Exécute une itération de navigation
            time.sleep(CONFIG["control_interval"])  # Attend le prochain cycle
            
    except KeyboardInterrupt:
        print("\nArrêt demandé...")
    finally:
        boat.cleanup()  # Nettoyage avant arrêt
        print("Système arrêté proprement")

if __name__ == "__main__":
    main()