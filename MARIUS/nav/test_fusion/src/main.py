#!/usr/bin/env python3
# main.py - Programme principal de navigation autonome

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
    def __init__(self):
        # Initialisation des états
        self.boat_position = None      # Position actuelle [lat, lon]
        self.boat_heading = 0.0       # Cap actuel en degrés
        self.boat_speed = 0.0         # Vitesse actuelle en km/h
        self.wind_direction = 0.0     # Direction du vent en degrés
        self.wind_speed = 0.0         # Vitesse du vent en noeuds
        self.current_tack = 1         # Bord actuel (1 = tribord, -1 = bâbord)
        
        # Navigation
        self.waypoints = []           # Liste des waypoints [ [lat1,lon1], ... ]
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.safety_triggered = False
        
        # Communication
        self.connected_clients = set()  # Clients WebSocket connectés
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((CONFIG["udp_ip"], CONFIG["udp_port"]))
        
        # Initialisation des composants
        self.actuator_calculator = PureSailboatActuators()
        self.stop_event = threading.Event()
        
        # Initialisation GPIO et PWM
        self.init_gpio()
        
        # Chargement des waypoints
        self.load_waypoints()
        
        # Démarrer les threads
        self.start_threads()

    def init_gpio(self):
        """Initialisation des GPIO et PWM"""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        # Configuration des pins
        self.ENA_GV = 32      # PWM Grand Voile
        self.SENS_GV = 29     # Sens Grand Voile
        self.ENA_SAF = 33     # PWM Safran
        self.SENS_SAF = 31    # Sens Safran
        
        GPIO.setup(self.ENA_GV, GPIO.OUT)
        GPIO.setup(self.SENS_GV, GPIO.OUT)
        GPIO.setup(self.ENA_SAF, GPIO.OUT)
        GPIO.setup(self.SENS_SAF, GPIO.OUT)
        
        # Initialisation PWM
        self.PWM_GV = GPIO.PWM(self.ENA_GV, 100)  # Fréquence 100Hz
        self.PWM_GV.start(0)
        self.PWM_SAF = GPIO.PWM(self.ENA_SAF, 100)
        self.PWM_SAF.start(0)

    def load_waypoints(self):
        """Charge les waypoints depuis le fichier JSON"""
        try:
            if os.path.exists(CONFIG["gps_points_file"]):
                with open(CONFIG["gps_points_file"], 'r') as f:
                    data = json.load(f)
                    self.waypoints = [(p['lat'], p['lng']) for p in data.get("points", [])]
                    print(f"Waypoints chargés: {len(self.waypoints)} points")
        except Exception as e:
            print(f"Erreur lors du chargement des waypoints: {e}")
            self.waypoints = []

    def start_threads(self):
        """Démarre tous les threads nécessaires"""
        # Thread pour lire le GPS
        self.gps_thread = threading.Thread(
            target=self.read_gps,
            daemon=True
        )
        self.gps_thread.start()
        
        # Thread pour lire le capteur de vent Airmar
        self.airmar_thread = threading.Thread(
            target=self.read_airmar,
            daemon=True
        )
        self.airmar_thread.start()
        
        # Thread pour traiter les messages UDP
        self.udp_thread = threading.Thread(
            target=self.process_udp_messages,
            daemon=True
        )
        self.udp_thread.start()
        
        # Thread pour le serveur WebSocket
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server,  # This is now a regular method, not a coroutine
            daemon=True
        )
        self.websocket_thread.start()

    def read_gps(self):
        """Lit en continu les données GPS"""
        try:
            with serial.Serial(CONFIG["gps_port"], CONFIG["gps_baudrate"], timeout=1) as ser:
                print(f"GPS connecté sur {CONFIG['gps_port']}")
                nmr = NMEAReader(ser)
                
                while not self.stop_event.is_set():
                    try:
                        raw_data, parsed_data = nmr.read()
                        
                        if isinstance(parsed_data, NMEAMessage):
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
                            
                            elif parsed_data.msgID == "VTG":
                                self.boat_heading = parsed_data.cogt
                                self.boat_speed = parsed_data.spd_over_grnd_kmph
                                # Envoi des données de cap et vitesse via UDP
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
        """Lit en continu les données du capteur de vent Airmar"""
        try:
            with serial.Serial(CONFIG["airmar_port"], CONFIG["airmar_baudrate"], timeout=1) as ser:
                print(f"Capteur Airmar connecté sur {CONFIG['airmar_port']}")
                nmr = NMEAReader(ser)
                
                while not self.stop_event.is_set():
                    try:
                        raw_data, parsed_data = nmr.read()
                        
                        if isinstance(parsed_data, NMEAMessage):
                            if parsed_data.msgID == "MWV":
                                if parsed_data.ref == "R":  # Vent relatif
                                    self.wind_direction = parsed_data.wind_angle
                                    self.wind_speed = parsed_data.wind_speed
                                    # Envoi des données de vent via UDP
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
        """Traite les messages UDP entrants"""
        while not self.stop_event.is_set():
            try:
                data, _ = self.udp_socket.recvfrom(1024)
                message = json.loads(data.decode('utf-8'))
                
                if message.get('type') == 'command':
                    if message['command'] == 'start_navigation':
                        self.start_navigation()
                    elif message['command'] == 'stop_navigation':
                        self.stop_navigation()
                        
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Erreur traitement message UDP: {e}")

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections"""
        self.connected_clients.add(websocket)
        try:
            print(f"New connection from {websocket.remote_address}")  # Log new connections
            
            # Send initial status
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
            
            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    # Process message data here if needed
                    print(f"Received message: {data}")  # Log received messages
                    
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
        """Run the WebSocket server in a separate thread"""
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)

        # This wrapper must accept both parameters
        async def handler(websocket, path):
            await self.websocket_handler(websocket, path)

        async def server_main():
            try:
                self.ws_server = await websockets.serve(
                    handler,  # Pass the handler that accepts both parameters
                    "0.0.0.0",
                    CONFIG["websocket_port"],
                    ping_interval=None,  # Disable automatic pings
                    ping_timeout=None    # Disable ping timeout
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
        """Active la navigation autonome"""
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
        """Arrête la navigation autonome"""
        self.navigation_active = False
        print("Navigation arrêtée")
        
        # Arrêt des actionneurs
        self.PWM_SAF.ChangeDutyCycle(0)
        self.PWM_GV.ChangeDutyCycle(0)
        
        # Notification aux clients WebSocket
        self.broadcast_websocket({
            "type": "navigation_stopped"
        })

    def broadcast_websocket(self, message):
        """Envoie un message à tous les clients WebSocket connectés"""
        if self.connected_clients:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            async def send_messages():
                await asyncio.wait([client.send(json.dumps(message)) for client in self.connected_clients])
            
            loop.run_until_complete(send_messages())
            loop.close()

    def calculate_distance(self, pos1, pos2):
        """Calcule la distance entre deux positions en mètres (formule haversine)"""
        if None in pos1 or None in pos2:
            return float('inf')
            
        lat1, lon1 = np.radians(pos1)
        lat2, lon2 = np.radians(pos2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        return 6371000 * c  # Rayon de la Terre en mètres

    def navigate(self):
        """Exécute une itération de navigation autonome"""
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
            
            if self.current_waypoint_index >= len(self.waypoints):
                print("Tous les waypoints atteints!")
                self.stop_navigation()
                return
                
        # Calcul des angles des actionneurs
        rudder_angle, sail_angle = self.actuator_calculator.compute_actuators(
            boat_pos=np.array(self.boat_position) if self.boat_position else np.array([0, 0]),
            boat_heading=np.radians(self.boat_heading),
            wind_dir=np.radians(self.wind_direction),
            line_start=np.array(self.boat_position) if self.boat_position else np.array([0, 0]),
            line_end=np.array(self.waypoints[self.current_waypoint_index]),
            current_tack=self.current_tack
        )
        
        # Contrôle des actionneurs
        current_gv, pwm_gv = control_gv(
            sail_angle,
            self.PWM_GV,
            self.SENS_GV
        )
        
        current_saf, pwm_saf = control_saf(
            rudder_angle,
            self.PWM_SAF,
            self.SENS_SAF
        )
        
        # Log des données de navigation
        self.log_navigation(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)
        
        # Affichage du statut
        self.display_status(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)

    def check_safety_conditions(self):
        """Vérifie les conditions de sécurité"""
        if self.boat_position is None:
            self.trigger_safety("Position GPS invalide")
            return True
            
        if self.wind_speed > 20:  # Vent trop fort (20 noeuds)
            self.trigger_safety(f"Vent trop fort ({self.wind_speed} noeuds)")
            return True
            
        return False

    def trigger_safety(self, reason):
        """Active le mode sécurité"""
        if not self.safety_triggered:
            print(f"Sécurité activée: {reason}")
            self.safety_triggered = True
            self.navigation_active = False
            self.PWM_SAF.ChangeDutyCycle(0)
            self.PWM_GV.ChangeDutyCycle(0)
            
            # Notification aux clients
            self.broadcast_websocket({
                "type": "safety_triggered",
                "reason": reason
            })

    def log_navigation(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """Enregistre les données de navigation"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return
            
        current_wp = self.waypoints[self.current_waypoint_index]
        
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
        
        with open(CONFIG["log_file"], "a") as f:
            f.write("".join(log_entry))

    def display_status(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """Affiche le statut de navigation"""
        os.system('clear' if os.name == 'posix' else 'cls')
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
        """Nettoyage des ressources"""
        self.stop_event.set()
        self.PWM_GV.stop()
        self.PWM_SAF.stop()
        cleanup()  # Nettoyage de l'ADC
        GPIO.cleanup()
        self.udp_socket.close()

        # Properly close the WebSocket server if it exists
        if hasattr(self, 'ws_server') and self.ws_server is not None:
            if hasattr(self, 'ws_loop') and self.ws_loop is not None:
                # Create a task to close the server
                async def close_server():
                    try:
                        self.ws_server.close()
                        await self.ws_server.wait_closed()
                    except Exception as e:
                        print(f"Error closing WebSocket server: {e}")

                # Run the close operation if loop is still running
                if not self.ws_loop.is_closed():
                    try:
                        # Create a new task in the existing loop
                        if self.ws_loop.is_running():
                            asyncio.run_coroutine_threadsafe(close_server(), self.ws_loop)
                            # Give some time for the server to close
                            time.sleep(1)
                        else:
                            self.ws_loop.run_until_complete(close_server())
                    except Exception as e:
                        print(f"Error during WebSocket server shutdown: {e}")
                    finally:
                        if not self.ws_loop.is_closed():
                            self.ws_loop.close()                    
def main():
    # Initialisation du système
    boat = AutonomousBoat()
    
    try:
        # Boucle principale
        while not boat.stop_event.is_set():
            boat.navigate()
            time.sleep(CONFIG["control_interval"])
            
    except KeyboardInterrupt:
        print("\nArrêt demandé...")
    finally:
        boat.cleanup()
        print("Système arrêté proprement")

if __name__ == "__main__":
    main()