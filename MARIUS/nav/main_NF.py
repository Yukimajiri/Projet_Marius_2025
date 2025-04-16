#!/usr/bin/env python3
# main.py - Programme principal de navigation autonome complet

import json
import time
import threading
import os
import numpy as np
from queue import Queue
from datetime import datetime
import socket
import serial
from pynmeagps import NMEAReader, NMEAMessage
from navV5_T import PureSailboatActuators
from prop_saf_e_gv_V2 import control_gv, control_saf, cleanup
import RPi.GPIO as GPIO
from tools import udp_listener

# --- Configuration ---
CONFIG = {
    "control_interval": 0.5,
    "waypoint_tolerance": 10.0,
    "log_file": "navigation_log.csv",
    "max_rudder_pwm": 100,
    "max_sail_pwm": 100,
    "safety_timeout": 5.0,
    "gps_port": "/dev/ttyUSB0",
    "gps_baudrate": 4800,
    "airmar_port": "/dev/ttyACM0",
    "airmar_baudrate": 4800,
    "udp_ip": "127.0.0.1",
    "udp_port": 4000,
    "gv_kp": 20.0,
    "saf_kp": 3.0,
    "spi_bus": 0,
    "spi_device": 0
}

class AirmarReader:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.wind_direction = 0.0
        self.wind_speed = 0.0
        self.thread = threading.Thread(target=self.read_airmar, daemon=True)
    
    def start(self):
        self.thread.start()
    
    def read_airmar(self):
        """Lit en continu les données du capteur Airmar"""
        try:
            with serial.Serial(CONFIG["airmar_port"], CONFIG["airmar_baudrate"], timeout=1) as ser:
                print(f"📡 Capteur Airmar connecté sur {CONFIG['airmar_port']}")
                nmr = NMEAReader(ser)
                
                while not self.stop_event.is_set():
                    try:
                        raw_data, parsed_data = nmr.read()
                        
                        if isinstance(parsed_data, NMEAMessage):
                            if parsed_data.msgID == "MWV":
                                if parsed_data.ref == "R":
                                    self.wind_direction = parsed_data.wind_angle
                                    self.wind_speed = parsed_data.wind_speed
                                elif parsed_data.ref == "T":
                                    self.wind_direction = parsed_data.wind_angle
                                    self.wind_speed = parsed_data.wind_speed
                            
                    except Exception as e:
                        print(f"⚠️ Erreur lecture Airmar: {e}")
                        time.sleep(1)
                        
        except serial.SerialException as e:
            print(f"❌ Erreur connexion Airmar: {e}")
        finally:
            print("📴 Lecteur Airmar arrêté")

class WebSocketManager:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.clients = set()
        self.thread = threading.Thread(target=self.run_websocket, daemon=True)
        
    def start(self):
        self.thread.start()
        
    def run_websocket(self):
        async def handler(websocket, path):
            self.clients.add(websocket)
            try:
                async for message in websocket:
                    # Traiter les messages entrants du PC hôte
                    pass
            finally:
                self.clients.remove(websocket)
                
        async def main():
            async with websockets.serve(handler, "0.0.0.0", 8765):
                await asyncio.Future()  # run forever
                
        asyncio.run(main())

    def broadcast(self, message):
        if self.clients:
            asyncio.run(
                asyncio.wait([client.send(json.dumps(message)) for client in self.clients])
            )

class AutonomousNavigationSystem:
    def __init__(self):
        # 1. Initialisation
        self.config = Config()
        self.lock = threading.Lock()
        
        # 2. Sous-systèmes
        self.sensor_manager = SensorManager()
        self.actuator_manager = ActuatorManager()
        self.navigation_engine = NavigationEngine()
        self.communication = CommunicationManager()
        
        # 3. Initialisation
        self.init_components()
        
    def init_components(self):
        """Initialise tous les sous-systèmes"""
        self.communication.start_websocket_server()
        self.sensor_manager.start_sensors()
        self.actuator_manager.init_gpio()
        
        # Données du bateau
        self.boat_position = None
        self.boat_heading = 0.0
        self.wind_direction = 0.0
        self.wind_speed = 0.0
        self.boat_speed = 0.0
        self.current_tack = 1
        
        # Navigation
        self.waypoints = []
        self.current_waypoint_index = 0
        self.navigation_active = True
        self.last_update_time = time.time()
        
        # Sécurité
        self.safety_triggered = False
        self.last_valid_data_time = time.time()
        
        # Communication
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.websocket_manager = WebSocketManager(self.stop_event)
        self.websocket_manager.start()
        
        # Initialisation des composants
        self.actuator_calculator = PureSailboatActuators()
        self.stop_event = threading.Event()
        self.airmar_reader = AirmarReader(self.stop_event)
        
        # Initialisation GPIO
        self.init_gpio()
        
        # Initialisation des fichiers
        self.init_log_file()
        self.load_waypoints()
        
        # Threads
        self.waypoint_monitor_thread = threading.Thread(
            target=self.monitor_waypoint_changes, 
            daemon=True
        )
        self.waypoint_monitor_thread.start()
        
        # Démarrer le capteur Airmar
        self.airmar_reader.start()
        
        # Thread UDP
        self.udp_thread = threading.Thread(
            target=self.start_udp_listener,
            daemon=True
        )
        self.udp_thread.start()

    def start_udp_listener(self):
        """Reçoit les données UDP des capteurs"""
        def handle_message(data):
            try:
                msg = json.loads(data)
                with self.lock:
                    if msg.get('type') == 'gps_update':
                        self.boat_position = np.array([msg['data']['lat'], msg['data']['lon']])
                    elif msg.get('type') == 'attitude_update':
                        self.boat_heading = msg['data']['yaw']
                    elif msg.get('type') == 'speed_update':
                        self.boat_speed = msg['data']['speed']
            except Exception as e:
                print(f"Erreur traitement UDP: {e}")

        udp_listener(CONFIG["udp_ip"], CONFIG["udp_port"], handle_message)

        
        # Threads
        self.waypoint_monitor_thread = threading.Thread(
            target=self.monitor_waypoint_changes, 
            daemon=True
        )
        self.waypoint_monitor_thread.start()
        
        # Démarrer le capteur Airmar
        self.airmar_reader.start()
        
        # Verrou pour accès thread-safe
        self.udp_thread = threading.Thread(
            target=self.start_udp_listener, 
            daemon=True
        )
        self.udp_thread.start()
        

    def init_gpio(self):
        """Initialisation des GPIO et PWM"""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Configuration des pins
        self.ENA_GV = 32
        self.SENS_GV = 29
        self.ENA_SAF = 33
        self.SENS_SAF = 31
        
        GPIO.setup(self.ENA_GV, GPIO.OUT)
        GPIO.setup(self.SENS_GV, GPIO.OUT)
        GPIO.setup(self.ENA_SAF, GPIO.OUT)
        GPIO.setup(self.SENS_SAF, GPIO.OUT)
        
        # Initialisation PWM
        self.PWM_GV = GPIO.PWM(self.ENA_GV, 100)
        self.PWM_GV.start(0)
        self.PWM_SAF = GPIO.PWM(self.ENA_SAF, 100)
        self.PWM_SAF.start(0)
    
    def init_log_file(self):
        """Initialise le fichier de logs"""
        with open(CONFIG["log_file"], "w") as f:
            f.write("timestamp,lat,lon,heading,speed,wind_dir,wind_speed,wp_index,wp_lat,wp_lon,distance,rudder_angle,sail_angle,pwm_gv,pwm_saf\n")
    
    def load_waypoints(self):
        """Charge les waypoints depuis le fichier JSON"""
        try:
            # Chemin absolu vers le fichier
            file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "gps_points.json")
            with open(file_path, 'r') as f:
                data = json.load(f)
                points = data.get("points", [])
                with self.lock:
                    self.waypoints = [(p['lat'], p['lng']) for p in points]
                    print(f"Waypoints chargés: {len(self.waypoints)} points")
        except FileNotFoundError:
            print("Création d'un fichier gps_points.json vide")
            # Crée le fichier s'il n'existe pas
            os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config"), exist_ok=True)
            with open(file_path, 'w') as f:
                json.dump({"points": []}, f)
            with self.lock:
                self.waypoints = []
        except Exception as e:
            print(f"Erreur lors du chargement des waypoints: {e}")
            with self.lock:
                self.waypoints = []
    
    def monitor_waypoint_changes(self):
        """Surveille les changements dans le fichier de waypoints"""
        last_mtime = 0
        while not self.stop_event.is_set():
            try:
                current_mtime = os.path.getmtime("gps_points.json")
                if current_mtime != last_mtime:
                    self.load_waypoints()
                    last_mtime = current_mtime
            except:
                pass
            time.sleep(1.0)
    
    def update_navigation_data(self):
        """Met à jour les données de navigation"""
        with self.lock:
            self.wind_direction = self.airmar_reader.wind_direction
            self.wind_speed = self.airmar_reader.wind_speed
            self.last_valid_data_time = time.time()
        
        try:
            self.udp_socket.settimeout(0.1)
            data, _ = self.udp_socket.recvfrom(1024)
            decoded_data = json.loads(data.decode('utf-8'))
            
            with self.lock:
                if decoded_data.get('type') == 'gps_update':
                    self.boat_position = np.array([decoded_data['data']['lat'], decoded_data['data']['lon']])
                    self.last_valid_data_time = time.time()
                elif decoded_data.get('type') == 'attitude_update':
                    self.boat_heading = decoded_data['data']['yaw']
                elif decoded_data.get('type') == 'speed_update':
                    self.boat_speed = decoded_data['data']['speed']
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Erreur traitement données UDP: {e}")
    
    def check_safety_conditions(self):
        """Vérifie les conditions de sécurité"""
        if time.time() - self.last_valid_data_time > CONFIG["safety_timeout"]:
            self.trigger_safety(f"Aucune donnée valide reçue depuis {CONFIG['safety_timeout']} secondes")
            return False
        
        if self.boat_position is None:
            self.trigger_safety("Position GPS invalide")
            return False
        
        if self.wind_speed > 20:
            self.trigger_safety(f"Vent trop fort ({self.wind_speed} noeuds)")
            return False
        
        return True
    
    def trigger_safety(self, reason):
        """Active le mode sécurité"""
        if not self.safety_triggered:
            print(f"🚨 Sécurité activée: {reason}")
            self.safety_triggered = True
            self.navigation_active = False
            self.PWM_SAF.ChangeDutyCycle(0)
            self.PWM_GV.ChangeDutyCycle(0)
            self.send_udp_message({
                'type': 'alert',
                'message': f'SECURITE: {reason}',
                'timestamp': datetime.now().isoformat()
            })
    
    def send_udp_message(self, message):
        """Envoie un message UDP"""
        try:
            self.udp_socket.sendto(
                json.dumps(message).encode('utf-8'),
                (CONFIG["udp_ip"], CONFIG["udp_port"])
            )
        except Exception as e:
            print(f"Erreur envoi UDP: {e}")
    
    def distance_to_waypoint(self, waypoint):
        """Calcule la distance au waypoint en mètres"""
        if self.boat_position is None or waypoint is None:
            return float('inf')
            
        lat1, lon1 = np.radians(self.boat_position)
        lat2, lon2 = np.radians(waypoint)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        return 6371000 * c
    
    def control_actuators(self, rudder_angle, sail_angle):
        """Contrôle des actionneurs avec retour des PWM"""
        current_saf, pwm_saf = control_saf(
            rudder_angle, 
            self.PWM_SAF, 
            self.SENS_SAF
        )
        current_gv, pwm_gv = control_gv(
            sail_angle, 
            self.PWM_GV, 
            self.SENS_GV
        )
        
        # Envoi des données de contrôle
        self.send_udp_message({
            'type': 'actuators',
            'rudder_angle': np.degrees(rudder_angle),
            'rudder_pwm': pwm_saf,
            'sail_angle': np.degrees(sail_angle),
            'sail_pwm': pwm_gv,
            'timestamp': datetime.now().isoformat()
        })
        
        return current_gv, current_saf, pwm_gv, pwm_saf
    
    def start_navigation(self):
        """Active la navigation autonome"""
        if not self.waypoints:
            print("Aucun waypoint défini!")
            return
            
        with self.lock:
            self.navigation_active = True
            self.safety_triggered = False
            self.current_waypoint_index = 0
            print(f"🚢 Début de navigation vers {len(self.waypoints)} waypoints")
            self.send_udp_message({
                'type': 'navigation_start',
                'waypoints': len(self.waypoints),
                'timestamp': datetime.now().isoformat()
            })
    
    def stop_navigation(self):
        """Arrête la navigation autonome"""
        with self.lock:
            self.navigation_active = False
            print("⛵ Navigation arrêtée")
            self.send_udp_message({
                'type': 'navigation_stop',
                'timestamp': datetime.now().isoformat()
            })
    
    def navigate(self):
        """Exécute une itération de navigation"""
        if not self.navigation_active or not self.waypoints:
            return
        
        if not self.check_safety_conditions():
            return
            
        with self.lock:
            current_waypoint = self.waypoints[self.current_waypoint_index]
            distance = self.distance_to_waypoint(current_waypoint)
            
            if distance < CONFIG["waypoint_tolerance"]:
                print(f"🎯 Waypoint {self.current_waypoint_index + 1} atteint!")
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index >= len(self.waypoints):
                    print("🏁 Tous les waypoints atteints!")
                    self.stop_navigation()
                    return
            
            target_waypoint = self.waypoints[self.current_waypoint_index]
            line_start = np.array(self.boat_position) if self.boat_position is not None else np.array([0, 0])
            line_end = np.array(target_waypoint)
            
            rudder_angle, sail_angle = self.actuator_calculator.compute_actuators(
                boat_pos=line_start,
                boat_heading=np.radians(self.boat_heading),
                wind_dir=np.radians(self.wind_direction),
                line_start=line_start,
                line_end=line_end,
                current_tack=self.current_tack
            )
            
            current_gv, current_saf, pwm_gv, pwm_saf = self.control_actuators(rudder_angle, sail_angle)
            
            self.log_navigation_data(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)
            self.display_status(distance, rudder_angle, sail_angle, pwm_gv, pwm_saf)
    
    def log_navigation_data(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """Enregistre les données de navigation"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return
            
        current_wp = self.waypoints[self.current_waypoint_index]
        
        log_entry = (
            f"{datetime.now().isoformat()},"
            f"{self.boat_position[0] if self.boat_position else 'NaN'},"
            f"{self.boat_position[1] if self.boat_position else 'NaN'},"
            f"{self.boat_heading},"
            f"{self.boat_speed},"
            f"{self.wind_direction},"
            f"{self.wind_speed},"
            f"{self.current_waypoint_index},"
            f"{current_wp[0]},"
            f"{current_wp[1]},"
            f"{distance},"
            f"{np.degrees(rudder_angle)},"
            f"{np.degrees(sail_angle)},"
            f"{pwm_gv},"
            f"{pwm_saf}\n"
        )
        
        with open(CONFIG["log_file"], "a") as f:
            f.write(log_entry)
    
    def display_status(self, distance, rudder_angle, sail_angle, pwm_gv, pwm_saf):
        """Affiche le statut de navigation"""
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=== SYSTÈME DE NAVIGATION AUTONOME ===")
        print(f"Position: {self.boat_position}")
        print(f"Cap: {self.boat_heading:.1f}° | Vitesse: {self.boat_speed:.1f} km/h")
        print(f"Vent: {self.wind_direction:.1f}° ({self.wind_speed:.1f} kt) | Bord: {'Tribord' if self.current_tack == 1 else 'Bâbord'}")
        print(f"\nWaypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        print(f"Distance: {distance:.1f}m | Tolérance: {CONFIG['waypoint_tolerance']}m")
        print(f"\nCommandes:")
        print(f"GV: {np.degrees(sail_angle):.1f}° (PWM: {pwm_gv}%)")
        print(f"SAF: {np.degrees(rudder_angle):.1f}° (PWM: {pwm_saf}%)")
        print(f"\nSécurité: {'🚨 ACTIVE' if self.safety_triggered else '✅ OK'}")

    def cleanup(self):
        """Nettoyage des ressources"""
        self.PWM_GV.stop()
        self.PWM_SAF.stop()
        from prop_saf_e_gv_V2 import cleanup as cleanup_adc
        cleanup_adc()  # Nettoie l'ADC
        GPIO.cleanup()
        self.udp_socket.close()

def main():
    # Initialisation du système
    nav_system = AutonomousNavigationSystem()
    
    # Démarrer la navigation automatiquement si des waypoints existent
    if nav_system.waypoints:
        nav_system.start_navigation()
    
    try:
        # Boucle principale
        while not nav_system.stop_event.is_set():
            nav_system.update_navigation_data()
            nav_system.navigate()
            time.sleep(CONFIG["control_interval"])
    
    except KeyboardInterrupt:
        print("\nArrêt du système demandé...")
    finally:
        nav_system.stop_navigation()
        nav_system.stop_event.set()
        nav_system.cleanup()
        print("Système arrêté proprement")

if __name__ == "__main__":
    main()