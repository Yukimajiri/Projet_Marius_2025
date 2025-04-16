import serial
from pynmeagps import NMEAReader, NMEAMessage

def read_serial_with_pynmeagps(port, baudrate):
    """Lit les données NMEA depuis le port série et les traite avec pynmeagps."""
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"Lecture des données série sur {port} (Ctrl+C pour arrêter)")

            # Création du lecteur NMEA
            nmea_reader = NMEAReader(ser)

            while True:
                # Lire une trame depuis le port série
                try:
                    (raw_data, parsed_data) = nmea_reader.read()
                    
                    # Afficher la trame brute et les données décodées
                    print(f"Trame brute : {raw_data}")
                    if isinstance(parsed_data, NMEAMessage):
                        print(f"Trame décodée : {parsed_data}")
                        # Vous pouvez accéder directement aux champs :
                        if parsed_data.msgID == "GGA":  # Exemple pour GPGGA
                            print(f"Latitude: {parsed_data.lat}, Longitude: {parsed_data.lon}")
                        elif parsed_data.msgID == "VTG":  # Exemple pour GPVTG
                            print(f"Vitesse sol: {parsed_data.spd_over_grnd_kmph} km/h")
                except Exception as e:
                    print(f"Erreur de lecture ou de parsing : {e}")
    except serial.SerialException as e:
        print(f"Erreur de port série : {e}")
    except KeyboardInterrupt:
        print("\nArrêt de la lecture série.")

# Configuration du port série
PORT = '/dev/ttyUSB0'  # Remplacez par le port série utilisé
BAUDRATE = 4800  # Assurez-vous que cela correspond au périphérique

# Démarrage de la lecture série
read_serial_with_pynmeagps(PORT, BAUDRATE)  