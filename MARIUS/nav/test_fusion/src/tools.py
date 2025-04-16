# udp_utils.py
import socket
import json

BUFFER_SIZE = 1024  # Taille du buffer

def udp_listener(ip, port, callback): # callback est une fonction passée en argument
    """Écoute les messages UDP entrants et exécute un callback sur les données reçues."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"🖥️ Serveur UDP en écoute sur {ip}:{port}")

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            decoded_data = data.decode('utf-8')
            parsed_data = json.loads(decoded_data)
            print(f"📥 Données reçues de {addr}: {parsed_data}")
            callback(decoded_data)
        except json.JSONDecodeError:
            print("❌ Erreur : données mal formatées")
        except Exception as e:
            print(f"❌ Erreur inattendue : {e}")

def udp_forwarder(data, destinations):
    """
    Envoie des données UDP vers plusieurs destinations.
    
    :param data: Données à envoyer (string ou dictionnaire)
    :param destinations: Liste de tuples (IP, Port)
    """
    try:
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        json_data = data if isinstance(data, str) else json.dumps(data)
        encoded_data = json_data.encode('utf-8')

        for ip, port in destinations:
            send_sock.sendto(encoded_data, (ip, port))
            print(f"📤 Données envoyées à {ip}:{port} : {data}")

    except Exception as e:
        print(f"❌ Erreur lors de l'envoi des données : {e}")


# Fonction de réception TCP (tourne en boucle infinie)
def tcp_listener(TCP_IP,TCP_PORT):
    global stop_flag
    serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serveur.bind((TCP_IP, TCP_PORT))
    serveur.listen()

    print(f"Serveur TCP en écoute sur {TCP_IP}:{TCP_PORT}...")

    while True:
        try:
            client, infosclient = serveur.accept()
            request = client.recv(1024)
            message = request.decode('utf-8').strip()
            
            print(f"Message TCP reçu : {message}")
            print(f"IP client connecté : {infosclient[0]}")
            
            if message.lower() == "stop":
                print("Message d'arrêt reçu, arrêt de l'émission UDP.")
                stop_flag = True  # Active le flag pour stopper l'UDP
            
            client.close()
        except Exception as e:
            print(f"Erreur TCP : {e}")
            break

    serveur.close()
    print("Serveur TCP fermé.")