import asyncio
import websockets
import json
from datetime import datetime
import os

"""
Serveur WebSocket pour la gestion de points GPS en temps réel.

Fonctionnalités principales :
- Recevoir des points GPS depuis des clients
- Diffuser les mises à jour à tous les clients connectés
- Sauvegarder les points dans un fichier JSON
- Gérer les connexions/déconnexions des clients
"""

# Liste globale pour stocker les points GPS
points = []
# Ensemble pour garder une trace des clients connectés
connected_clients = set()
# Nom du fichier pour la sauvegarde des données
JSON_FILE = "gps_points.json"

def save_points_to_json():
    """
    Sauvegarde la liste des points dans un fichier JSON.
    Inclut un horodatage pour savoir quand la sauvegarde a été faite.
    """
    data = {
        "timestamp": datetime.now().isoformat(),
        "points": points
    }
    with open(JSON_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"💾 Points sauvegardés dans {JSON_FILE}")

async def handle_client(websocket, path=None):
    """
    Gère la connexion d'un client WebSocket.
    
    Args:
        websocket: La connexion WebSocket du client
        path: Le chemin de la requête (optionnel)
    """
    global points
    # Ajouter le nouveau client à la liste des clients connectés
    connected_clients.add(websocket)
    try:
        print("🟢 Nouveau client connecté")

        # Envoyer l'état actuel des points au nouveau client
        await websocket.send(json.dumps({"type": "update", "points": points}))

        # Écouter les messages entrants de ce client
        async for message in websocket:
            print(f"📩 Message reçu: {message}")
            data = json.loads(message)

            # Traitement des différents types de messages
            if data["type"] == "add_point":
                # Ajout d'un nouveau point GPS
                new_point = data["point"]
                points.append(new_point)
                # Diffusion de la mise à jour à tous les clients
                await broadcast({"type": "update", "points": points})
                save_points_to_json()  # Sauvegarde après ajout
                print(f"✅ Point ajouté: {new_point}")

            elif data["type"] == "clear_points":
                # Suppression de tous les points
                points = []
                await broadcast({"type": "update", "points": points})
                save_points_to_json()  # Sauvegarde après suppression
                print("🗑️ Liste des points effacée")

            elif data["type"] == "remove_point":
                # Suppression d'un point spécifique par son index
                index = data["index"]
                if 0 <= index < len(points):
                    deleted_point = points.pop(index)
                    await broadcast({"type": "update", "points": points})
                    save_points_to_json()  # Sauvegarde après suppression
                    print(f"❌ Point supprimé: {deleted_point}")

    except websockets.exceptions.ConnectionClosed:
        print("🔴 Client déconnecté")
    finally:
        # Nettoyage: retirer le client de la liste des connectés
        connected_clients.remove(websocket)

async def broadcast(message):
    """
    Envoie un message à tous les clients connectés.
    
    Args:
        message: Le message à diffuser (sera converti en JSON)
    """
    if connected_clients:
        await asyncio.gather(
            *(client.send(json.dumps(message)) for client in connected_clients)
        )

async def main():
    """
    Fonction principale qui initialise le serveur WebSocket.
    """
    global points
    # Charger les points existants au démarrage du serveur
    if os.path.exists(JSON_FILE):
        try:
            with open(JSON_FILE, 'r') as f:
                data = json.load(f)
                points = data.get("points", [])
                print(f"📂 Chargement de {len(points)} points depuis {JSON_FILE}")
        except Exception as e:
            print(f"⚠️ Erreur lors du chargement du fichier JSON: {e}")

    # Démarrer le serveur WebSocket
    print("✅ Serveur WebSocket démarré sur ws://localhost:8765")
    async with websockets.serve(handle_client, "localhost", 8765):
        # Maintenir le serveur en fonctionnement
        await asyncio.Future()

if __name__ == "__main__":
    # Point d'entrée principal - démarrer le serveur
    asyncio.run(main())