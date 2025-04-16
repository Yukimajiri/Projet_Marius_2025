import asyncio
import websockets
import json
from datetime import datetime
import os

# Liste des points et clients connectés
points = []
connected_clients = set()
JSON_FILE = "gps_points.json"

def save_points_to_json():
    """Sauvegarde les points dans un fichier JSON avec horodatage"""
    data = {
        "timestamp": datetime.now().isoformat(),
        "points": points
    }
    with open(JSON_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"💾 Points sauvegardés dans {JSON_FILE}")

async def handle_client(websocket, path=None):
    global points
    connected_clients.add(websocket)
    try:
        print("🟢 Nouveau client connecté")

        # Envoyer les points actuels au nouveau client
        await websocket.send(json.dumps({"type": "update", "points": points}))

        async for message in websocket:
            print(f"📩 Message reçu: {message}")
            data = json.loads(message)

            if data["type"] == "add_point":
                new_point = data["point"]
                points.append(new_point)
                await broadcast({"type": "update", "points": points})
                save_points_to_json()  # Sauvegarde après ajout
                print(f"✅ Point ajouté: {new_point}")

            elif data["type"] == "clear_points":
                points = []
                await broadcast({"type": "update", "points": points})
                save_points_to_json()  # Sauvegarde après suppression
                print("🗑️ Liste des points effacée")

            elif data["type"] == "remove_point":
                index = data["index"]
                if 0 <= index < len(points):
                    deleted_point = points.pop(index)
                    await broadcast({"type": "update", "points": points})
                    save_points_to_json()  # Sauvegarde après suppression
                    print(f"❌ Point supprimé: {deleted_point}")

    except websockets.exceptions.ConnectionClosed:
        print("🔴 Client déconnecté")
    finally:
        connected_clients.remove(websocket)

async def broadcast(message):
    """Envoie un message à tous les clients connectés."""
    if connected_clients:
        await asyncio.gather(*(client.send(json.dumps(message)) for client in connected_clients))

async def main():
    # Charger les points existants au démarrage
    global points
    if os.path.exists(JSON_FILE):
        try:
            with open(JSON_FILE, 'r') as f:
                data = json.load(f)
                points = data.get("points", [])
                print(f"📂 Chargement de {len(points)} points depuis {JSON_FILE}")
        except Exception as e:
            print(f"⚠️ Erreur lors du chargement du fichier JSON: {e}")

    print("✅ Serveur WebSocket démarré sur ws://localhost:8765")
    async with websockets.serve(handle_client, "localhost", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())