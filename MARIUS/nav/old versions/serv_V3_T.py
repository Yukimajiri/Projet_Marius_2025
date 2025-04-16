import asyncio
import websockets
import json
from datetime import datetime
import os
from haversine import haversine

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
        await websocket.send(json.dumps({"type": "update", "points": points}))

        async for message in websocket:
            print(f"📩 Message reçu: {message}")
            data = json.loads(message)

            if data["type"] == "add_point":
                new_point = data["point"]
                points.append(new_point)
                await broadcast({"type": "update", "points": points})
                save_points_to_json()
                print(f"✅ Point ajouté: {new_point}")

            elif data["type"] == "clear_points":
                points = []
                await broadcast({"type": "update", "points": points})
                save_points_to_json()
                print("🗑️ Liste des points effacée")

            elif data["type"] == "remove_point":
                index = data["index"]
                if 0 <= index < len(points):
                    deleted_point = points.pop(index)
                    await broadcast({"type": "update", "points": points})
                    save_points_to_json()
                    print(f"❌ Point supprimé: {deleted_point}")
                    
            elif data["type"] == "gps_position":
                current_pos = data["position"]
                print(f"📍 Position GPS reçue: {current_pos}")
                
                to_remove = []
                for i, point in enumerate(points):
                    distance = haversine(
                        (current_pos["lat"], current_pos["lng"]),
                        (point["lat"], point["lng"]),
                        unit='m'
                    )
                    if distance < 10:  # 10 meters radius
                        to_remove.append(i)
                
                for i in sorted(to_remove, reverse=True):
                    deleted_point = points.pop(i)
                    print(f"❌ Point supprimé (proximité): {deleted_point}")
                
                if to_remove:
                    await broadcast({"type": "update", "points": points})
                    save_points_to_json()

    except websockets.exceptions.ConnectionClosed:
        print("🔴 Client déconnecté")
    finally:
        connected_clients.remove(websocket)

async def broadcast(message):
    """Envoie un message à tous les clients connectés."""
    if connected_clients:
        await asyncio.gather(*(client.send(json.dumps(message)) for client in connected_clients))

async def main():
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