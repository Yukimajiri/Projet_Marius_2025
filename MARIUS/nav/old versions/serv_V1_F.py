import asyncio
import websockets
import json

# Liste des points et clients connectés
points = []
connected_clients = set()

async def handle_client(websocket, path=None):

    global points
    connected_clients.add(websocket)
    try:
        print("🟢 Nouveau client connecté")

        # Envoyer les points actuels au nouveau client
        await websocket.send(json.dumps({"type": "update", "points": points}))

        async for message in websocket:
            print(f"📩 Message reçu: {message}")  # Ajout d'un log pour vérifier la réception
            data = json.loads(message)

            if data["type"] == "add_point":
                new_point = data["point"]
                points.append(new_point)
                await broadcast({"type": "update", "points": points})
                print(f"✅ Point ajouté: {new_point}")

            elif data["type"] == "clear_points":
                points = []
                await broadcast({"type": "update", "points": points})
                print("🗑️ Liste des points effacée")

            elif data["type"] == "remove_point":
                index = data["index"]
                if 0 <= index < len(points):
                    deleted_point = points.pop(index)
                    await broadcast({"type": "update", "points": points})
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
    print("✅ Serveur WebSocket démarré sur ws://localhost:8765")
    async with websockets.serve(handle_client, "localhost", 8765):
        await asyncio.Future()  # Garde le serveur en marche

if __name__ == "__main__":
    asyncio.run(main())
