import json
import asyncio
import websockets
import os


fichier_json = "points.json"

def reset_json():
    """Réinitialise le fichier JSON en vidant la liste des points."""
    with open(fichier_json, "w") as f:
        json.dump([], f)
    print("Fichier JSON réinitialisé.")

# Réinitialisation au lancement du programme
reset_json()

# Liste des connexions clients
connected_clients = set()

# Charger les points au démarrage
def load_points():
    global points
    if os.path.exists(fichier_json):
        with open(fichier_json, "r") as f:
            try:
                points = json.load(f)
                print("📂 Points chargés depuis le fichier JSON")
            except json.JSONDecodeError:
                points = []
                print("⚠️ Fichier JSON corrompu, initialisation à vide.")

# Sauvegarder les points à chaque modification
def save_points():
    with open(fichier_json, "w") as f:
        json.dump(points, f, indent=4)
    print("💾 Points sauvegardés dans points.json")

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
                save_points()  # Sauvegarde après ajout
                await broadcast({"type": "update", "points": points})
                print(f"✅ Point ajouté: {new_point}")

            elif data["type"] == "clear_points":
                points = []
                save_points()  # Sauvegarde après suppression
                await broadcast({"type": "update", "points": points})
                print("🗑️ Liste des points effacée")

            elif data["type"] == "remove_point":
                index = data["index"]
                if 0 <= index < len(points):
                    deleted_point = points.pop(index)
                    save_points()  # Sauvegarde après suppression
                    await broadcast({"type": "update", "points": points})
                    print(f"❌ Point supprimé: {deleted_point}")
            elif data["type"] == "reset_points":
                points = []
                if os.path.exists(fichier_json):
                    os.remove(fichier_json)
                await broadcast({"type": "update", "points": points})
                print("🔄 Réinitialisation complète (JSON supprimé)")


    except websockets.exceptions.ConnectionClosed:
        print("🔴 Client déconnecté")
    finally:
        connected_clients.remove(websocket)

async def broadcast(message):
    """Envoie un message à tous les clients connectés."""
    if connected_clients:
        await asyncio.gather(*(client.send(json.dumps(message)) for client in connected_clients))

async def main():
    load_points()  # Charger les points au démarrage
    print("🚀 Serveur WebSocket démarré sur ws://localhost:8765")
    async with websockets.serve(handle_client, "localhost", 8765):
        await asyncio.Future()  # Garde le serveur en marche

if __name__ == "__main__":
    asyncio.run(main())
