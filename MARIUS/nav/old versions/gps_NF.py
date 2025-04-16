import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import numpy as np

# Liste des points GPS [(lat, lon, nom)]
points_gps = []
text_annotations = []  # Stocker les annotations pour pouvoir les modifier

def on_click(event):
    """Gère le clic de la souris"""
    if event.xdata is None or event.ydata is None:
        return
    
    lat, lon = m(event.xdata, event.ydata, inverse=True)

    if event.button == 1:  # Clic gauche → Ajouter un point
        nom = ""  # On attend l'entrée de texte
        points_gps.append([lat, lon, nom])
        update_map()

    elif event.button == 3:  # Clic droit → Supprimer le point le plus proche
        if points_gps:
            distances = [np.sqrt((p[0] - lat) ** 2 + (p[1] - lon) ** 2) for p in points_gps]
            idx = np.argmin(distances)
            del points_gps[idx]
            update_map()

def on_key(event):
    """Ajoute un nom au dernier point ajouté ou ferme la carte si 'q' est pressé"""
    if event.key == 'q':
        plt.close()
    elif points_gps and event.key.isalnum():  # Ajouter du texte au dernier point
        points_gps[-1][2] += event.key
        update_map()

def update_map():
    """Met à jour la carte avec les points actuels"""
    plt.clf()

    # Fond de carte en image satellite
    m.bluemarble(scale=10)

    # Supprimer les anciennes annotations
    for txt in text_annotations:
        txt.remove()
    text_annotations.clear()

    # Affichage des points GPS
    for lat, lon, nom in points_gps:
        x, y = m(lon, lat)
        plt.scatter(x, y, c="red", marker="o", s=100)
        txt = plt.text(x, y, f" {nom}", fontsize=12, color="white", 
                       bbox=dict(facecolor='black', alpha=0.5))
        text_annotations.append(txt)

    plt.draw()

# Création de la carte satellite avec Basemap
fig, ax = plt.subplots(figsize=(10, 8))
m = Basemap(projection="mill", llcrnrlat=42.8, urcrnrlat=43.3, llcrnrlon=5.9, urcrnrlon=6.4, resolution='h')

# Affichage initial
update_map()

# Ajout des événements
fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("key_press_event", on_key)

plt.show()
