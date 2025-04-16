import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import numpy as np
import json
import os
import pygeodesy as pg
# doc du module formy de la librairie pygeodesy (module formy);
# https://mrjean1.github.io/PyGeodesy/docs/pygeodesy.formy-module.html
from cartopy.io.img_tiles import GoogleTiles

# Paramètres
#lon_init, lat_init = 6.00977, 43.13504 # Coordonnées de SEATECH, parking
#lon_init, lat_init = 6.0124, 43.1365   # Coordonnées de SEATECH, bassin
lon_init, lat_init = 6.15045, 43.0464 # Coordonées bergerie
map_radius = 0.001 # Radius of the map around central point (in nautic miles)

# Initialisation
WP_lat = []  # Liste pour stocker les latitudes des points cliqués
WP_lon = []  # Liste pour stocker les longitudes des points cliqués


## Fonctions ##
def onclick(event):
    if event.inaxes is not None:
        lon, lat = event.xdata, event.ydata
        print(f"Latitude: {lat}, Longitude: {lon}")
        WP_lat.append(lat)
        WP_lon.append(lon)

# Clean old figures
plt.close('all')

# Créer une figure et ajouter une carte
projection = ccrs.PlateCarree()
fig = plt.figure(figsize=(9, 9))
ax = plt.axes(projection=projection)

# Ajouter des caractéristiques à la carte
imagery = GoogleTiles(style="satellite")
ax.add_image(imagery, 20)
ax.set_title("Selection WP")
ax.set_extent([lon_init - map_radius, lon_init + map_radius, lat_init - map_radius, lat_init + map_radius]) #Zoom bassin
#ax.set_extent([lon_init - 0.001, lon_init + 0.001, lat_init - 0.001, lat_init + 0.001]) #Zoom dparking

# Connecter l'événement de clic de souris à la fonction de gestion
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()

# Construire le chemin complet du fichier JSON
script_dir = os.path.dirname(os.path.abspath(__file__))
json_file_path = os.path.join(script_dir, 'Waypoints_data.json')

# Exporter les listes WP_lat et WP_lon dans un fichier JSON
data = {'WP_lat': WP_lat, 'WP_lon': WP_lon}
with open(json_file_path, 'w') as outfile:
    json.dump(data, outfile)

# Calcul des caps et distances entre les WP
caps = []
distances = []

for i in range(1, len(WP_lat)):
    distance = pg.haversine(WP_lat[i-1], WP_lon[i-1], WP_lat[i], WP_lon[i])
    bearing =  pg.compassAngle(WP_lat[i-1], WP_lon[i-1], WP_lat[i], WP_lon[i]) 
    caps.append(bearing)
    distances.append(distance)

# Tracer les WP GPS sur la carte
fig = plt.figure(figsize=(9, 9))
ax = plt.axes(projection=projection)
imagery = GoogleTiles(style="satellite")
ax.add_image(imagery, 20)
ax.set_title("Route")
ax.set_extent([np.nanmin(WP_lon)-map_radius, np.nanmax(WP_lon)+map_radius, np.nanmin(WP_lat)-map_radius, np.nanmax(WP_lat)+map_radius])
ax.plot(WP_lon, WP_lat, linestyle="-", marker='+', markersize=10, color='red', transform=projection, label='Trajectoire Blend')

# Afficher les caps et distances
for i in range(len(caps)):
    plt.text(WP_lon[i], WP_lat[i], f"Cap: {caps[i]:.2f}\nDistance: {distances[i]:.2f} m",color='cyan', transform=projection)

# Save et Affiche la figure
map_path = os.path.join(script_dir,'Mission_plan.png')
plt.savefig(map_path)
plt.show()
