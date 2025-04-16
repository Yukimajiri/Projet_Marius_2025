import folium

# Création de la carte
latitude, longitude = 43.095, 6.137
m = folium.Map(location=[latitude, longitude], zoom_start=16, tiles="OpenStreetMap")

# Ajout de JavaScript pour capturer les clics
click_js = """
function onMapClick(e) {
    var name = prompt("Nom du point :");  // Fenêtre pour nommer le point
    if (name) {
        var marker = L.marker(e.latlng).bindPopup(name).addTo(map);
    }
}
map.on('click', onMapClick);
"""

m.get_root().html.add_child(folium.Element(f"<script>{click_js}</script>"))

# Sauvegarde de la carte
m.save("carte_interactive.html")
print("✅ Carte interactive générée ! Ouvrez 'carte_interactive.html'.")
