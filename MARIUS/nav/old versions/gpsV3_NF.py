import folium

# Création de la carte
latitude, longitude = 43.095, 6.137
m = folium.Map(location=[latitude, longitude], zoom_start=16, tiles="OpenStreetMap")

# JavaScript pour gérer l'ajout/suppression des points avec affichage
click_js = """
// Stocke les marqueurs avec leurs noms
var markers = {};

// Désactiver le menu contextuel sur toute la page (évite le menu Firefox)
document.addEventListener("contextmenu", function(e) {
    e.preventDefault();
}, false);

// Ajouter une liste des points affichée sur la carte
var infoDiv = document.createElement("div");
infoDiv.style.position = "absolute";
infoDiv.style.top = "10px";
infoDiv.style.right = "10px";
infoDiv.style.padding = "10px";
infoDiv.style.background = "white";
infoDiv.style.border = "1px solid black";
infoDiv.style.zIndex = "1000";
infoDiv.innerHTML = "<b>Points ajoutés :</b><ul id='point-list'></ul>";
document.body.appendChild(infoDiv);

// Fonction pour mettre à jour la liste des points
function updateList() {
    var list = document.getElementById("point-list");
    list.innerHTML = "";
    for (var key in markers) {
        var li = document.createElement("li");
        li.innerText = markers[key].name + " (" + key + ")";
        list.appendChild(li);
    }
}

// Fonction pour ajouter ou supprimer un marqueur
function onMapRightClick(e) {
    var latlngStr = e.latlng.lat.toFixed(6) + ", " + e.latlng.lng.toFixed(6);
    
    // Vérifier si un marqueur est déjà présent à cet endroit
    if (markers[latlngStr]) {
        // Supprimer le marqueur
        map.removeLayer(markers[latlngStr].marker);
        delete markers[latlngStr];
    } else {
        // Ajouter un nouveau marqueur
        var name = prompt("Nom du point : " + latlngStr, "Point");
        if (name) {
            var marker = L.marker(e.latlng).bindPopup(name).addTo(map);
            markers[latlngStr] = { marker: marker, name: name };
        }
    }

    updateList();  // Rafraîchir la liste affichée
}

// Attacher l'événement au clic droit
map.on('contextmenu', onMapRightClick);

// Debugging: Log pour vérifier que le script est bien chargé
console.log("✅ JavaScript chargé avec succès !");
"""

# Ajout du script dans la carte Folium
m.get_root().html.add_child(folium.Element(f"<script>{click_js}</script>"))

# Sauvegarde de la carte
m.save("carte_interactive.html")
print("✅ Carte interactive mise à jour avec affichage des points ! Ouvrez 'carte_interactive.html'.")
