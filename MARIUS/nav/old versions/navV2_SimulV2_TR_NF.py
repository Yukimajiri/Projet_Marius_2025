import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button

# Simulation parameters
dt = 1
total_time = 500
boat_speed = 0.5  # Réduction de la vitesse initiale
wind_speed = 0.5
wind_angle = np.pi / 4  # Direction initiale du vent

# Initialisation des positions
position = np.array([0.0, 0.0])
theta = 0
waypoint = np.array([-100.0, 100.0])
waypoint_radius = 10

# Historique pour l'affichage
history = []

def update(frame):
    global position, theta, wind_angle
    
    # Calcul du vent en tant que vecteur
    V_wind = wind_speed * np.array([np.cos(wind_angle), np.sin(wind_angle)])
    
    # Ajustement de la vitesse du bateau en fonction du vent
    wind_effect = np.dot(V_wind, [np.cos(theta), np.sin(theta)])
    speed = boat_speed + wind_effect  # Influence du vent sur la vitesse
    speed = max(0.2, min(1.0, speed))  # Contraintes de vitesse
    
    # Calcul du mouvement
    direction = waypoint - position
    a = position
    b = position - direction / np.linalg.norm(direction) * 10
    
    if np.linalg.norm(position - waypoint) < waypoint_radius:
        ani.event_source.stop()
    
    position[:] += np.array([np.cos(theta), np.sin(theta)]) * dt * speed
    history.append(position.copy())
    
    # Mise à jour du graphique
    line.set_data(*zip(*history))
    return line,

def update_wind(val):
    global wind_speed
    wind_speed = wind_slider.val

def change_wind_angle(event):
    global wind_angle
    wind_angle += np.pi / 4  # Change la direction du vent de 45°

# Création de la figure
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(-120, 20)
ax.set_ylim(-10, 120)
ax.set_title("Boat Navigation Simulation")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.grid()

# Tracé des éléments graphiques
line, = ax.plot([], [], label="Boat Path")
ax.scatter(*waypoint, color='red', label="Waypoint")
circle = plt.Circle(waypoint, waypoint_radius, color='r', fill=False, linestyle='dashed')
ax.add_patch(circle)
ax.legend()

# Ajout du slider pour la vitesse du vent
ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03])
wind_slider = Slider(ax_slider, 'Wind Speed', 0, 2, valinit=wind_speed)
wind_slider.on_changed(update_wind)

# Bouton circulaire pour changer la direction du vent
ax_button = plt.axes([0.8, 0.02, 0.1, 0.1])
wind_button = Button(ax_button, "↻")
wind_button.on_clicked(change_wind_angle)

# Animation
ani = animation.FuncAnimation(fig, update, frames=total_time, interval=50, blit=True)
plt.show()
