import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

def tracking_function(m, theta, psi, a, b, r, gamma_inf, epsilon, s_r_max, q, correction, dt, z):
    e = np.cross((b - a) / np.linalg.norm(b - a), m - a)
    q = np.sign(e) if abs(e) > r / 2 else 0
    phi = np.arctan2(b[1] - a[1], b[0] - a[0])
    alpha = (correction / (e * dt)) if e != 0 else 0
    z = z + alpha * dt * e
    theta_etoile = phi - ((2 * gamma_inf) / np.pi) * np.array((e + z) / r)
    
    if (np.cos(psi - theta_etoile) + np.cos(epsilon) < 0) or ((abs(e) < epsilon and (np.cos(psi - phi) + np.cos(epsilon) < 0))):
        theta_conj = np.pi + psi - q * epsilon
    else:
        theta_conj = theta_etoile

    if np.cos(theta - theta_conj) >= 0:
        delta_r = s_r_max * np.sin(theta - theta_conj)
    else:
        delta_r = s_r_max * np.sign(np.sin(theta - theta_conj))

    s_s_max = (np.pi / 2) * ((np.cos(psi - theta) + 1) / 2)
    print(delta_r, s_s_max)
    return delta_r, s_s_max, q, z

gammainf1 = (np.pi) / 4
epsilon1 = (np.pi) / 3
s_r_max1 = (np.pi) / 4
r1 = 50
q1 = 0
correction1 = 1
dt1 = 1

def update(frame):
    global position, theta, q, z, wind_speed, boat_speed
    
    psi = wind_speed_slider.val  # Mise à jour de la vitesse du vent
    speed = boat_speed_slider.val  # Mise à jour de la vitesse du bateau
    
    direction = waypoint - position
    a = position
    b = position - direction / np.linalg.norm(direction) * 10
    
    delta_r, s_s_max, q, z = tracking_function(position, theta, psi, a, b, r1, gammainf1, epsilon1, s_r_max1, q, correction1, dt1, z)
    
    theta += delta_r * dt1
    position += np.array([np.cos(theta), np.sin(theta)]) * dt1 * speed  # Ajustement de la vitesse
    history.append(position.copy())
    
    if np.linalg.norm(position - waypoint) < waypoint_radius:
        ani.event_source.stop()
    
    history_np = np.array(history)
    boat_path.set_data(history_np[:, 0], history_np[:, 1])
    return boat_path,


position = np.array([0.0, 0.0])
theta = 0
waypoint = np.array([-300.0, 300.0])
waypoint_radius = 10
q = 0
z = 0
history = []

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.25)
ax.set_xlim(-300, 300)
ax.set_ylim(-300, 300)
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_title("Boat Navigation Simulation")
ax.grid()

boat_path, = ax.plot([], [], label="Boat Path")
ax.scatter([waypoint[0]], [waypoint[1]], color='red', label="Waypoint")
circle = plt.Circle(waypoint, waypoint_radius, color='r', fill=False, linestyle='dashed')
ax.add_patch(circle)
ax.legend()

axcolor = 'lightgoldenrodyellow'
ax_speed = plt.axes([0.15, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_wind = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor=axcolor)
boat_speed_slider = Slider(ax_speed, 'Boat Speed', 0.1, 10, valinit=1.0, valstep=0.1)
wind_speed_slider = Slider(ax_wind, 'Wind Speed', -np.pi, np.pi*10, valinit=np.pi/4, valstep=0.1)

ani = animation.FuncAnimation(fig, update, frames=500, interval=50, blit=False)
plt.show()
