import math
import numpy as np
import matplotlib.pyplot as plt

def tracking_function(m, theta, psi, a, b,r, gamma_inf, epsilon, s_r_max, q, correction, dt, z):
#    e = np.linalg.det(np.array([[b - a] / np.linalg.norm(b - a), m - a]))
    e = np.cross((b - a) / np.linalg.norm(b - a), m - a)
    q = np.sign(e) if abs(e) > r / 2 else 0
    phi = np.arctan2(b[1] - a[1], b[0] - a[0])
#    theta_etoile = phi - (2 * gamma_inf / np.pi) * np.arctan(e / r)
    alpha = (correction/(e*dt)) if e != 0 else 0
    z = z + alpha * dt * e
    theta_etoile = phi - ((2*gamma_inf)/np.pi) * np.array((e+z)/r)    

#    if e >50:
#        'on change de ligne'

#    print(f"psi = {psi}, theta_etoile = {theta_etoile}, epsilon = {epsilon}")
#    print(f"cos(psi - theta_etoile) + cos(epsilon) = {np.cos(psi - theta_etoile) + np.cos(epsilon)}")
#    print(f"cos(psi - phi) + cos(epsilon) = {np.cos(psi - phi) + np.cos(epsilon)}")
#    print(f"e = {e}, abs(e) < epsilon: {abs(e) < epsilon}")

    if (np.cos(psi - theta_etoile) + np.cos(epsilon) < 0) or ((abs(e) < epsilon and (np.cos(psi - phi) + np.cos(epsilon) < 0))):
        theta_conj = np.pi + psi - q * epsilon
    else:
        theta_conj = theta_etoile

    if np.cos(theta - theta_conj) >= 0:
        delta_r = s_r_max * np.sin(theta - theta_conj)
    else:
        delta_r = s_r_max * np.sign(np.sin(theta - theta_conj))

    s_s_max = (np.pi / 2) * ((np.cos(psi - theta) + 1) / 2)
#    print(theta, theta_conj, theta_etoile, delta_r)
    print(a,b)
    return delta_r, s_s_max, q, z

gammainf1 = (np.pi)/4
epsilon1 = (np.pi)/3
s_r_max1 = (np.pi)/4
r1 = 50
q1 = 0
correction1 = 1 # distance en mètre de correction de trajectoire 
dt1 = 1 #temps en secondes pour actualiser la trajectoire

def simulate_boat():
    # Simulation parameters
    dt = 1
    total_time = 500
    position = np.array([0.0, 0.0])
    theta = 0
    psi = np.pi / 4
    wind_variation = lambda t: np.pi / 4 * np.sin(t / 50)
    
    waypoint = np.array([-100.0, 100.0])
    waypoint_radius = 10
    q = 0    
    z = 0  # Integral term for correction
    
    history = []
    
    for t in range(total_time):
        psi = wind_variation(t)
        
        # Compute waypoints dynamically
        direction = waypoint - position
        a = (position)
        b = (position - direction / np.linalg.norm(direction) * 10)
        
        if np.linalg.norm(position - waypoint) < waypoint_radius:
            break  # Stop when inside the waypoint
        
        delta_r, s_s_max, q, z = tracking_function(position, theta, psi, a, b, r1, gammainf1, epsilon1, s_r_max1, q, correction1, dt, z)
        
        # Simulate movement
        theta += delta_r * dt
        position += np.array([np.cos(theta), np.sin(theta)]) * dt * 2  # Boat speed factor
        history.append(position.copy())
    
    history = np.array(history)
    
    # Plot results
    plt.figure(figsize=(8, 8))
    plt.plot(history[:, 0], history[:, 1], label="Boat Path")
    plt.scatter([waypoint[0]], [waypoint[1]], color='red', label="Waypoint")
    circle = plt.Circle(waypoint, waypoint_radius, color='r', fill=False, linestyle='dashed')
    plt.gca().add_patch(circle)
    plt.legend()
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Boat Navigation Simulation")
    plt.grid()
    plt.show()
    print(a,b)
    
simulate_boat()
