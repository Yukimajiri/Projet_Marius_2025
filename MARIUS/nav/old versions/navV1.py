import math
import numpy as np

def tracking_function(m, theta, psi, a, b,r, gamma_inf, epsilon, s_r_max, q):
    e = np.linalg.det(np.array([[b - a] / np.linalg.norm(b - a), m - a]))
    q = np.sign(e) if abs(e) > r / 2 else 0
    phi = np.arctan2(b[1] - a[1], b[0] - a[0])
    theta_etoile = phi - (2 * gamma_inf / np.pi) * np.arctan(e / r)
    if (np.cos(psi - theta_etoile) + np.cos(epsilon) < 0) or ((abs(e) < epsilon and (np.cos(psi - phi) + np.cos(epsilon) < 0))):
        theta_conj = np.pi + psi - q * epsilon
    else:
        theta_conj = theta_etoile

    if np.cos(theta - theta_conj) >= 0:
        delta_r = s_r_max * np.sin(theta - theta_conj)
    else:
        delta_r = s_r_max * np.sign(np.sin(theta - theta_conj))

    s_s_max = (np.pi / 2) * ((np.cos(psi - theta) + 1) / 2)

    return delta_r, s_s_max, q

gammainf1 = (np.pi)/4
epsilon1 = (np.pi)/3
s_r_max1 = (np.pi)/4
r1 = 50
q1 = 0