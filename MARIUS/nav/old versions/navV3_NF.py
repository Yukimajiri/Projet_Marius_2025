import math
import numpy as np

def compute_deltas(theta, psi, a_x, a_y, b_x, b_y, m_x, m_y, q, r):
    def T(x, y):
        return np.array([x, y])
    
    a = T(a_x, a_y)
    b = T(b_x, b_y)
    m = T(m_x, m_y)
    
    e = np.dot(m.T, (a / np.linalg.norm(a) * b / np.linalg.norm(b)))
    
    if abs(e) > r / 2:
        q = np.sign(e)
    
    M = np.array([
        [-np.sin(m_x), np.cos(m_x), 0],
        [-np.cos(m_x) * np.sin(m_y), -np.sin(m_x) * np.sin(m_y), np.cos(m_y)]
    ])
    
    phi = np.arctan2(np.dot(M, (b - a)))
    phi_star = phi - 2 * np.arctan(e / np.pi)
    
    if np.cos(psi - phi_star) + np.cos(q) < 0 or (np.linalg.norm(e) < np.r and (np.cos(psi - phi) + np.cos(q) < 0)):
        theta = np.pi + psi - q * np.cos(q)
    else:
        theta = phi_star
    
    if np.cos(theta - theta) > 0:
        delta_r = (np.pi / 2) * np.sin(theta - theta)
    else:
        delta_r = (np.pi / 2) * np.sign(np.sin(theta - theta))
    
    delta_s_max = (np.pi / 2) * ((np.cos(psi - theta) + 1) / 2)
    
    return delta_r, delta_s_max, q
