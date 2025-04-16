import numpy as np

class PureSailboatActuators:
    def __init__(self):
        # Paramètres physiques constants
        self.max_rudder_angle = np.pi/4      # 45° max pour le safran
        self.close_hauled_angle = np.pi/3    # 60° angle au près serré
        self.nominal_incidence = np.pi/4     # 45° incidence nominale
        self.track_corridor = 50             # 50m corridor de navigation

    def compute_actuators(self, boat_pos, boat_heading, wind_dir, line_start, line_end, current_tack):
        """
        Calcule UNIQUEMENT les angles de voile et safran sans aucune correction
        
        Args:
            boat_pos: position actuelle [x,y]
            boat_heading: cap actuel en radians
            wind_dir: direction du vent en radians (d'où vient le vent)
            line_start: point départ ligne [x,y]
            line_end: point arrivée ligne [x,y]
            current_tack: bord actuel (1 = tribord, -1 = bâbord)
            
        Returns:
            rudder_angle: angle du safran en radians
            sail_angle: angle de la voile en radians (0 = alignée avec bateau)
        """
        # 1. Calcul distance à la ligne
        line_vec = (line_end - line_start)/np.linalg.norm(line_end - line_start)
        pos_vec = boat_pos - line_start
        e = np.linalg.det(np.array([line_vec, pos_vec]))
        
        # 2. Calcul cap désiré
        line_angle = np.arctan2(line_end[1]-line_start[1], line_end[0]-line_start[0])
        theta_star = line_angle - (2*self.nominal_incidence/np.pi) * np.arctan(e/self.track_corridor)
        
        # 3. Gestion du près (mode au vent)
        if np.cos(wind_dir - theta_star) + np.cos(self.close_hauled_angle) < 0:
            theta_hat = np.pi + wind_dir - current_tack * self.close_hauled_angle
        else:
            theta_hat = theta_star
        
        # 4. Calcul angle safran
        if np.cos(boat_heading - theta_hat) >= 0:
            rudder_angle = self.max_rudder_angle * np.sin(boat_heading - theta_hat)
        else:
            rudder_angle = self.max_rudder_angle * np.sign(np.sin(boat_heading - theta_hat))
        
        # 5. Calcul angle voile (simplifié)
        apparent_wind = wind_dir - theta_hat
        sail_angle = np.pi/2 * (1 - np.cos(apparent_wind))  # 0° vent arrière, 90° vent de travers
        
        return rudder_angle, sail_angle