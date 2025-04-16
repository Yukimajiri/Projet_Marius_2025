import math
from geopy.distance import geodesic
import json
import time
from ReadNDecrypt import NAVIGATION_QUEUE

def get_navigation_updates():
    """Process incoming sensor data"""
    while not NAVIGATION_QUEUE.empty():
        msg_type, data = NAVIGATION_QUEUE.get()
        if msg_type == "POSITION":
            navigator.update_position(*data)
        elif msg_type == "HEADING":
            navigator.current_heading = math.radians(data)

class GPSNavigator:
    def __init__(self):
        # GPS-based initialization
        self.current_pos = None  # (latitude, longitude)
        self.waypoints = []      # List of (lat, lon) tuples
        self.current_wp_index = 0
        self.proximity_threshold = 10  # meters
        self.current_heading = 0       # degrees (0-360)
        
        # Navigation parameters (from your original code)
        self.gammainf = math.pi/4      # 45°
        self.epsilon = math.pi/3       # 60°
        self.s_r_max = math.pi/4       # 45°
        self.r = 50                    # meters
        self.correction = 1
        self.dt = 1
        self.z = 0

    def update_position(self, lat, lon, heading=None):
        """Update current GPS position and heading"""
        self.current_pos = (lat, lon)
        if heading is not None:
            self.current_heading = heading
            
    def load_waypoints(self, json_file):
        """Load waypoints from JSON file created by the IHM"""
        with open(json_file) as f:
            data = json.load(f)
            self.waypoints = [(p['lat'], p['lng']) for p in data['points']]
        self.current_wp_index = 0 if self.waypoints else -1
        
    def distance_to_current_waypoint(self):
        """Calculate distance to current waypoint in meters"""
        if not self.current_pos or self.current_wp_index < 0:
            return float('inf')
        return geodesic(self.current_pos, self.waypoints[self.current_wp_index]).meters
    
    def tracking_function(self, wind_direction):
        """Adapted tracking function for GPS navigation"""
        if not self.current_pos or self.current_wp_index < 0:
            return 0, 0, 0, self.z  # Neutral positions
            
        a = self.current_pos
        b = self.waypoints[self.current_wp_index]
        
        # Convert GPS coordinates to vectors for calculations
        # Using approximate conversion (good for small distances)
        dx = geodesic((a[0], a[1]), (a[0], b[1])).meters * (1 if b[1] > a[1] else -1)
        dy = geodesic((a[0], a[1]), (b[0], a[1])).meters * (1 if b[0] > a[0] else -1)
        
        # Normalize the vector
        norm = math.sqrt(dx**2 + dy**2)
        if norm > 0:
            dx /= norm
            dy /= norm
        
        # Calculate cross track error (e)
        e = -dy * (b[0] - a[0]) + dx * (b[1] - a[1])
        q = 1 if e > 0 else -1 if e < 0 else 0
        
        # Calculate desired bearing
        phi = math.atan2(dy, dx)
        
        # Your original control logic adapted for GPS
        alpha = (self.correction / (e * self.dt)) if e != 0 else 0
        self.z = self.z + alpha * self.dt * e
        theta_etoile = phi - ((2 * self.gammainf) / math.pi) * ((e + self.z) / self.r)
        
        if (math.cos(self.current_heading - theta_etoile) + math.cos(self.epsilon) < 0) or \
           ((abs(e) < self.epsilon) and (math.cos(self.current_heading - phi) + math.cos(self.epsilon) < 0)):
            theta_conj = math.pi + self.current_heading - q * self.epsilon
        else:
            theta_conj = theta_etoile

        if math.cos(self.current_heading - theta_conj) >= 0:
            delta_r = self.s_r_max * math.sin(self.current_heading - theta_conj)
        else:
            delta_r = self.s_r_max * math.sign(math.sin(self.current_heading - theta_conj))

        s_s_max = (math.pi / 2) * ((math.cos(wind_direction - self.current_heading) + 1) / 2)
        
        return delta_r, s_s_max, q, self.z
    
    def navigate_to_waypoints(self, wind_callback, sail_control, rudder_control):
        """Main navigation loop"""
        while True:
            # 1. Check if we have waypoints
            if not self.waypoints:
                time.sleep(1)
                continue
                
            # 2. Check if we've reached current waypoint
            if self.distance_to_current_waypoint() < self.proximity_threshold:
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    print("All waypoints completed!")
                    return
                    
            # 3. Get current wind direction (from callback)
            wind_dir = wind_callback()
            
            # 4. Calculate control angles
            delta_r, s_s_max, q, self.z = self.tracking_function(wind_dir)
            
            # 5. Update heading based on rudder command
            self.current_heading += delta_r * self.dt
            self.current_heading %= 2 * math.pi  # Normalize to 0-2π
            
            # 6. Convert to sail and rudder angles (-40° to 40° for sail, -45° to 45° for rudder)
            sail_angle = math.degrees(max(-self.epsilon, min(self.epsilon, s_s_max)))
            rudder_angle = math.degrees(max(-math.pi/4, min(math.pi/4, delta_r)))
            
            # 7. Send commands to actuators
            sail_control(sail_angle)
            rudder_control(rudder_angle)
            
            # 8. Prevent CPU overload
            time.sleep(self.dt)
    # Example usage at the bottom of nav.py:

def get_wind_direction():
        """Implement this to get real wind direction from sensors"""
        # This should return wind direction in radians (0-2π)
        pass

def control_sail(angle):
        """Implement this to control your sail"""
        print(f"Setting sail angle to {angle:.1f}°")

def control_rudder(angle):
        """Implement this to control your rudder"""
        print(f"Setting rudder angle to {angle:.1f}°")

if __name__ == "__main__":
    while True:
        get_navigation_updates()  # Process incoming sensor data
        # Rest of your navigation logic...
        navigator = GPSNavigator()
        
        # Load waypoints from IHM system
        navigator.load_waypoints("gps_points.json")

        # Start navigation (this will block)
        navigator.navigate_to_waypoints(
            wind_callback=get_wind_direction,
            sail_control=control_sail,
            rudder_control=control_rudder
        )