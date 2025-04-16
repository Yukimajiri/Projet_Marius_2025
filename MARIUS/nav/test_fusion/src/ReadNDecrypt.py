import threading
import serial
import time
import tools
from pymavlink import mavutil
from pynmeagps import NMEAReader, NMEAMessage
from queue import Queue

# --- CONFIGURATION ---
UDP_SERVER_IP = "127.0.0.1"
UDP_SERVER_PORT_TRAJ = 4000
UDP_SERVER_PORT_NETWORK = 4001
PIXHAWK_PORT = "/dev/ttyACM0"
BAUDRATE_PIXHAWK = 115200
GPS_PORT = "/dev/ttyUSB0"
BAUDRATE_GPS = 4800

# Define UDP destinations (MISSING IN ORIGINAL)
UDP_DESTINATIONS = [
    ("127.0.0.1", 4000),  # Doit correspondre à CONFIG["udp_port"] dans main.py
    ("127.0.0.1", 4001)   # Optionnel pour un autre système
]

# Navigation system integration
NAVIGATION_QUEUE = Queue()  # Thread-safe queue for GPS data

# --- Enhanced GPS Reader ---
def gps_reader(stop_event):
    """Thread function for continuous GPS reading"""
    try:
        with serial.Serial(GPS_PORT, BAUDRATE_GPS, timeout=1) as ser:
            print(f"📡 GPS reader started on {GPS_PORT}")
            nmr = NMEAReader(ser)
            
            while not stop_event.is_set():
                try:
                    raw_data, parsed_data = nmr.read()
                    
                    if isinstance(parsed_data, NMEAMessage):
                        if parsed_data.msgID == "GGA":
                            gps_data = {
                                "timestamp": time.time(),
                                "lat": parsed_data.lat,
                                "lon": parsed_data.lon,
                                "alt": parsed_data.alt,
                                "quality": parsed_data.quality
                            }
                            
                            NAVIGATION_QUEUE.put(("POSITION", (parsed_data.lat, parsed_data.lon)))
                            
                            tools.udp_forwarder({
                                "type": "gps_update",
                                "data": gps_data
                            }, UDP_DESTINATIONS)
                        
                        elif parsed_data.msgID == "VTG":
                            NAVIGATION_QUEUE.put(("HEADING", parsed_data.cogt))
                            NAVIGATION_QUEUE.put(("SPEED", parsed_data.spd_over_grnd_kmph))
                            
                except Exception as e:
                    print(f"⚠️ GPS read error: {str(e)}")
                    time.sleep(1)
                    
    except serial.SerialException as e:
        print(f"❌ GPS serial error: {str(e)}")
    finally:
        print("📴 GPS reader stopped")

# --- MAVLink Reader ---
def mavlink_reader(stop_event):
    """Thread function for Pixhawk data"""
    try:
        print(f"Connecting to Pixhawk on {PIXHAWK_PORT}...")
        master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUDRATE_PIXHAWK)
        master.wait_heartbeat()
        print("✅ Pixhawk connected")
        
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1
        )
        
        while not stop_event.is_set():
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg:
                if msg.get_type() in ['RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3']:
                    imu_data = {
                        "accel": [msg.xacc, msg.yacc, msg.zacc],
                        "gyro": [msg.xgyro, msg.ygyro, msg.zgyro],
                        "mag": [msg.xmag, msg.ymag, msg.zmag]
                    }
                    tools.udp_forwarder({"type": "imu_update", "data": imu_data}, UDP_DESTINATIONS)
                
                elif msg.get_type() == 'ATTITUDE':
                    NAVIGATION_QUEUE.put(("ATTITUDE", msg.yaw))
                    tools.udp_forwarder({
                        "type": "attitude_update",
                        "data": {
                            "roll": msg.roll,
                            "pitch": msg.pitch,
                            "yaw": msg.yaw
                        }
                    }, UDP_DESTINATIONS)
                    
    except Exception as e:
        print(f"❌ Pixhawk error: {str(e)}")
    finally:
        print("📴 Pixhawk reader stopped")

def main():
    stop_event = threading.Event()
    
    threads = [
        threading.Thread(target=gps_reader, args=(stop_event,), daemon=True),
        threading.Thread(target=mavlink_reader, args=(stop_event,), daemon=True)
    ]
    
    for t in threads:
        t.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        stop_event.set()
        for t in threads:
            t.join(timeout=1)
        print("✅ System stopped")

if __name__ == "__main__":
    main()