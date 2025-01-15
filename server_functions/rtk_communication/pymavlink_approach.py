import socket
import serial
from pymavlink import mavutil

# Configuration
gps_serial_port = '/dev/ttyUSB0'  # Port connected to the RTK base station
gps_baudrate = 57600             # RTK base station baud rate
raspberry_ip = '192.168.10.45'   # Raspberry Pi IP address (running MAVROS)
raspberry_udp_port = 14551       # MAVROS UDP port
buffer_size = 1024               # Buffer size for serial reads

def main():
    # Open GPS serial connection
    gps_serial = serial.Serial(gps_serial_port, gps_baudrate, timeout=1)
    
    # Create a UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # MAVLink connection (for message creation only)
    mavlink_connection = mavutil.mavlink_connection('udpout:127.0.0.1:14550')  # Local dummy connection
    
    try:
        print("Starting RTCM transmission to Raspberry Pi...")
        while True:
            # Read RTCM data from GPS
            rtcm_data = gps_serial.read(buffer_size)
            if rtcm_data:
                # Create a MAVLink RTCM message
                rtcm_message = mavlink_connection.mav.gps_inject_data_encode(
                    target_system=1,        # FCU system ID (default: 1)
                    target_component=1,     # FCU component ID (default: 1)
                    len=len(rtcm_data),
                    data=rtcm_data
                )
                
                # Send the RTCM message over UDP to the Raspberry Pi
                udp_socket.sendto(rtcm_message.pack(mavlink_connection.mav), 
                                  (raspberry_ip, raspberry_udp_port))
    
    except KeyboardInterrupt:
        print("Script terminated by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        gps_serial.close()
        udp_socket.close()

if __name__ == '__main__':
    main()
