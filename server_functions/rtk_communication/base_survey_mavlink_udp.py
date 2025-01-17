import serial
import socket
from pyubx2 import UBXReader, UBXMessage
from pymavlink import mavutil
from pyrtcm import RTCMReader

class RTKBaseReceiver:
    def __init__(self, port, baudrate, udp_ip, udp_port):
        self.port = port
        self.baudrate = baudrate
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.serial_conn = None
        self.mavlink_connection = None
        self.inject_seq_nr = 0

    def connect(self):
        """Connect to GNSS receiver and prepare UDP socket."""
        try:
            # Connect to serial port
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to GNSS receiver on {self.port}")
            # Set up MAVLink connection
            self.mavlink_connection = mavutil.mavlink_connection(f'udpout:{self.udp_ip}:{self.udp_port}', dialect='common')
            return True
        except serial.SerialException as e:
            print(f"Error connecting to GNSS receiver: {e}")
            return False

    def configureSurveyIn(self, acc_limit, min_duration):
        """Configure receiver for Survey-In mode."""
        print("Configuring Survey-In...")
        acc_limit = int(round(acc_limit / 0.1, 0))  # Convert to required units
        cfg_data = [
            ("CFG_TMODE_MODE", 1),  # Survey-In
            ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
            ("CFG_TMODE_SVIN_MIN_DUR", min_duration),
            ("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1),
            ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        ]
        ubx_msg = UBXMessage.config_set(1, 0, cfg_data)  # RAM layer only
        self.serial_conn.write(ubx_msg.serialize())
        print("Survey-In configuration sent.")

    def monitorSurveyIn(self):
        """Monitor Survey-In progress and print it."""
        if not self.serial_conn:
            print("Connection was not established, quitting survey in monitoring.")
            return

        print("Monitoring survey in progress...")
        ubr = UBXReader(self.serial_conn)
        try:
            while True:
                (_, parsed_data) = ubr.read()
                if parsed_data:
                    # Monitoring NAV-SVIN (Survey-In progress)
                    if parsed_data.identity == "NAV-SVIN":
                        print(f"\nSurvey in progress:")
                        print(f"  Duration: {parsed_data.dur} segundos")
                        print(f"  Precision: {parsed_data.meanAcc / 10000:.3f} metros")
                        print(f"  Finished: {'Yes' if parsed_data.active == 0 else 'No'}")
                        if parsed_data.active == 0:  # Survey-In sucessfully completed
                            print("\nSurvey in finished successfuly!")
                            break
        except KeyboardInterrupt:
            print("Monitoring interrupted.")
        except Exception as e:
            print(f"Error when monitoring survey in: {e}")

    def configureRtcmMessages(self):
        """Configures the RTCM3 messages required for RTK correction."""
        print("Configuring RTCM3 messages...")
        
        # Configuring RTCM messages via USB port
        cfg_data = [
            ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),  # Base station position
            ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 1),  # GPS
            ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 1),  # GLONASS
            ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),  # GLONASS code-phase biases
            ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 1),  # Galileo
            ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 1),  # BeiDou
        ]
        
        ubx_msg = UBXMessage.config_set(1, 0, cfg_data)
        self.serial_conn.write(ubx_msg.serialize())
        print("RTCM3 messages configuration sent.")

    def monitorRtcmOutput(self):
        """Monitor and transmit RTCM messages using pyrtcm."""
        print("Monitoring RTCM messages...")
        rtcm_reader = RTCMReader(self.serial_conn)
        
        try:
            while True:
                (data, parsed_data) = rtcm_reader.read()
                if parsed_data:
                    print(parsed_data)

                if data:
                    msglen = 180

                    # Check if the message is too large
                    if len(data) > msglen * 4:
                        print(f"DGPS: Message too large {len(data)}")
                        return
                    
                    # Determine the number of messages to send
                    msgs = len(data) // msglen if len(data) % msglen == 0 else (len(data) // msglen) + 1

                    for a in range(0, msgs):
                        flags = 0
                        # Set the fragment flag if we're sending more than 1 packet
                        if msgs > 1:
                            flags = 1
                        # Set the ID of this fragment
                        flags |= (a & 0x3) << 1
                        # Set an overall sequence number
                        flags |= (self.inject_seq_nr & 0x1f) << 3

                        # Get the chunk of data
                        amount = min(len(data) - a * msglen, msglen)
                        datachunk = data[a * msglen : a * msglen + amount]

                        # Send the RTCM data via MAVLink
                        self.mavlink_connection.mav.gps_rtcm_data_send(
                            flags,
                            len(datachunk),
                            bytearray(datachunk.ljust(180, b'\0'))
                        )
                    # Send a terminal 0-length message if we sent 2 or 3 exactly-full messages.     
                    if (msgs < 4) and (len(data) % msglen == 0) and (len(data) > msglen):
                        flags = 1 | (msgs & 0x3)  << 1 | (self.inject_seq_nr & 0x1f) << 3
                        self.mavlink_connection.mav.gps_rtcm_data_send(
                            flags,
                            0,
                            bytearray("".ljust(180, '\0')))
            
                    self.inject_seq_nr += 1
                    print(f"RTCM chunk sent: {len(datachunk)} bytes")
        except KeyboardInterrupt:
            print("RTCM monitoring interrupted.")
        except Exception as e:
            print(f"Error monitoring RTCM: {e}")

    def close(self):
        """Close serial and UDP connections."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed.")
        if self.udp_socket:
            self.udp_socket.close()
            print("UDP socket closed.")
        if self.mavlink_connection:
            self.mavlink_connection.close()
            print("MAVLink connection closed.")

if __name__ == "__main__":
    rtk_base_receiver = RTKBaseReceiver(port='COM4', baudrate=115200, udp_ip='192.168.10.161', udp_port=14550)

    if rtk_base_receiver.connect():
        try:
            # Configure Survey-In with accuracy limit in mm and minimum duration in seconds
            rtk_base_receiver.configureSurveyIn(acc_limit=20000, min_duration=60)
            # Monitor Survey In progress
            rtk_base_receiver.monitorSurveyIn()
            # Transmit RTCM messages
            rtk_base_receiver.configureRtcmMessages()
            rtk_base_receiver.monitorRtcmOutput()
        finally:
            rtk_base_receiver.close()
