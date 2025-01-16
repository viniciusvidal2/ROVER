import serial
import socket
from pyubx2 import UBXReader, UBXMessage
from pymavlink import mavutil
from pyrtcm import RTCMReader

class RTKSurveyIn:
    def __init__(self, port, baudrate, udp_ip, udp_port):
        self.port = port
        self.baudrate = baudrate
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.serial_conn = None
        self.udp_socket = None
        self.mavlink_connection = None

    def connect(self):
        """Connect to GNSS receiver and prepare UDP socket."""
        try:
            # Connect to serial port
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to GNSS receiver on {self.port}")
            # Set up UDP socket
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP socket ready to send to {self.udp_ip}:{self.udp_port}")
            # Set up MAVLink connection (local dummy for message creation)
            self.mavlink_connection = mavutil.mavlink_connection('udpout:127.0.0.1:14550')
            return True
        except serial.SerialException as e:
            print(f"Error connecting to GNSS receiver: {e}")
            return False

    def configure_survey_in(self, acc_limit, min_duration):
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

    def monitor_survey_in(self):
        """Monitor Survey-In progress and print it."""
        if not self.serial_conn:
            print("Conexão não estabelecida.")
            return

        print("Monitorando o progresso do Survey-In...")
        ubr = UBXReader(self.serial_conn)
        try:
            while True:
                (raw_data, parsed_data) = ubr.read()
                if parsed_data:
                    # Monitoring NAV-SVIN (Survey-In progress)
                    if parsed_data.identity == "NAV-SVIN":
                        print(f"\nProgresso do Survey-In:")
                        print(f"  Duração: {parsed_data.dur} segundos")
                        print(f"  Precisão: {parsed_data.meanAcc / 10000:.3f} metros")
                        print(f"  Concluído: {'Sim' if parsed_data.active == 0 else 'Não'}")
                        if parsed_data.active == 0:  # Survey-In sucessfully completed
                            print("\nSurvey-In concluído com sucesso!")
                            break
        except KeyboardInterrupt:
            print("Monitoramento interrompido.")
        except Exception as e:
            print(f"Erro ao monitorar Survey-In: {e}")

    def configure_rtcm_messages(self):
        """Configures the RTCM3 messages required for RTK correction."""
        print("Configurando mensagens RTCM3...")
        
        # Configuring RTCM messages via USB port
        cfg_data = [
            ("CFG_MSGOUT_RTCM_3X_TYPE1006_USB", 1),  # Base station position
            ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 1),  # GPS
            ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 1),  # GLONASS
            ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 1),  # Galileo
            ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 1),  # BeiDou
            ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),  # GLONASS code-phase biases
        ]
        
        ubx_msg = UBXMessage.config_set(1, 0, cfg_data)
        self.serial_conn.write(ubx_msg.serialize())
        print("Mensagens RTCM3 configuradas")

    def monitor_rtcm_output(self):
        """Monitor and display RTCM messages using pyrtcm."""
        print("Monitorando mensagens RTCM...")
        rtcm_reader = RTCMReader(self.serial_conn)
        
        try:
            while True:
                (raw_data, parsed_data) = rtcm_reader.read()
                if parsed_data:
                    print(parsed_data)
                    
                if raw_data:
                    # Encapsulate raw RTCM data into MAVLink
                    rtcm_message = self.mavlink_connection.mav.gps_inject_data_encode(
                        target_system=1,        # FCU system ID
                        target_component=1,     # FCU component ID
                        len=len(raw_data),
                        data=raw_data
                    )
                    self.udp_socket.sendto(rtcm_message.pack(self.mavlink_connection.mav), 
                                            (self.udp_ip, self.udp_port))
                    print("RTCM data sent.")
                
        except KeyboardInterrupt:
            print("Monitoramento RTCM interrompido")
        except Exception as e:
            print(f"Erro ao monitorar RTCM: {e}")

    def close(self):
        """Close serial and UDP connections."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed.")
        if self.udp_socket:
            self.udp_socket.close()
            print("UDP socket closed.")

if __name__ == "__main__":
    rtk = RTKSurveyIn(port='COM4', baudrate=115200, udp_ip='192.168.10.161', udp_port=14551)

    if rtk.connect():
        try:
            # Configure Survey-In with accuracy limit in mm and minimum duration in seconds
            rtk.configure_survey_in(acc_limit=20000, min_duration=60)
            # Monitor Survey In progress
            rtk.monitor_survey_in()
            # Transmit RTCM messages
            rtk.configure_rtcm_messages()
            rtk.monitor_rtcm_output()
        finally:
            rtk.close()
