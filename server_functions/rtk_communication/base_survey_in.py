import serial
from pyubx2 import UBXReader, UBXMessage
from pyrtcm import RTCMReader, RTCMMessage

class RTKSurveyIn:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None

    def connect(self):
        """Conecta ao receptor GNSS."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Conectado à porta {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Erro ao conectar: {e}")
            return False

    def configure_survey_in(self, acc_limit, min_duration):
        """Configura o receptor para modo Survey-In."""
        print("Configurando Survey-In...")
        acc_limit = int(round(acc_limit / 0.1, 0))
        cfg_data = [
            ("CFG_TMODE_MODE", 1),  # Survey-In
            ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),  # Precisão em mm
            ("CFG_TMODE_SVIN_MIN_DUR", min_duration),  # Duração mínima em segundos
            ("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1),  # Ativa saída NAV-SVIN pela USB
            ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),   # Ativa saída NAV-PVT pela USB
        ]
        ubx_msg = UBXMessage.config_set(1, 0, cfg_data)  # RAM layer only
        self.serial_conn.write(ubx_msg.serialize())
        print(
            "Set ZED-F9P to Survey-In Timing Mode Basestation, "
            f"CFG, CFG_VALSET, {ubx_msg.payload.hex()}, 1\n"
        )

    def monitor_survey_in(self):
        """Monitora o progresso do Survey-In e imprime coordenadas."""
        if not self.serial_conn:
            print("Conexão não estabelecida.")
            return

        print("Monitorando o progresso do Survey-In...")
        ubr = UBXReader(self.serial_conn)
        try:
            while True:
                (raw_data, parsed_data) = ubr.read()
                if parsed_data:
                    # Monitorar NAV-SVIN (progresso do Survey-In)
                    if parsed_data.identity == "NAV-SVIN":
                        # print(parsed_data)
                        print(f"\nProgresso do Survey-In:")
                        print(f"  Duração: {parsed_data.dur} segundos")
                        print(f"  Precisão: {parsed_data.meanAcc / 10000:.3f} metros")
                        print(f"  Concluído: {'Sim' if parsed_data.active == 0 else 'Não'}")
                        if parsed_data.active == 0:  # Survey-In concluído
                            print("\nSurvey-In concluído com sucesso!")
                            break
        except KeyboardInterrupt:
            print("Monitoramento interrompido.")
        except Exception as e:
            print(f"Erro ao monitorar Survey-In: {e}")

    def configure_rtcm_messages(self):
        """Configura as mensagens RTCM3 necessárias para correção RTK."""
        print("Configurando mensagens RTCM3...")
        
        # Configuração das mensagens RTCM via porta USB
        cfg_data = [
            ("CFG_MSGOUT_RTCM_3X_TYPE1006_USB", 1),  # Posição da estação base
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
        """Monitora e exibe as mensagens RTCM usando pyrtcm."""
        print("Monitorando mensagens RTCM...")
        rtcm_reader = RTCMReader(self.serial_conn)
        
        try:
            while True:
                (raw_data, parsed_data) = rtcm_reader.read()
                if parsed_data:
                    # parsed_data é um objeto RTCMMessage com os dados já parseados
                    print(parsed_data)
                    
        except KeyboardInterrupt:
            print("Monitoramento RTCM interrompido")
        except Exception as e:
            print(f"Erro ao monitorar RTCM: {e}")

    def close(self):
        """Fecha a conexão serial."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Conexão fechada.")

if __name__ == "__main__":
    rtk = RTKSurveyIn(port='COM24', baudrate=115200)

    if rtk.connect():
        try:
            rtk.configure_survey_in(acc_limit=20000, min_duration=60)
            # Monitorar progresso do Survey-In e exibir coordenadas
            rtk.monitor_survey_in()
            # Após survey-in bem sucedido, configura e monitora RTCM
            rtk.configure_rtcm_messages()
            rtk.monitor_rtcm_output()
        finally:
            rtk.close()
