import serial
from pyubx2 import UBXReader, UBXMessage

# Código para configurar a base no modo Survey-In e monitrar o status da configuração

class RTKSurveyIn:
    def __init__(self, port='COM4', baudrate=115200):
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
                        print(parsed_data)
                        print(f"\nProgresso do Survey-In:")
                        print(f"  Duração: {parsed_data.dur} segundos")
                        print(f"  Precisão: {parsed_data.meanAcc / 10000:.3f} metros")
                        print(f"  Concluído: {'Sim' if parsed_data.active == 0 else 'Não'}")
                        if parsed_data.active == 0:  # Survey-In concluído
                            print("\nSurvey-In concluído com sucesso!")
                            break
                    # Monitorar NAV-PVT (dados de posição)
                    elif parsed_data.identity == "NAV-PVT":
                        if hasattr(parsed_data, "lat") and hasattr(parsed_data, "lon"):
                            lat = parsed_data.lat
                            lon = parsed_data.lon
                            alt = parsed_data.hMSL / 1000  # Altura em metros
                            print(f"\nCoordenadas atuais:")
                            print(f"  Latitude: {lat:.7f}°")
                            print(f"  Longitude: {lon:.7f}°")
                            print(f"  Altitude: {alt:.2f} metros (nível do mar)")
                        else:
                            print("Dados NAV-PVT inválidos ou incompletos.")
        except KeyboardInterrupt:
            print("Monitoramento interrompido.")
        except Exception as e:
            print(f"Erro ao monitorar Survey-In: {e}")

    def close(self):
        """Fecha a conexão serial."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Conexão fechada.")

if __name__ == "__main__":
    rtk = RTKSurveyIn(port='COM4', baudrate=115200)

    if rtk.connect():
        try:
            #  Configura Survey-In com precisão em mm e duração mínima em segundos
            rtk.configure_survey_in(acc_limit=2000, min_duration=60)
            # Monitorar progresso do Survey-In e exibir coordenadas
            rtk.monitor_survey_in()
        finally:
            rtk.close()
