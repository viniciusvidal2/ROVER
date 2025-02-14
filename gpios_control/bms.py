#!/usr/bin/env python3
from dalybms import DalyBMS
import time

def read_bms_data():
    try:
        # Inicializa o BMS - usar address=4 para RS485/USB
        bms = DalyBMS(address=4)
        
        # Conecta ao dispositivo serial (ajuste a porta conforme necessário)
        bms.connect("/dev/ttyUSB0")
        
        while True:
            # Lê os dados básicos
            soc = bms.get_soc()
            cells = bms.get_cell_voltages()
            temp = bms.get_temperatures()
            status = bms.get_mosfet_status()
            
            # Limpa a tela
            print("\033[H\033[J")
            
            # Exibe os dados
            print("=== Dados do BMS ===")
            
            if soc:
                print(f"\nTensão Total: {soc['total_voltage']:.2f}V")
                print(f"Corrente: {soc['current']:.2f}A")
                print(f"SOC: {soc['soc_percent']:.1f}%")
            
            if cells:
                print("\nTensões das Células:")
                for cell, voltage in cells.items():
                    print(f"Célula {cell}: {voltage:.3f}V")
            
            if temp:
                print("\nTemperaturas:")
                for sensor, temp in temp.items():
                    print(f"Sensor {sensor}: {temp}°C")
            
            if status:
                print("\nStatus:")
                print(f"Modo: {status['mode']}")
                print(f"MOSFET Carga: {'Ligado' if status['charging_mosfet'] else 'Desligado'}")
                print(f"MOSFET Descarga: {'Ligado' if status['discharging_mosfet'] else 'Desligado'}")
                print(f"Capacidade: {status['capacity_ah']:.1f}Ah")
            
            # Aguarda 2 segundos antes da próxima leitura
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nEncerrando...")
    except Exception as e:
        print(f"\nErro: {e}")
    finally:
        bms.disconnect()

if __name__ == "__main__":
    read_bms_data()