import json
import time
import paho.mqtt.client as mqtt

from lewansoul_servo_bus import ServoBus

# -------------------------------------------------------------
# Configurações do MQTT
BROKER_ADDRESS = "localhost"  # ou "localhost", etc.
BROKER_PORT = 1883
TOPIC_DELTA_PIXEL = "camera/delta_pixel"  # Tópico que retorna deltaX/deltaY

# -------------------------------------------------------------
# Inicializa comunicação com os servos (ajustar se necessário)
servo_bus = ServoBus('COM3')  # ou a porta correspondente no seu sistema

PAN_SERVO_ID = 1
TILT_SERVO_ID = 2

# Ângulos iniciais "standby"
PAN_STANDBY_ANGLE = servo_bus.pos_read(PAN_SERVO_ID)
servo_bus.move_time_write(1, 125, 2)
TILT_STANDBY_ANGLE = servo_bus.pos_read(TILT_SERVO_ID)
servo_bus.move_time_write(2, 240, 2)

# Limites
PAN_VALUES = 15
TIL_VALUES = 90
PAN_MIN_ANGLE = PAN_STANDBY_ANGLE - PAN_VALUES
PAN_MAX_ANGLE = PAN_STANDBY_ANGLE + PAN_VALUES
TILT_MIN_ANGLE = TILT_STANDBY_ANGLE - TIL_VALUES
TILT_MAX_ANGLE = TILT_STANDBY_ANGLE + TIL_VALUES

PAN_PIXELS = 1280  # Largura da imagem
TILT_PIXELS = 720  # Altura da imagem
PAN_FOV = 120
TILT_FOV = 60


def pixels_to_angle(pixels, max_pixels, fov):
    """Converte um número de pixels em ângulos respeitando os limites."""
    angle = (pixels / max_pixels) * fov
    return -angle


def set_servo_angle(servo_id, angle, duration=2):
    """Define o ângulo do servo especificado com um tempo de movimento."""
    if 0 <= angle <= 240:  # Limite de angulo permitido pela lib
        servo_bus.move_time_write(servo_id, angle, duration)
    else:
        print(f"Erro: Ângulo {angle} fora do limite permitido (0-240 graus).")


# -------------------------------------------------------------
# Callbacks MQTT
def read_topic(msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        deltaX = data.get("deltaX", 0)
        deltaY = data.get("deltaY", 0)

        print(f"  deltaX: {deltaX}")
        print(f"  deltaY: {deltaY}")

        # Movimenta servos conforme deltaX, deltaY
        # Coloca primeiro o servo em standby
        set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 2)
        time.sleep(2)
        set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 2)
        time.sleep(2)

        pan_angle = pixels_to_angle(deltaX, PAN_PIXELS, PAN_FOV)
        tilt_angle = pixels_to_angle(deltaY, TILT_PIXELS, TILT_FOV)

        # Ajusta com base no ângulo "standby"
        pan_angle = -pan_angle + PAN_STANDBY_ANGLE
        tilt_angle = tilt_angle + TILT_STANDBY_ANGLE

        # Respeita limites
        pan_angle = max(PAN_MIN_ANGLE, min(PAN_MAX_ANGLE, pan_angle))
        tilt_angle = max(TILT_MIN_ANGLE, min(TILT_MAX_ANGLE, tilt_angle))

        print(f"Movendo servo pan para {pan_angle} e tilt para {tilt_angle}")
        set_servo_angle(PAN_SERVO_ID, pan_angle, 2)
        time.sleep(2)
        set_servo_angle(TILT_SERVO_ID, tilt_angle, 2)
        time.sleep(2)

    except Exception as e:
        print("Falha ao processar resposta de escolha:", e)


def on_connect(client, userdata, flags, rc):
    """Callback de conexão ao broker."""
    if rc == 0:
        print("Conectado ao broker MQTT!")
        # Inscrever-se nos tópicos relevantes
        client.subscribe(TOPIC_DELTA_PIXEL)
    else:
        print(f"Falha na conexão. Código de retorno = {rc}")


def on_message(client, userdata, msg):
    """Callback chamado quando chega mensagem em algum tópico inscrito."""

    if msg.topic == TOPIC_DELTA_PIXEL:
        # Recebemos a resposta com deltaX, deltaY
        read_topic(msg)

# -------------------------------------------------------------
# Função principal


def main():
    # Ajusta servos em standby no início
    set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 2)
    time.sleep(2)
    set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 2)
    time.sleep(2)

    # Cria cliente MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    # Conecta-se ao broker
    print(f"Conectando ao broker {BROKER_ADDRESS}:{BROKER_PORT}...")
    client.connect(BROKER_ADDRESS, BROKER_PORT, keepalive=60)

    try:
        # Loop infinito para ouvir mensagens
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        # Coloca servos em standby ao encerrar
        set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 2)
        set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 2)

# -------------------------------------------------------------


if __name__ == "__main__":
    main()
