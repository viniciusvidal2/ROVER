import requests
import cv2
import numpy as np
import time
from lewansoul_servo_bus import ServoBus

# Inicializa a comunicação
servo_bus = ServoBus('COM3')  # mudar no ubuntu


# MOTOR 1: limitar a 90 graus
# MOTOR 2: limitar a 15 graus
# IDs
PAN_SERVO_ID = 1
TILT_SERVO_ID = 2

# default angles
PAN_STANDBY_ANGLE = 145
TILT_STANDBY_ANGLE = 150

# faixa de valores aceitavel
PAN_VALUES = 90
TIL_VALUES = 15

# Limites para os ângulos de pan e tilt
PAN_MIN_ANGLE = PAN_STANDBY_ANGLE - PAN_VALUES
PAN_MAX_ANGLE = PAN_STANDBY_ANGLE + PAN_VALUES
TILT_MIN_ANGLE = TILT_STANDBY_ANGLE - TIL_VALUES
TILT_MAX_ANGLE = TILT_STANDBY_ANGLE + TIL_VALUES

PAN_PIXELS = 1280   # Largura da imagem em pixels
TILT_PIXELS = 720   # Altura da imagem em pixels
PAN_FOV = 120       # Campo de visão horizontal em graus
TILT_FOV = 60       # Campo de visão vertical em graus


def pixels_to_angle(pixels, max_pixels, fov):
    """Converte um número de pixels em ângulos respeitando os limites."""
    angle = (pixels / max_pixels) * fov
    return -angle


def set_servo_angle(servo_id, angle, duration):
    """Define o ângulo do servo especificado com um tempo de movimento."""
    if 0 <= angle <= 240:  # Limite de angulo permitido pela biblioteca
        servo_bus.move_time_write(servo_id, angle, duration)
    else:
        print(f"Erro: Ângulo {angle} fora do limite permitido (0-240 graus).")


def main():
    set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 5)
    set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 5)
    time.sleep(6)
    deltas_X = []
    deltas_Y = []
    base_url = "http://192.168.0.101:8080"
    for i in range(1):
        # 1) Obter bounding boxes
        try:
            print("Buscando /pos...")
            r_pos = requests.get(f"{base_url}/pos", timeout=10)
            r_pos.raise_for_status()
            bboxes = r_pos.json()  # lista de dicts

        except Exception as e:
            print(f"Erro ao obter /pos: {e}")
            return

        if not bboxes:
            print("Nenhum objeto detectado no momento.")
            return

        # Exibe a lista de objetos
        print("Objetos detectados:")
        for i, obj in enumerate(bboxes):
            tag = obj.get('tag', 'obj')
            conf = obj.get('box', [0,0,0,0,0])[-1]  # 5o elemento
            print(f"[{i}] -> {tag} ({conf*100:.0f}%)")

        # 2) Perguntar ao usuário qual índice deseja
        choice_str = input("Digite o índice do objeto desejado: ")
        try:
            choice_idx = int(choice_str)
        except ValueError:
            print("Entrada inválida (não é um número). Encerrando.")
            return

        # 3) Fazer a requisição /escolha?index=X
        try:
            print(f"Enviando escolha = {choice_idx} para o servidor...")
            r_escolha = requests.get(
                f"{base_url}/escolha",
                params={"index": str(choice_idx)},
                timeout=10
            )
            r_escolha.raise_for_status()
            resposta = r_escolha.json()  # deve ser {"deltaX": ..., "deltaY": ..., "objTag": ...}
        except Exception as e:
            print(f"Erro ao obter /escolha: {e}")
            return

        deltaX = resposta.get("deltaX")
        deltaY = resposta.get("deltaY")
        deltas_X.append(deltaX)
        deltas_Y.append(deltaY)
        objTag = resposta.get("objTag", "obj?")
        print(f"Retorno do servidor:\n  Objecto: {objTag}\n  deltaX: {deltaX}\n  d'eltaY: {deltaY}")

    print("Médias:",np.mean(deltas_X),np.mean(deltas_Y))

    #pan_angle = pixels_to_angle(deltaX, PAN_PIXELS, PAN_FOV, PAN_MIN_ANGLE, PAN_MAX_ANGLE)
    #tilt_angle = pixels_to_angle(deltaY, TILT_PIXELS, TILT_FOV, TILT_MIN_ANGLE, TILT_MAX_ANGLE)

    # Colocar os servos em posição de standby
    set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 5)
    set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 5)
    try:
        pan_angle = pixels_to_angle(deltaX, PAN_PIXELS, PAN_FOV)
        tilt_angle = pixels_to_angle(deltaY, TILT_PIXELS, TILT_FOV)
        print(pan_angle)
        print(tilt_angle)
        pan_angle = -pan_angle + PAN_STANDBY_ANGLE
        tilt_angle = tilt_angle + TILT_STANDBY_ANGLE
        pan_angle = max(PAN_MIN_ANGLE, min(PAN_MAX_ANGLE, pan_angle))
        tilt_angle = max(TILT_MIN_ANGLE, min(TILT_MAX_ANGLE, tilt_angle))
        print(pan_angle)
        print(tilt_angle)

        set_servo_angle(PAN_SERVO_ID, pan_angle, 2)
        #set_servo_angle(TILT_SERVO_ID, tilt_angle, 2)


        # Aguardar um curto período antes de receber novos comandos
        time.sleep(0.1)
    except KeyboardInterrupt:
        # Colocar os servos em posição de standby
        set_servo_angle(PAN_SERVO_ID, PAN_STANDBY_ANGLE, 5)
        set_servo_angle(TILT_SERVO_ID, TILT_STANDBY_ANGLE, 5)
        print("\nEncerrando o controle de Pan e Tilt.")

    # 4) (Opcional) Baixar a imagem e desenhar TAMBÉM os bounding boxes
    #    Só pra vermos visualmente.
    print("Baixando a imagem de /imagem...")

    try:
        print("Buscando /pos...")
        r_pos = requests.get(f"{base_url}/pos", timeout=10)
        r_pos.raise_for_status()
        bboxes = r_pos.json()  # lista de dicts
    except:
        print("o")

    try:
        r_image = requests.get(f"{base_url}/imagem", timeout=10)
        r_image.raise_for_status()
        np_image = np.frombuffer(r_image.content, dtype=np.uint8)
        image = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
        if image is None:
            print("Falha ao decodificar a imagem.")
            return
    except Exception as e:
        print(f"Erro ao obter /imagem: {e}")
        return

    # 5) Desenhar bounding boxes na imagem
    for result in bboxes:
        box = result.get("box", [])
        tag = result.get("tag", "obj")
        if len(box) < 5:
            continue

        left, top, right, bottom, confidence = box
        left, top, right, bottom = map(int, [left, top, right, bottom])

        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
        label = f"{tag} {int(confidence*100)}%"
        cv2.putText(
            image,
            label,
            (left, top - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

    # Exibe a imagem
    cv2.imshow("Resultado", image)
    print("Feche a janela para encerrar.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
