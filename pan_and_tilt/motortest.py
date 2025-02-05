from lewansoul_servo_bus import ServoBus
import time

servo_bus = ServoBus('COM3')

# servo_bus.id_write(1, 2)

# Move servo with ID 1 to 90 degrees in 1.0 seconds
#servo_bus.move_time_write(1, 145, 1.0)
#print(servo_bus.pos_read(1))
pan_angle_default = servo_bus.pos_read(1)
#tilt_angle_default = servo_bus.pos_read(2)
tilt_angle_default = 240
servo_bus.move_time_write(1, 125, 2)
time.sleep(2)
servo_bus.move_time_write(2, 240, 2)
# servo_bus.move_time_write(2, 95, 1.0)
# time.sleep(2)
# servo_bus.move_time_write(1, 155, 1.0)
# time.sleep(2)
# servo_bus.move_time_write(2, 65, 1.0)
# time.sleep(2)
# servo_bus.move_time_write(1, 140, 1.0)
# time.sleep(2)
# servo_bus.move_time_write(2, 80, 1.0)


def set_servo_angle(servo_id, angle, duration):
    """Define o ângulo do servo especificado com um tempo de movimento."""
    if 0 <= angle <= 240:  # Limite de angulo permitido pela biblioteca
        servo_bus.move_time_write(servo_id, angle, duration)
    else:
        print(f"Erro: Ângulo {angle} fora do limite permitido (0-240 graus).")


def main():
    i = 0
    try:
        while True:
            i = i + 1
            set_servo_angle(1, pan_angle_default, 0.5)
            set_servo_angle(2, tilt_angle_default, 1)
            pan_angle = pan_angle_default + 15
            tilt_angle = tilt_angle_default
            set_servo_angle(1, pan_angle, 0.5)
            time.sleep(1)
            # set_servo_angle(2, tilt_angle, 2)

            # Aguardar um curto período antes de receber novos comandos
            #time.sleep(3)

            set_servo_angle(1, pan_angle_default, 0.5)
            set_servo_angle(2, tilt_angle_default, 0.5)
            time.sleep(1)

            pan_angle = pan_angle_default - 15
            tilt_angle = tilt_angle_default - 50
            set_servo_angle(1, pan_angle, 0.5)
            time.sleep(1)
            set_servo_angle(1, pan_angle_default, 0.5)
            time.sleep(1)
            set_servo_angle(2, tilt_angle, 0.5)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nEncerrando o controle de Pan e Tilt.")
        set_servo_angle(1, pan_angle_default, 3)
        set_servo_angle(2, tilt_angle_default, 3)
        print(i)


if __name__ == "__main__":
    main()
