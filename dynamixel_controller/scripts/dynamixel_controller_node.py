import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCOut
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import argparse
import atexit


# We will reserve all the variables in a global dictionary to be use throughout the node
DYNAMIXEL_DATA = dict()
portHandler = None
packetHandler = None


def send_moving_speed(motor_id, speed_value):
    global DYNAMIXEL_DATA, portHandler, packetHandler
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, motor_id, DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"], speed_value
    )


# Deal with the node kill at execution termination
def exit_handler():
    global DYNAMIXEL_DATA, portHandler, packetHandler
    rospy.loginfo("My application is ending!")
    try:
        # Send 0 speeds to stop motors
        send_moving_speed(DYNAMIXEL_DATA["DXL_ID1"], 0)
        send_moving_speed(DYNAMIXEL_DATA["DXL_ID3"], 0)
        send_moving_speed(DYNAMIXEL_DATA["DXL_ID2"], 0)
        send_moving_speed(DYNAMIXEL_DATA["DXL_ID4"], 0)
    except:
        rospy.logerr("No motors connected!")


atexit.register(exit_handler)


# Control allocation
def output_mapping(valor):
    global DYNAMIXEL_DATA, portHandler, packetHandler
    if valor < 0:
        saida = int((-valor / (2000 - DYNAMIXEL_DATA["PWM_TRIM"])) * 1023)
    else:
        saida = int((1024 + (valor / (2000 - DYNAMIXEL_DATA["PWM_TRIM"])) * 1023))

    return saida


# Callback for input radio signal
def rc_callback(data):
    global DYNAMIXEL_DATA, portHandler, packetHandler
    motor_1_3_speed = output_mapping(
        -(data.channels[DYNAMIXEL_DATA["MOTOR_1_CHANNEL"]] - DYNAMIXEL_DATA["PWM_TRIM"])
    )
    motor_2_4_speed = output_mapping(
        data.channels[DYNAMIXEL_DATA["MOTOR_2_CHANNEL"]] - DYNAMIXEL_DATA["PWM_TRIM"]
    )

    # Send proper speeds
    send_moving_speed(DYNAMIXEL_DATA["DXL_ID1"], motor_1_3_speed)
    send_moving_speed(DYNAMIXEL_DATA["DXL_ID3"], motor_1_3_speed)
    send_moving_speed(DYNAMIXEL_DATA["DXL_ID2"], motor_2_4_speed)
    send_moving_speed(DYNAMIXEL_DATA["DXL_ID4"], motor_2_4_speed)


def init_motor(motor_id):
    global DYNAMIXEL_DATA, portHandler, packetHandler
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler,
        motor_id,
        DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"],
        1,
    )
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo(f"Dynamixel {motor_id} has been successfully connected!")


def init_motors():
    global DYNAMIXEL_DATA, portHandler, packetHandler
    try:
        if portHandler.openPort():
            rospy.loginfo("Succeeded to open the port")
        else:
            rospy.loginfo("Failed to open the port")
            quit()
    except:
        rospy.logerr("Failed to open the port - port is busy")
        quit()

    if portHandler.setBaudRate(DYNAMIXEL_DATA["BAUDRATE"]):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        quit()

    # Init the 4 motors
    init_motor(motor_id=DYNAMIXEL_DATA["DXL_ID1"])
    init_motor(motor_id=DYNAMIXEL_DATA["DXL_ID2"])
    init_motor(motor_id=DYNAMIXEL_DATA["DXL_ID3"])
    init_motor(motor_id=DYNAMIXEL_DATA["DXL_ID4"])


def main():
    global DYNAMIXEL_DATA, portHandler, packetHandler

    rospy.init_node("dynamixel_controller_node", anonymous=False)
    # Retrieve parameters with default values
    DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"] = rospy.get_param(
        "addr_mx_torque_enable", 24
    )
    DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"] = rospy.get_param("addr_mx_moving_speed", 32)
    DYNAMIXEL_DATA["DXL_ID1"] = rospy.get_param("dxl_id1", 1)
    DYNAMIXEL_DATA["DXL_ID2"] = rospy.get_param("dxl_id2", 2)
    DYNAMIXEL_DATA["DXL_ID3"] = rospy.get_param("dxl_id3", 3)
    DYNAMIXEL_DATA["DXL_ID4"] = rospy.get_param("dxl_id4", 4)
    DYNAMIXEL_DATA["BAUDRATE"] = rospy.get_param("baudrate", 57600)
    DYNAMIXEL_DATA["MOTOR_1_CHANNEL"] = rospy.get_param("motor_1_channel", 0)
    DYNAMIXEL_DATA["MOTOR_2_CHANNEL"] = rospy.get_param("motor_2_channel", 2)
    DYNAMIXEL_DATA["PWM_TRIM"] = rospy.get_param("pwm_trim", 1500)
    DYNAMIXEL_DATA["DEVICE_NAME"] = rospy.get_param("device_name", "/dev/ttyUSB0")

    # Set the port path
    portHandler = PortHandler(DYNAMIXEL_DATA["DEVICE_NAME"])

    # Set the protocol version
    protocol_version = 1.0  # See which protocol version is used in the Dynamixel
    packetHandler = PacketHandler(protocol_version)

    # Send initial commands and values to start the motors
    init_motors()

    # Read input commands and map to the motors
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
