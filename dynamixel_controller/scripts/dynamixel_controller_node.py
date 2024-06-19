import rospy
from std_msgs.msg import String
from mavros_msgs.msg import RCOut
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import argparse
import atexit


# Deal with the node kill at execution termination
def exit_handler():
    global DYNAMIXEL_DATA
    rospy.loginfo("My application is ending!")
    try:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, DYNAMIXEL_DATA["DXL_ID1"], DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"], 0
        )
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, DYNAMIXEL_DATA["DXL_ID2"], DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"], 0
        )
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, DYNAMIXEL_DATA["DXL_ID3"], DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"], 0
        )
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, DYNAMIXEL_DATA["DXL_ID4"], DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"], 0
        )
    except:
        rospy.logerr("No motors connected!")


atexit.register(exit_handler)

# We will reserve all the variables in a global dictionary to be use throughout the node
DYNAMIXEL_DATA = dict()


# Control allocation
def output_mapping(valor):
    global DYNAMIXEL_DATA
    if valor < 0:
        saida = int((-valor / (2000 - DYNAMIXEL_DATA["PWM_TRIM"])) * 1023)
    else:
        saida = int((1024 + (valor / (2000 - DYNAMIXEL_DATA["PWM_TRIM"])) * 1023))
    return saida


# Callback for input radio signal
def rc_callback(data):
    global DYNAMIXEL_DATA

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID1"],
        DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"],
        output_mapping(
            -(
                data.channels[DYNAMIXEL_DATA["MOTOR_1_CHANNEL"]]
                - DYNAMIXEL_DATA["PWM_TRIM"]
            )
        ),
    )
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID2"],
        DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"],
        output_mapping(
            data.channels[DYNAMIXEL_DATA["MOTOR_2_CHANNEL"]]
            - DYNAMIXEL_DATA["PWM_TRIM"]
        ),
    )
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID3"],
        DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"],
        output_mapping(
            -(
                data.channels[DYNAMIXEL_DATA["MOTOR_1_CHANNEL"]]
                - DYNAMIXEL_DATA["PWM_TRIM"]
            )
        ),
    )
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID4"],
        DYNAMIXEL_DATA["ADDR_MX_MOVING_SPEED"],
        output_mapping(
            data.channels[DYNAMIXEL_DATA["MOTOR_2_CHANNEL"]]
            - DYNAMIXEL_DATA["PWM_TRIM"]
        ),
    )


def init_motors():
    global DYNAMIXEL_DATA
    # Set the port path
    portHandler = PortHandler(DYNAMIXEL_DATA["DEVICE_NAME"])

    # Set the protocol version
    protocol_version = 1.0  # See which protocol version is used in the Dynamixel
    packetHandler = PacketHandler(protocol_version)

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

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID1"],
        DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"],
        1,
    )
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel 1 has been successfully connected")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID2"],
        DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"],
        1,
    )
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel 2 has been successfully connected")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID3"],
        DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"],
        1,
    )
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel 3 has been successfully connected")
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler,
        DYNAMIXEL_DATA["DXL_ID4"],
        DYNAMIXEL_DATA["ADDR_MX_TORQUE_ENABLE"],
        1,
    )
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel 4 has been successfully connected")


def main():
    global DYNAMIXEL_DATA

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

    # Send initial commands and values to start the motors
    init_motors()

    # Read input commands and map to the motors
    rospy.Subscriber("/mavros/rc/out", RCOut, rc_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
