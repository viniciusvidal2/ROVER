import serial
from pyubx2 import UBXReader, UBXMessage
from pymavlink import mavutil
from pyrtcm import RTCMReader
from time import time
import yaml
from typing import Tuple

class RTKBaseReceiver:
    def __init__(self, usb_port: str, baudrate: int, rovers_file: str) -> None:
        """Constructor

        Args:
            usb_port (str): usb port of the GNSS receiver
            baudrate (int): usb port baudrate
            rovers_file (str): file with rovers information
        """
        # Serial connection to the GNSS receiver
        self.usb_port = usb_port
        self.baudrate = baudrate
        self.gnss_connection = None
        # Sequence number for RTCM messages
        self.inject_seq_nr = 0
        # Load rovers information
        with open(rovers_file, 'r') as file:
            rovers_list = yaml.safe_load(file)
            # List of mavlink connections to rovers
            self.rovers_mavlink_connections = [mavutil.mavlink_connection(
                f'udpout:{rover["ip"]}:{rover["port"]}', dialect='common') for rover in rovers_list["rovers"]]

    def connectToReceiver(self) -> None:
        """Connect to GNSS receiver and prepare UDP socket."""
        # Connect to serial port
        self.gnss_connection = serial.Serial(
            self.usb_port, self.baudrate, timeout=1)
        print(f"Connected to GNSS receiver on {self.usb_port}")
        return True

    def configureSurveyIn(self, acc_limit: int, min_duration: int) -> None:
        """Configure the Survey-In procedure.

        Args:
            acc_limit (int): Accuracy limit in millimeters
            min_duration (int): Minimum duration to run Survey-In in seconds
        """
        print("Configuring Survey-In...")
        # Convert to required units
        acc_limit = int(round(acc_limit / 0.1, 0))
        cfg_data = [
            ("CFG_TMODE_MODE", 1),  # Survey-In
            ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
            ("CFG_TMODE_SVIN_MIN_DUR", min_duration),
            ("CFG_MSGOUT_UBX_NAV_SVIN_USB", 1),
            ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        ]
        ubx_msg = UBXMessage.config_set(1, 0, cfg_data)  # RAM layer only
        self.gnss_connection.write(ubx_msg.serialize())
        print("Survey-In configuration sent.")

    def monitorSurveyIn(self) -> None:
        """Monitor Survey-In progress and print it."""
        if not self.gnss_connection:
            print("Connection was not established, quitting survey in monitoring.")
            return

        print("Monitoring survey in progress...")
        ubr = UBXReader(self.gnss_connection)

        while True:
            (_, parsed_data) = ubr.read()
            if parsed_data:
                # Monitoring NAV-SVIN (Survey-In progress)
                if parsed_data.identity == "NAV-SVIN":
                    print(f"\nSurvey in progress:")
                    print(f"  Duration: {parsed_data.dur} seconds")
                    print(
                        f"  Precision: {parsed_data.meanAcc / 10000:.3f} meters")
                    print(
                        f"  Finished: {'Yes' if parsed_data.active == 0 else 'No'}")
                    if parsed_data.active == 0:  # Survey-In sucessfully completed
                        print("\nSurvey in finished successfuly!")
                        break

    def configureRtcmMessages(self) -> None:
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
        self.gnss_connection.write(ubx_msg.serialize())
        print("RTCM3 messages configuration sent.")

    def createMessageContent(self, msg_data: bytes, chunk_number: int, n_chunks: int, msg_len: int) -> Tuple[int, bytes]:
        """Generates the message content based on the input data

        Args:
            msg_data (bytes): full message data
            chunk_number (int): the message chunk number
            n_chunks (int): how many chunks we have for this message
            msg_len (int): the message length, in bytes

        Returns:
            Tuple[int, bytes]: the message flags and the formated message data chunk
        """
        # Set the fragment flag if we're sending more than 1 packet
        flags = 0
        if n_chunks > 1:
            flags = 1
        # Set the ID of this fragment
        flags |= (chunk_number & 0x3) << 1
        # Set an overall sequence number
        flags |= (self.inject_seq_nr & 0x1f) << 3
        # Prepare the data chunk
        amount = min(len(msg_data) - chunk_number * msg_len, msg_len)
        data_chunk = msg_data[chunk_number *msg_len: chunk_number * msg_len + amount]

        return (flags, data_chunk)

    def sendMessageToRovers(self, flags: int, data_chunk: bytes) -> None:
        """Send RTCM message to each rover through specified mavlink connection

        Args:
            flags (int): the message flags in bynary
            data_chunk (bytes): the data chunk to be sent, in bytes
        """
        for mavlink_connection in self.rovers_mavlink_connections:
            mavlink_connection.mav.gps_rtcm_data_send(
                flags, len(data_chunk), bytearray(data_chunk.ljust(180, b'\0')))
        print(f"RTCM chunk sent: {len(data_chunk)} bytes")

    def monitorRtcmOutput(self) -> None:
        """Monitor and transmit RTCM messages using pyrtcm."""
        print("Monitoring RTCM messages...")
        rtcm_reader = RTCMReader(self.gnss_connection)

        start_time = 0
        while True:
            # Measuring time to avoid sending messages too fast
            
            # Read RTCM data
            if not start_time:
                start_time = time()
            (raw_data, parsed_data) = rtcm_reader.read()
            if raw_data:
                msg_len = 180  # Maximum length of RTCM message [bytes]
                # Check if the message is too large
                if len(raw_data) > msg_len * 4:
                    print(f"DGPS: Message too large {len(raw_data)}")
                    return

                # Determine the number of messages to send and send each chunk
                n_chunks = len(raw_data) // msg_len if len(raw_data) % msg_len == 0 else (len(raw_data) // msg_len) + 1
                for chunk_id in range(0, n_chunks):
                    # Send the RTCM data for the rovers via mavlink connections
                    flags, data_chunk = self.createMessageContent(
                        raw_data, chunk_id, n_chunks, msg_len)
                    self.sendMessageToRovers(
                        flags=flags, data_chunk=data_chunk)
                # Send a terminal 0-length message if we sent 2 or 3 exactly-full messages.
                if (n_chunks < 4) and (len(raw_data) % msg_len == 0) and (len(raw_data) > msg_len):
                    flags = 1 | (n_chunks & 0x3) << 1 | (
                        self.inject_seq_nr & 0x1f) << 3
                    data_chunk = ""
                    self.sendMessageToRovers(
                        flags=flags, data_chunk=data_chunk)

                # Update sequence number so the FCU can know we are done with this message
                self.inject_seq_nr += 1
                print(f"RTCM chunk sent: {len(data_chunk)} bytes")
            if parsed_data.identity == "1230":
                end_time = time()
                # Print the current send frequency
                print(f"Message send frequency: {1/(end_time - start_time):.2f} Hz")
                start_time = 0

    def close(self) -> None:
        """Close serial and UDP connections."""
        if self.gnss_connection and self.gnss_connection.is_open:
            self.gnss_connection.close()
            print("Serial connection closed.")
        for mavlink_connection in self.rovers_mavlink_connections:
            mavlink_connection.close()
        print("MAVLink connections closed.")


def main(usb_port: str, baudrate: int, rovers_file: str) -> None:
    """Generates the Base RTK procedure and monitors the Survey-In and RTCM messages.

    Args:
        usb_port (str): base receiver USB port
        baudrate (int): base receiver baudrate
        rovers_file (str): file with rovers information
    """
    rtk_base_receiver = RTKBaseReceiver(
        usb_port=usb_port, baudrate=baudrate, rovers_file=rovers_file)

    if rtk_base_receiver.connectToReceiver():
        try:
            # Configure Survey-In with accuracy limit in mm and minimum duration in seconds
            rtk_base_receiver.configureSurveyIn(acc_limit=20000, min_duration=10)
            # Monitor Survey In progress
            rtk_base_receiver.monitorSurveyIn()
            # Transmit RTCM messages
            rtk_base_receiver.configureRtcmMessages()
            rtk_base_receiver.monitorRtcmOutput()
        except KeyboardInterrupt:
            rtk_base_receiver.close()


if __name__ == "__main__":
    main(usb_port='COM24', baudrate=115200, rovers_file='rovers.yaml')
