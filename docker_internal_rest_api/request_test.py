import requests
from typing import Tuple


def build_action_data(action: str, server_url: str) -> Tuple[str, dict]:
    """Build the data to be sent in the POST request

    Args:
        action (str): the action to be performed
        server_url (str): the server URL

    Returns:
        Tuple[str, dict]: the endpoint and data
    """
    # Define your POST endpoint HERE
    endpoint = f"{server_url}/{action}"
    # Define your POST data HERE
    data = {}

    if action == "/system/start_bag_record":
        data = {"bag_name": "test_bag"}
    elif action == "/system/stop_bag_record":
        data = {}
    elif action == "/mapping/start":
        data = {"map_name": "test_map"}
    elif action == "/mapping/stop":
        data = {}
    elif action == "/localization/start":
        data = {"map_name": "test_map"}
    elif action == "/localization/stop":
        data = {}
    elif action == "/system/copy_data_usb":
        data = {}

    return endpoint, data


def send_request(remote_ip: str,  action: "str", port: str = "5000") -> None:
    """Send a POST request to a remote server to test the rest api in the rover

    Args:
        remote_ip (str): the rover IP
        action (str): the action to be performed
        port (str): port number, defaults to 5000
    """
    server_url = f"http://{remote_ip}:{port}"

    # Build request data depending on the action
    endpoint, data = build_action_data(action, server_url)

    try:
        response = requests.post(endpoint, json=data)
        if response.status_code == 200:
            print(f"Success: {response.json().get('status')}")
        else:
            print(
                f"Failed: {response.status_code}, {response.json().get('error')}")
    except Exception as e:
        print(f"Error while making request: {e}")


if __name__ == "__main__":
    # remote_ip = "192.168.10.94"
    remote_ip = "127.0.0.1"
    port = "5000"
    # action = "/system/start_bag_record"
    # action = "/system/stop_bag_record"
    # action = "/mapping/start"
    # action = "/mapping/stop"
    # action = "/localization/start"
    # action = "/localization/stop"
    action = "/system/copy_data_usb"

    # Call the function with parsed arguments
    send_request(remote_ip=remote_ip, port=port, action=action)
