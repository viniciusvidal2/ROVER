import requests
import argparse


def send_request(action, remote_ip):
    server_url = f"http://{remote_ip}:5000"

    # Define the mapping for actions to corresponding URLs
    endpoints = {
        "mapping_start": f"{server_url}/mapping/start",
        "mapping_stop": f"{server_url}/mapping/stop",
        "localization_start": f"{server_url}/localization/start",
        "localization_stop": f"{server_url}/localization/stop",
    }

    if action not in endpoints:
        print(
            f"Invalid action: {action}. Allowed actions: {list(endpoints.keys())}")
        return

    try:
        response = requests.post(endpoints[action])
        if response.status_code == 200:
            print(f"Success: {response.json()}")
        else:
            print(f"Failed: {response.status_code}, {response.text}")
    except Exception as e:
        print(f"Error while making request: {e}")


if __name__ == "__main__":
    # Setup argument parser
    parser = argparse.ArgumentParser(
        description='Send requests to a remote Flask server to control ROS nodes.')
    parser.add_argument(
        'action', type=str, help='Action to perform: mapping_start, mapping_stop, localization_start, localization_stop')
    parser.add_argument('remote_ip', type=str, help='Remote server IP address')

    # Parse arguments
    args = parser.parse_args()

    # Call the function with parsed arguments
    send_request(args.action, args.remote_ip)
