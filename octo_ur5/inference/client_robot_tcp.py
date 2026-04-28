"""Robot-side TCP client for distributed Octo inference.

Runs on the robot machine. Connects to the GPU inference server
(octo_ur5.inference.server_tcp.py), receives action commands,
executes them on the real robot, and sends back observations.

Usage:
    python -m octo_ur5.inference.client_robot_tcp \
        --server_ip=10.8.14.160 --server_port=1242

This is the counterpart to server_tcp.py:
    server_tcp.py  (GPU machine)  <---TCP--->  client_robot_tcp.py  (robot machine)
"""

import os
import socket
import pickle
import time

from octo_ur5.real.real_ur5_env import RealUR5
from octo_ur5.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper


def main():
    server_ip = os.environ.get("OCTO_SERVER_IP", "10.8.14.160")
    server_port = int(os.environ.get("OCTO_SERVER_PORT", "1242"))

    env = RealUR5()
    env.reset()

    print(f"Connecting to inference server at {server_ip}:{server_port}...")

    while True:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client_socket.connect((server_ip, server_port))
        except ConnectionRefusedError:
            print("Connection refused. Retrying in 5s...")
            time.sleep(5)
            continue

        message = pickle.loads(client_socket.recv(1024))
        print(f"Received: {message['type']}")

        if message['type'] == 'reset':
            obs, _ = env.reset()
            data = pickle.dumps({'obs': obs, 'type': 'reset'})
        elif message['type'] == 'step':
            action = message['action']
            obs, reward, terminated, truncated, info = env.step(action)
            data = pickle.dumps({
                'obs': obs,
                'reward': reward,
                'terminated': terminated,
                'truncated': truncated,
                'info': info,
                'type': 'step',
            })
        else:
            print(f"Unknown message type: {message['type']}")
            client_socket.close()
            continue

        data_length = len(data)
        client_socket.sendall(pickle.dumps(data_length))

        chunk_size = 1024 * 33
        total_sent = 0
        while total_sent < data_length:
            chunk = data[total_sent:total_sent + chunk_size]
            sent = client_socket.send(chunk)
            total_sent += sent

        client_socket.send(b"EndTransmission")
        client_socket.close()
        print("Sent observation back to server.")


if __name__ == "__main__":
    main()
