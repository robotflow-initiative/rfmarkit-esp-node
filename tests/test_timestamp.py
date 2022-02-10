import logging
import socket
import time


def tcp_send_bytes(arguments):
    addr: str = arguments['addr']
    port: int = arguments['port']
    data: int = arguments['data']
    reply = ''

    logging.debug(f"Sending {data} to {addr}:{port}")

    # Setup the server
    ctrl_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # ctrl_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ctrl_socket.settimeout(2)
    try:
        ctrl_socket.connect((addr, port))
    except Exception as err:
        if isinstance(err, socket.timeout) or isinstance(err, ConnectionRefusedError):
            return {"addr": addr, "msg": reply}
        else:
            logging.warning(f"{err} for {addr}")
            return {"addr": addr, "msg": reply}

    ctrl_socket.settimeout(0.05)

    try:
        ctrl_socket.send(data)
        while True:
            reply += str(ctrl_socket.recv(1024), encoding='ascii')
    except socket.timeout:
        logging.warning(f"Socket timeout for {addr}")
    return {"addr": addr, "msg": reply}


# with tqdm.tqdm(range(1)) as pbar:
#     while True:
#         pbar.set_description(f"Timestamp={time.time()}")
res = tcp_send_bytes({'addr': '10.53.24.180', 'port': 18888, 'data': "time".encode(encoding='ascii')})
local_time = time.time()
imu_time = float(res['msg'].split(':')[-1])

print(res)


print(f'local_time={local_time}, imu_time={imu_time}, delta={local_time - imu_time}')
