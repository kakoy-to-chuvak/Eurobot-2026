import EspClientApi


HOST = "192.168.0.123"
PORT = 8090

lidar_client = EspClientApi.LidarClient(HOST, PORT)
lidar_client.connect()

while True:
    data = lidar_client.receive_lidar()
    if data:
        print(len(data), data)