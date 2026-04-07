import socket
import struct
import time
import math
from EspClientApi import EspClient



# robot variables
angular = 0
linear = 0

# LIft
tgt_height = 0
cur_height = 0

lift_speed = 80 # mm per sec
lift_timer = 0

# Servos
tgt_servos = [ 90, 90, 90, 90 ]
cur_servos = [ 90, 90, 90, 90 ]

servo_speed = 100 # deg per sec
servo_delay = 1 / servo_speed

servo_timer = 0

# Lidar simulation
lidar_timer = 0
lidar_delay = 0.5

# Odometry
xPos = 0.0
yPos = 0.0
theta = 0.0


# Lidar simulation objects
beacon_size = 0.095
lines = [
    [
        [ 1.5, -beacon_size / 2 ],
        [ 1.5, beacon_size / 2 ],
    ],
    [
        [ 1.5, beacon_size / 2  ],
        [ 1.5 + beacon_size, beacon_size / 2  ],
    ],
    [
        [ 1.5 + beacon_size, beacon_size / 2 ],
        [ 1.5 + beacon_size, -beacon_size / 2 ],
    ],
    [
        [ 1.5 + beacon_size, -beacon_size / 2 ],
        [ 1.5, -beacon_size / 2 ],
    ],

    [
        [ -1.5, -1.0 ],
        [ -1.5, -1.0 + beacon_size],
    ],
    [
        [ -1.5, -1.0 + beacon_size ],
        [ -1.5 - beacon_size, -1.0 + beacon_size ],
    ],
    [
        [ -1.5 - beacon_size, -1.0 + beacon_size ],
        [ -1.5 - beacon_size, -1.0 ],
    ],
    [
        [ -1.5 - beacon_size, -1.0 ],
        [ -1.5, -1.0 ],
    ],

    [
        [ -1.5, 1.0 ],
        [ -1.5, 1.0 - beacon_size ],
    ],
    [
        [ -1.5, 1.0 - beacon_size ],
        [ -1.5 - beacon_size, 1.0 - beacon_size ],
    ],
    [
        [ -1.5 - beacon_size, 1.0 - beacon_size ],
        [ -1.5 - beacon_size, 1.0 ],
    ],
    [
        [ -1.5 - beacon_size, 1.0 ],
        [ -1.5, 1.0 ],
    ],
]


# -------------------------- Helper function --------------------------
def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Modbus-совместимый CRC-16"""
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def get_time():
    return f"> {time.strftime("%H:%M:%S", time.localtime())} {time.perf_counter():.10} |"


# -------------------------- Server client class --------------------------
class MyClient:
    receive_lidar = 0
    
    socket_client = None
    data_size = 0
    request_time = 0
    
    def __init__(self, socket_client: socket.socket):
        self.socket_client = socket_client
        self.connect_time = time.perf_counter()
        self.socket_client.setblocking(False)
        self.connected = True
    
    def send_msg(self, type: str, format: str, *args) -> None:
        data = struct.pack("<b" + format, type, *args)
        data = struct.pack("<H", len(data)) + data
        
        self.socket_client.sendall(data)
        
        
    def handle_client(self):
        if self.data_size == 0:
            if self.available() < 2:
                return
            self.data_size = struct.unpack("<H", self.socket_client.recv(2))[0]
            self.request_time = time.perf_counter()
            # print(get_time(), "New data:", self.data_size)

        if self.data_size and time.perf_counter() - self.request_time > 0.3:
            print(get_time(), "Too big request time!")
            return
    
    
        if self.available() < self.data_size:
            return 
    
        data = self.socket_client.recv(self.data_size)

        event = struct.unpack("<B", data[:1])[0]
        #print(get_time(), "Event:", list(EspClient.message_types.keys())[list(EspClient.message_types.values()).index(event)])
        self.data_size = 0
        
        self.handle_data(event, data[1:])
        
    def handle_data(self, event_type, data):
        global linear, angular, tgt_height, cur_height, cur_servos, tgt_servos, xPos, yPos, theta
        match event_type:
            case 2:
                self.receive_lidar = struct.unpack("B", data)[0]
                print(get_time(), "Receive lidar:", self.receive_lidar)
            case 20:
                self.send_msg(30, "ff", linear, angular)
            case 21:
                self.send_msg(31, "f", cur_height)
            case 22:
                self.send_msg(32, "BBBB", cur_servos[0], cur_servos[1], cur_servos[2], cur_servos[3])
            case 23:
                self.send_msg(33, "fff", xPos, yPos, theta)
            case 29:
                self.send_msg(39, "fffBBBBfff", linear, angular, cur_height, cur_servos[0], cur_servos[1], cur_servos[2], cur_servos[3], xPos, yPos, theta)
            case 10:
                linear, angular = struct.unpack("ff", data)
            case 11:
                xPos, yPos, theta = struct.unpack("fff", data)
            case 13:
                tgt_height = struct.unpack("f", data)[0]
            case 12:
                tgt_servos = list(struct.unpack("BBBB", data))
              
                
    def available(self):
        try:
            data = self.socket_client.recv(1000, socket.MSG_PEEK)
            if len(data) == 0:
                self.connected = False
            return len(data)
        except BlockingIOError:
            return 0
        except (ConnectionResetError, BrokenPipeError):
            self.connected = False
            return 0
    
    def send_lidar(self, data):
        data = struct.pack("<B", 40) + data + struct.pack("<H", crc16(data))
        data = struct.pack("<H", len(data)) + data
        
        self.socket_client.setblocking(True)
        self.socket_client.sendall(data)
        self.socket_client.setblocking(False)


# -------------------------- Server --------------------------
HOST = "127.0.0.1"
PORT = 8080
PASSWORD = "374tfb39784"

clients = []

last_compute = 0
run = True


# Creating server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen()
server.setblocking(False)
print(get_time(), "Server created!")



# -------------------------- Robot simulation --------------------------
def get_angle(point: tuple) -> float:
    cos = point[0] / math.sqrt(point[0]**2 + point[1]**2)
    ang = -math.acos(cos)

    if point[1] < 0:
        ang = -ang
    
    if ang > 2*math.pi:
        ang -= 2 * math.pi
    elif ang < 0:
        ang += 2 * math.pi
        
    return ang


def transform_line(line: list):
    global xPos, yPos, theta

    point0 = [ line[0][0] - xPos, line[0][1] - yPos ]
    point1 = [ line[1][0] - xPos, line[1][1] - yPos ]

    ang0 = get_angle(point0)
    ang1 = get_angle(point1)

    if ang0 > ang1:
        ang0, ang1, point0, point1 = ang1, ang0, point1, point0

    vec2 = [ point0[0] - point1[0], point0[1] - point1[1] ]
    ang_b = math.acos( ( point0[0]*vec2[0] + point0[1]*vec2[1] ) / math.sqrt( ( point0[0]**2 + point0[1]**2 ) * ( vec2[0]**2 + vec2[1]**2 ) ) )

    line_len = math.sqrt( point0[0]**2 + point0[1]**2 )

    return [ang0, ang1, ang_b, line_len]

def create_lidar_pkg(size: int) -> bytes:
    global theta, lines

    t = time.perf_counter()
    pkg = bytes()
    ang = 0.0
    ang_inc = 360.0 / size

    # squares on scan
    objects = list(map(transform_line, lines))

    for i in range(size):
        if i%8 == 0:
            pkg += struct.pack("<HH", int(ang*100), int((ang+8*ang_inc)*100))
        rad = math.radians(ang) - theta
        if rad > 2 * math.pi:
            rad -= 2 * math.pi
        elif rad < 0:
            rad += 2 * math.pi
        
        rng = 10**9
        for obj in objects:
            # Проверяем перескок через 0
            if obj[1] - obj[0] > math.pi:
                if obj[0] <= rad <= obj[1]:
                    continue
                ang3 = math.pi - rad - obj[2] - obj[0]
            else:
                if not obj[0] <= rad <= obj[1]:
                    continue
                ang3 = math.pi - rad - obj[2] + obj[0]

            new_rng = 1000 * obj[3] * math.sin(obj[2]) / math.sin(ang3)
            if new_rng < rng:
                rng = new_rng


        # simulating noise
        if rng == 10**9:
            rng = 0  # 3700 + 250 * ( 2 * math.sin(rad*1.5 + t/60) + math.cos(rad*4 + 1 + t/20) + 0.5 * math.sin(rad*10 + 2 + t) + 0.4 * math.cos(rad*30 + 4*t) ) 


        rng = abs(rng)
        if rng > 65535:
            rng = 65000
        pkg += struct.pack("<H", int(rng))

        ang += ang_inc

    if len(pkg) % 20:
        pkg += ( 20 - (len(pkg) % 20) ) * struct.pack("<B", 0)

    return pkg






def ComputeOdometry():
    global xPos, yPos, theta, linear, angular, last_compute

    delta_t = time.perf_counter() - last_compute
    delta_s = linear * delta_t
    
    theta += angular * delta_t
    
    if theta > math.pi:
        theta -= 2 * math.pi
    elif theta < -math.pi:
        theta += 2 * math.pi

    xPos += delta_s * math.cos(theta)
    yPos += delta_s * math.sin(theta)

    last_compute = time.perf_counter()

def SimulateLift():
    global cur_height, lift_speed, lift_timer

    delta_t = time.perf_counter() - lift_timer
    if cur_height < tgt_height:
        cur_height += lift_speed * delta_t
        if cur_height > tgt_height:
            cur_height = tgt_height

    elif cur_height > tgt_height:
        cur_height -= lift_speed * delta_t
        if cur_height < tgt_height:
            cur_height = tgt_height

    lift_timer = time.perf_counter()

def SimulatServo():
    global servo_delay, servo_timer, tgt_servos, cur_servos

    if time.perf_counter() - servo_timer < servo_delay:
        return
    
    for i, ang in enumerate(zip(tgt_servos, cur_servos)):
        if ang[0] > ang[1]:
            cur_servos[i] += 1
        elif ang[0] < ang[1]:
            cur_servos[i] -= 1

    servo_timer = time.perf_counter()

def PublishLidar(client: MyClient):
    global lidar_timer, lidar_delay

    if time.perf_counter() - lidar_timer < lidar_delay:
        return
    
    lidar_pkg = create_lidar_pkg(400)
    client.send_lidar(lidar_pkg)

    lidar_timer = time.perf_counter()




# -------------------------- main cycle --------------------------
while run:

    # Simulate moving
    ComputeOdometry()
    SimulatServo()
    SimulateLift()

    # handle clients
    for i, client in enumerate(clients):
        try:
            client.handle_client()
            if not client.connected:
                print(get_time(), "Clent disconected")
                print(get_time(), "Deleting client")
                clients.pop(i)
                continue
            elif client.receive_lidar:
                PublishLidar(client)
        except KeyboardInterrupt:
            run = False


    # Accepting new clients
    try:
        new_client, addr = server.accept()        
        clients.append(MyClient(new_client))
        print(get_time(), "New client!")
    except KeyboardInterrupt:
        run = False

    except:
        pass


    
