import socket
import struct






class EspClient():
    
    message_types = {
        "GET_ALL"                    : 10,
        "GET_MOTORS_SPEED"           : 11,
        "SET_LIFT_HEIGHT"            : 12,
        "GET_SERVO_STATE"            : 13,
        "GET_ODOMETRY"               : 14,
        
        "ANSWER_GET_ALL"             : 40,
        "ANSWER_GET_MOTORS_SPEED"    : 41,
        "ANSWER_GET_LIFT_HEIGHT"     : 42,
        "ANSWER_GET_SERVO_STATE"     : 43,
        "ANSWER_GET_ODOMETRY"        : 44,
        
        "SET_MOTORS_SPEED"           : 71,
        "SET_LIFT_HEIGHT"            : 72,
        "SET_SERVO_STATE"            : 73,
        "SET_ODOMETRY"               : 74,
    }

    password = "374tfb39784"
    
    def __init__(self, host, port, log_function = print):
        self.host = host
        self.port = port
        self.log = log_function
        
        self.__client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sended_msgs = 0
        self.received_msgs = 0
        
        self.data_size = 0
       
    def connect(self, password: str, timeout=1.0):
        self.__client.settimeout(timeout)
        self.__client.setblocking(True)
        
        self.log(f"Connecting to server {self.host}:{self.port}")
        self.__client.connect((self.host, self.port))
        self.log("Connected!")
            
        self.__client.setblocking(False)
        
    def disconnect(self) -> None:
        self.__client.close()
        
    def create_msg(self, type: str, format: str, *args):
        data = struct.pack("<b" + format, self.message_types[type], *args)
        data = struct.pack("<H", len(data)) + data
        
        return data
        
    def send_msg(self, type: str, format: str, *args) -> None:
        data = self.create_msg(type, format, *args)

        self.__client.sendall(data)
        self.sended_msgs += 1
        
    def send(self, msg):
        self.__client.sendall(msg)
        self.sended_msgs += 1
        
    def available(self) -> int:
        try:
            return len(self.__client.recv(4000, socket.MSG_PEEK))
        except:
            return 0
        
        
    def receive_msg(self):
        if self.data_size == 0:
            if self.available() < 2:
                return None
            self.data_size = struct.unpack("<H", self.__client.recv(2))[0]
            if self.data_size > 3000:
                self.log(f"Too big data size: {self.data_size}")
                self.data_size = 0
                return None
            
        if self.available() < self.data_size:
            return None
            
        bin_data = self.__client.recv(self.data_size)
        self.data_size = 0
        
        event_type = struct.unpack("<b", bin_data[:1])[0]
        
        try:
            event = list(self.message_types.keys())[list(self.message_types.values()).index(event_type)]
        except ValueError:
            self.log(f"Undefined event type {event_type}")
            return None
            
        self.received_msgs += 1
        bin_data = bin_data[1:]
        
        match event:
            case "ANSWER_GET_MOTORS_SPEED":
                data = struct.unpack("ff", bin_data)
                return {
                    "event"   : event,
                    "linear"  : data[0],
                    "angular" : data[1],
                }
                
            case "ANSWER_GET_LIFT_HEIGHT":
                data = struct.unpack("f", bin_data)
                return {
                    "event"  : event,
                    "height" : data[0],
                }
                
            case "ANSWER_GET_SERVO_STATE":
                data = struct.unpack("BBBB", bin_data)
                return {
                    "event" : event,
                    "state" : data
                }
                
            case "ANSWER_GET_ODOMETRY":
                data = struct.unpack("fff", bin_data)
                return {
                    "event" : event,
                    "x"     : data[0],
                    "y"     : data[1],
                    "theta" : data[2],
                }
                
            case "ANSWER_GET_ALL":
                data = struct.unpack("fffBBBBfff", bin_data)
                return {
                    "event" : event,
                    "motors_speed": {
                        "linear"  : data[0],
                        "angular" : data[1],
                    },
                    
                    "lift_height": data[2],
                    
                    "servo_state": data[3:7],
                    
                    "odometry": {
                        "x"     : data[7],
                        "y"     : data[8],
                        "theta" : data[9],
                    }
                }
                
            case _:
                self.log(f"Undefined event: {str(event)}")
                return None

    def set_motors_speed(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        self.send_msg("SET_MOTORS_SPEED", "ff", linear_x, angular_z)
        
    def set_lift_height(self, height: float) -> None:
        self.send_msg("SET_LIFT_HEIGHT", "f", height)
        
    def set_servo_state(self, ang0: int = 0, ang1: int = 0, ang2: int = 0, ang3: int = 0) -> None:
        
        try:
            self.send_msg("SET_SERVO_STATE", "BBBB", ang0, ang1, ang2, ang3)
        except:
            pass
        
    def set_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.send_msg("SET_ODOMETRY", "fff", x, y, theta)
        
        
    
    def get_motors_speed(self) -> None:
        self.send_msg("GET_MOTORS_SPEED", "")

        
    def get_lift_height(self) -> None:
        self.send_msg("GET_LIFT_HEIGHT", "")

        
    def get_servo_state(self) -> None:
        self.send_msg("GET_SERVO_STATE", "")

    
    def get_odometry(self) -> None:
        self.send_msg("GET_ODOMETRY", "")
        
    def get_all(self):
        self.send_msg("GET_ALL", "")




def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Modbus-совместимый CRC-16"""
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF
    



class LidarClient:
    def __init__(self, host, port, log_function = print):
        self.host = host
        self.port = port
        self.log = log_function

        self.__client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sended_msgs = 0
        self.received_msgs = 0

        # Lidar parameters
        self.FRAME_SIZE = 20
        self.FRAME_FORMAT = "<HHHHHHHHHH" # 20B – угол начала, угол конца, 8×дистанция
        self.CRC_SIZE = 2
        self.MAX_PKGS = 200 # max packages per rev

    def connect(self, password: str, timeout=1.0):
        self.__client.settimeout(timeout)
        self.__client.setblocking(True)
        
        self.log(f"Connecting to server {self.host}:{self.port}")
        self.__client.sendto("connect str", (self.host, self.port))
        self.log("Connected!")
            
        self.__client.setblocking(False)

    def convert_lidar(self, raw: bytes):
        package_len = len(raw) // self.FRAME_SIZE
        if package_len < 38:  # < ~270°
            return None

        angles, ranges, intens = [], [], []
        prev = None
        off = 0.0

        for i in range(package_len):
            start, end, *dist = struct.unpack_from(self.FRAME_FORMAT, raw, i * self.FRAME_SIZE)
            
            start /= 100.0
            end   /= 100.0
            
            if end< start:
                end += 360.0

            for j, d in enumerate(dist):
                a = start + ( end - start ) * j / 7
                if prev is not None and a + off < prev - 300:
                    off += 360.0
                a += off
                prev = a

                angles.append(a)
                ranges.append(d / 1000.0 if d else float("inf"))
                intens.append(1.0 if d else 0.0)
                
        return (angles, ranges, intens)
    
    def receive_lidar(self):
        try:
            bin_data = self.__client.recv(self.FRAME_SIZE)
        except BlockingIOError:
            return None
        
        # Check for wrong data
        if not ( self.FRAME_SIZE + self.CRC_SIZE <= len(bin_data) <= self.MAX_PKGS * self.FRAME_SIZE + self.CRC_SIZE ):
            return None
        
        if crc16(bin_data[:-2]) != int.from_bytes(bin_data[-2:], "little"):
            return None
                
        lidar_data = self.convert_lidar(bin_data[:-2])

        if lidar_data == None:
            return None
                
        return {
            "angles": lidar_data[0],
            "ranges": lidar_data[1],
            "intens": lidar_data[2],
        }