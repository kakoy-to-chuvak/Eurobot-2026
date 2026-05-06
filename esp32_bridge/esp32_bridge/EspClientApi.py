import socket
import struct
import threading
from collections import deque



def constrain(a, min_value, max_value):
    if a < min_value:
        return min_value
    if a > max_value:
        return max_value
    return a

MESSAGE_TYPES = {
    "GET_ALL"                    : 10,
    "GET_MOTORS_SPEED"           : 11,
    "GET_LIFT_HEIGHT"            : 12,
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
    
    "SEND_START"                 : 112,
    "SEND_SIDE"                  : 111,
}

LIDAR_MESSAGE_TYPES = {
    "SEND_LIDAR"                 : 110,
}



class AsyncSocket:
    def __init__(self, 
                 host: str,
                 port: int, 
                 log_function = print, 
                 socket_timeout: float = 0.5, 
                 max_pkg_size: int = 1024,
                 supported_messages: dict[str, int] = None ):
        self.host = host
        self.port = port
        self.log = log_function
        self.socket_timeout = socket_timeout
        self.max_pkg_size = max_pkg_size

        if supported_messages is None:
            self.supported_messages = {}
        else:
            self.supported_messages = supported_messages

        self.socket =  None
        self.running = False
        self.__thread =  None
        self.message_queue = deque(maxlen=100)

        self.sended_msgs = 0
        self.received_msgs = 0

    def create_msg(self, type: str, format: str, *args):
        if type not in self.supported_messages:
            self.log(f"Unknown message type: {type}")
            return None
        
        data = struct.pack("<b" + format, self.supported_messages[type], *args)
        data = struct.pack("<H", len(data)) + data
        
        return data
        
    def send_msg(self, type: str, format: str, *args) -> None:
        if self.socket and self.running:
            data = self.create_msg(type, format, *args)
            self.send(data)
        
    def send(self, msg):
        if self.socket and self.running:
            try:
                self.socket.sendall(msg)
                self.sended_msgs += 1 
            except Exception as e:
                self.log(f"Send error: {str(e)}")
                self.disconnect()

    def is_connected(self) -> bool:
        return self.socket is not None and self.running
    
    def _recv_exact(self, size: int) -> bytes:
        """Читает ровно size байт. Возвращает None при ошибке."""
        data = bytearray()
        while len(data) < size:
            if not self.running:
                return None
            try:
                chunk = self.socket.recv(size - len(data))
                if not chunk:  # Socket closed
                    return None
                data.extend(chunk)
            except socket.timeout:
                continue
            except (ConnectionError, BrokenPipeError, OSError) as e:
                self.log(f"Recv error: {e}")
                return None
        return bytes(data)
    
    def connect(self, timeout=1.0) -> bool:
        try:
            self.disconnect()
        except:
            pass
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(timeout)
        self.socket.setblocking(True)

        try:
            self.log(f"Connecting to server {self.host}:{self.port}")
            self.socket.connect((self.host, self.port))
            self.log("Connected!")
            self.socket.settimeout(self.socket_timeout)

            self.log("Start reading thread...")
            self.running = True
            self.__thread = threading.Thread(target=self._reader_thread, daemon=True)
            self.__thread.start()
            return True
        except Exception as e:
            self.log(f"Connection failed: {e}")
            if self.socket:
                self.socket.close()
            return False

    def disconnect(self) -> None:
        self.running = False
        if self.socket:
            try:
                self.socket.settimeout(0.1) 
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except:
                pass
        self.socket = None

        # Ждём завершение потока
        if self.__thread and self.__thread.is_alive():
            self.__thread.join(timeout=1.0)
            if self.__thread.is_alive():
                self.log("Warning: reader thread did not terminate")
        self.__thread = None

    def _reader_thread(self):
        self.log(f"Reading thread started {self.running} {self.socket is not None}")
        try:
            while self.running and self.socket is not None:
                try:
                    raw_len = self._recv_exact(2)
                    if raw_len is None:
                        break

                    data_size = struct.unpack("<H", raw_len)[0]
                    if data_size == 0:
                        continue
                    if data_size > self.max_pkg_size:
                        self.log(f"Invalid pkg size: {data_size}")
                        break
                    
                    data = self._recv_exact(data_size)
                    if data is None:
                        break

                    self.message_queue.append(data)
                    self.received_msgs += 1

                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"Reader error: {str(e)}")
                    break
        finally:
            self.disconnect()

class EspClient(AsyncSocket):
    def __init__(self, host: str, port: int, log_function = print, socket_timeout: float = 0.5):
        super().__init__(host, port, log_function, socket_timeout, 64, MESSAGE_TYPES)

        # обратный солварь для быстрого поиска по именам
        self.type_to_name = {v: k for k, v in MESSAGE_TYPES.items()}
        
    def receive_msg(self):
        if self.socket is None or self.running == False or len(self.message_queue) <= 0:
            return None
        
        bin_data = self.message_queue.popleft()
        if bin_data is None or len(bin_data) < 1:
            return None

        event_type = struct.unpack("<b", bin_data[:1])[0]
        
        event = self.type_to_name.get(event_type)
        if event is None:
            self.log(f"Undefined event type {event_type}")
            return None
        
        bin_data = bin_data[1:]
        
        try:
            match event:
                case "ANSWER_GET_MOTORS_SPEED":
                    data = struct.unpack("<ff", bin_data)
                    return {
                        "event"   : event,
                        "linear"  : data[0],
                        "angular" : data[1],
                    }

                case "ANSWER_GET_LIFT_HEIGHT":
                    data = struct.unpack("<H", bin_data)
                    return {
                        "event"  : event,
                        "height" : data[0],
                    }

                case "ANSWER_GET_SERVO_STATE":
                    data = struct.unpack("<BBBB", bin_data)
                    return {
                        "event" : event,
                        "state" : data
                    }

                case "ANSWER_GET_ODOMETRY":
                    data = struct.unpack("<fff", bin_data)
                    return {
                        "event" : event,
                        "theta" : data[0],
                        "x"     : data[1],
                        "y"     : data[2],
                    }

                case "ANSWER_GET_ALL":
                    data = struct.unpack("<ffHBBBBfff", bin_data)
                    return {
                        "event" : event,
                        "motors_speed": {
                            "linear"  : data[0],
                            "angular" : data[1],
                        },

                        "lift_height": data[2],

                        "servo_state": data[3:7],

                        "odometry": {
                            "theta" : data[7],
                            "x"     : data[8],
                            "y"     : data[9],
                        }
                    }

                case "SEND_START":
                    data = struct.unpack("<B", bin_data)
                    return {
                        "event": event,
                        "start": bool(data[0])
                    }

                case "SEND_SIDE":
                    data = struct.unpack("<B", bin_data)
                    return {
                        "event": event,
                        "side": "yellow" if data[0] == 0 else "blue"
                    }

                case _:
                    self.log(f"Undefined event: {str(event)}")
                    return None
        except Exception as e:
            self.log(f"Parse error for {event}: {str(e)}")
            return None

    def set_motors_speed(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        self.send_msg("SET_MOTORS_SPEED", "ff", linear_x, angular_z)
        
    def set_lift_height(self, height: int) -> None:
        self.send_msg("SET_LIFT_HEIGHT", "H", constrain(height, 0, 1000))
        
    def set_servo_state(self, angles) -> None:
        self.send_msg("SET_SERVO_STATE", "BBBB", constrain(angles[0], 0, 255), constrain(angles[1], 0, 255), constrain(angles[2], 0, 255), constrain(angles[3], 0, 255))
        
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
    



class LidarClient(AsyncSocket):
    # Lidar parameters
    FRAME_SIZE = 20
    FRAME_FORMAT = "<HHHHHHHHHH" # 20B – угол начала, угол конца, 8×дистанция
    CRC_SIZE = 2
    MAX_PKGS = 200 # max packages per rev                    

    def __init__(self, host: str, port: int, log_function = print, socket_timeout: float = 1.5):
        super().__init__(host, port, log_function, socket_timeout, self.MAX_PKGS * self.FRAME_SIZE + 1 + 12 +self.CRC_SIZE, LIDAR_MESSAGE_TYPES)
        
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
        if self.socket is None or self.running == False or len(self.message_queue) <= 0:
            return None

        bin_data = self.message_queue.popleft()
        if bin_data is None or len(bin_data) < 1:
            return None

        # Новый формат: [тип][theta][x][y][данные_лидара][CRC16]
        if len(bin_data) < 1 + 12 + self.FRAME_SIZE + self.CRC_SIZE:
            self.log("Packet too short")
            return None

        offset = 0
        msg_type = bin_data[offset]
        offset += 1

        # Проверяем тип сообщения (110 = LIDAR_MSG_DATA)
        if msg_type != LIDAR_MESSAGE_TYPES["SEND_LIDAR"]:
            self.log(f"Unexpected lidar message type: {msg_type}")
            return None

        # Читаем одометрию
        theta, xPos, yPos = struct.unpack("<fff", bin_data[offset:offset + 12])
        offset += 12

        # Остаток данных - лидар + CRC
        remaining = len(bin_data) - offset

        # Проверяем минимальную длину
        if remaining < self.FRAME_SIZE + self.CRC_SIZE:
            self.log(f"Not enough lidar data: {remaining} bytes")
            return None

        # Отделяем данные лидара и CRC
        lidar_raw = bin_data[offset:offset + remaining - self.CRC_SIZE]
        received_crc = int.from_bytes(bin_data[-self.CRC_SIZE:], "little")

        # Проверяем CRC
        if crc16(lidar_raw) != received_crc:
            self.log("Wrong crc")
            return None

        # Проверяем размер данных лидара
        if len(lidar_raw) < self.FRAME_SIZE or len(lidar_raw) > self.MAX_PKGS * self.FRAME_SIZE:
            self.log(f"Wrong lidar data size: {len(lidar_raw)}")
            return None

        # Конвертируем лидарные данные
        lidar_data = self.convert_lidar(lidar_raw)
        if lidar_data is None:
            self.log("Failed conversion")
            return None

        return {
            "angles": lidar_data[0],
            "ranges": lidar_data[1],
            "intens": lidar_data[2],
            "theta": theta,  
            "x": xPos,           
            "y": yPos,           
        }