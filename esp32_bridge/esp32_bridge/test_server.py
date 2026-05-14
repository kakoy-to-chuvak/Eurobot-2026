import socket
import struct
import time
import math
import random
from EspClientApi import MESSAGE_TYPES


# -------------------------- Общие утилиты --------------------------
def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Modbus-совместимый CRC-16"""
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def log(*args):
    """Возвращает отформатированную метку времени для логов."""
    print(f"> {time.strftime('%H:%M:%S', time.localtime())} {time.perf_counter():.10} |", *args)


# -------------------------- Модель состояния робота --------------------------
class RobotState:
    """Хранит и обновляет физическое состояние робота (одометрия, лифт, сервы)."""

    def __init__(self):
        # Одометрия
        self.x = 1.2
        self.y = 0.75
        self.theta = math.pi * 3 / 2
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self._last_odom_time = 0.0

        # Лифт
        self.lift_target = 0
        self.lift_current = 0
        self.lift_speed = 160.0          # мм/с
        self._last_lift_time = 0.0

        # Сервы (4 шт.)
        self.servo_target = [90, 90, 90, 90]
        self.servo_current = [90, 90, 90, 90]
        self.servo_speed = 100.0          # град/с
        self._servo_delay = 1.0 / self.servo_speed
        self._last_servo_time = 0.0

    def update_odometry(self) -> None:
        """Обновляет позицию робота на основе заданных линейной и угловой скоростей."""
        now = time.perf_counter()
        if self._last_odom_time == 0.0:
            self._last_odom_time = now
            return
        
        dt = now - self._last_odom_time
        if dt > 0.5:
            return
        
        ds = self.linear_speed * dt

        self.theta += self.angular_speed * dt
        # Нормировка угла в [-pi, pi]
        if self.theta > math.pi:
            self.theta -= 2 * math.pi
        elif self.theta < -math.pi:
            self.theta += 2 * math.pi

        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self._last_odom_time = now

    def update_lift(self) -> None:
        """Плавно перемещает лифт к целевой высоте."""
        now = time.perf_counter()
        if self._last_lift_time == 0.0:
            self._last_lift_time = now
            return
        
        dt = now - self._last_lift_time
        if self.lift_current < self.lift_target:
            self.lift_current += self.lift_speed * dt
            if self.lift_current > self.lift_target:
                self.lift_current = self.lift_target
        elif self.lift_current > self.lift_target:
            self.lift_current -= self.lift_speed * dt
            if self.lift_current < self.lift_target:
                self.lift_current = self.lift_target
        self._last_lift_time = now

    def update_servos(self) -> None:
        """Плавно поворачивает сервоприводы к целевым углам."""
        now = time.perf_counter()
        if now - self._last_servo_time < self._servo_delay:
            return
        for i in range(4):
            if self.servo_target[i] > self.servo_current[i]:
                self.servo_current[i] += 1
            elif self.servo_target[i] < self.servo_current[i]:
                self.servo_current[i] -= 1
        self._last_servo_time = now


# -------------------------- Лидарный сервер --------------------------
class LidarServer:
    """Генерирует лидарные данные и отправляет их по сокету."""

    def __init__(self, host: str, port: int, scan_size: int = 400):
        self.host = host
        self.port = port
        self.scan_size = scan_size

        # Геометрия эталонных объектов (четыре квадратных маркера)
        beacon_size = 0.095
        self._static_lines = [
            [[1.5, -beacon_size / 2], [1.5, beacon_size / 2]],
            [[1.5, beacon_size / 2], [1.5 + beacon_size, beacon_size / 2]],
            [[1.5 + beacon_size, beacon_size / 2], [1.5 + beacon_size, -beacon_size / 2]],
            [[1.5 + beacon_size, -beacon_size / 2], [1.5, -beacon_size / 2]],

            [[-1.5, -1.0], [-1.5, -1.0 + beacon_size]],
            [[-1.5, -1.0 + beacon_size], [-1.5 - beacon_size, -1.0 + beacon_size]],
            [[-1.5 - beacon_size, -1.0 + beacon_size], [-1.5 - beacon_size, -1.0]],
            [[-1.5 - beacon_size, -1.0], [-1.5, -1.0]],

            [[-1.5, 1.0], [-1.5, 1.0 - beacon_size]],
            [[-1.5, 1.0 - beacon_size], [-1.5 - beacon_size, 1.0 - beacon_size]],
            [[-1.5 - beacon_size, 1.0 - beacon_size], [-1.5 - beacon_size, 1.0]],
            [[-1.5 - beacon_size, 1.0], [-1.5, 1.0]],

            # test lines
            # [[0.0, 0.0], [0.0, beacon_size]],
            # [[0.0, beacon_size], [beacon_size, beacon_size]],
            # [[beacon_size, beacon_size], [beacon_size, 0.0]],
            # [[beacon_size, 0.0], [0.0, 0.0]],
        ]

        self._noisy_lines = self._static_lines[:]
        self._last_noise_update = time.perf_counter()

        # Параметры публикации
        self._publish_delay = 0.1          # 5 Гц
        self._last_publish_time = 0.0

        # Сокет
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen()
        self._server_socket.setblocking(False)
        self._client_socket = None
        log(f"Lidar server started on {self.host}:{self.port}")

    def _create_random_lines(self,
                             min_r: float = 2.4,
                             max_r: float = 3.6,
                             noise_power: int = 100,
                             k: float = 1.4,
                             line_creation: float = 0.6) -> list:
        """Генерирует случайные отрезки для имитации шумовых объектов."""
        if k < 0:
            k = 0
        elif k > 1.8:
            k = 1.8

        points = []
        alpha = 0.0
        delta_alpha = math.pi * 2 / noise_power

        while alpha <= math.pi * 2:
            r = min_r + random.random() * (max_r - min_r)
            points.append([r * math.sin(alpha), r * math.cos(alpha)])
            alpha += delta_alpha + (random.random() - 0.5) * delta_alpha * k

        noisy_segments = []
        prev = points[0]
        for point in points[1:]:
            if random.random() > 1 - line_creation:
                noisy_segments.append([prev, point])
            else:
                noisy_segments.append([[prev[0]*1.5, prev[1]*1.5], [point[0]*1.5, point[1]*1.5]])
            prev = point
        return noisy_segments

    def _get_angle(self, point: tuple) -> float:
        """Возвращает угол вектора (point) в радианах от 0 до 2π."""
        if point[0] == 0 and point[1] == 0:
            return 0.0
        cos_a = point[0] / math.hypot(point[0], point[1])
        ang = -math.acos(cos_a)
        if point[1] < 0:
            ang = -ang
        return ang % (2 * math.pi)

    def _transform_line(self, line: list, robot_x: float, robot_y: float) -> tuple:
        p0 = [line[0][0] - robot_x, line[0][1] - robot_y]
        p1 = [line[1][0] - robot_x, line[1][1] - robot_y]

        a0 = self._get_angle(p0)
        a1 = self._get_angle(p1)

        if a0 > a1:
            a0, a1, p0, p1 = a1, a0, p1, p0

        vec = [p0[0] - p1[0], p0[1] - p1[1]]
        dot = p0[0] * vec[0] + p0[1] * vec[1]
        len_p0 = math.hypot(p0[0], p0[1])
        len_vec = math.hypot(vec[0], vec[1])
        angle_b = math.acos(dot / (len_p0 * len_vec)) if len_p0 * len_vec != 0 else 0.0
        line_len = len_p0
        return a0, a1, angle_b, line_len

    def _build_lidar_packet(self, robot_theta: float, xPos: float, yPos: float) -> bytes:
        """Генерирует сырой пакет лидара (область + значения расстояний)."""
        # Обновляем шумовые линии каждые 3 секунды
        if time.perf_counter() - self._last_noise_update > 3.0:
            self._noisy_lines = self._static_lines + self._create_random_lines()
            self._last_noise_update = time.perf_counter()

        # Преобразуем все линии в систему координат робота
        obj_data = [self._transform_line(line, xPos, yPos) for line in self._noisy_lines]

        packet = bytearray()
        angle = 0.0
        angle_inc = 360.0 / self.scan_size

        for i in range(self.scan_size):
            if i % 8 == 0:
                packet.extend(struct.pack("<HH", int(angle * 100), int((angle + 8 * angle_inc) * 100)))
            rad = math.radians(angle) - robot_theta
            rad %= 2 * math.pi

            rng = 1e9
            for a0, a1, ang_b, line_len in obj_data:
                # Обработка перехода через 0
                if a1 - a0 > math.pi:
                    if a0 <= rad <= a1:
                        continue
                    ang3 = math.pi - rad - ang_b - a0
                else:
                    if not (a0 <= rad <= a1):
                        continue
                    ang3 = math.pi - rad - ang_b + a0
                new_rng = 1000 * line_len * math.sin(ang_b) / math.sin(ang3)
                if new_rng < rng:
                    rng = new_rng

            rng = abs(rng)
            if rng > 65535:
                rng = 65000
            packet.extend(struct.pack("<H", int(rng)))
            angle += angle_inc

        # Выравнивание по 20 байт
        extra = len(packet) % 20
        if extra:
            packet = packet[:-extra]
        return bytes(packet)

    def publish(self, robot_theta: float, xPos: float, yPos: float) -> None:
        """Отправляет один лидарный пакет с одометрией клиенту."""
        now = time.perf_counter()
        if now - self._last_publish_time < self._publish_delay:
            return
        self._last_publish_time = now

        if self._client_socket is None:
            try:
                self._server_socket.setblocking(True)
                self._client_socket, addr = self._server_socket.accept()
                log(f"New lidar client: {addr}")
                self._server_socket.setblocking(False)
            except BlockingIOError:
                return
            except OSError:
                return

        if self._client_socket:
            try:
                lidar_data = self._build_lidar_packet(robot_theta, xPos, yPos)

                # Новый формат: [тип(1)][theta(4)][x(4)][y(4)][лидарные_данные][CRC16(2)]
                LIDAR_MSG_TYPE = 110

                # Собираем пакет
                packet = bytearray()
                packet.append(LIDAR_MSG_TYPE)
                packet.extend(struct.pack("<fff", robot_theta, xPos, yPos))
                packet.extend(lidar_data)

                # Вычисляем CRC (от всего пакета без CRC в конце)
                crc_val = crc16(lidar_data)
                packet.extend(struct.pack("<H", crc_val))

                # Добавляем префикс длины
                final_packet = struct.pack("<H", len(packet)) + packet

                self._client_socket.sendall(final_packet)
            except:
                log("Lidar client disconnected")
                self._client_socket.close()
                self._client_socket = None

    def stop(self):
        """Закрывает сокет сервера."""
        if self._server_socket:
            self._server_socket.close()
            self._server_socket.shutdown()


# -------------------------- Обработчик одного клиента ESP --------------------------
class EspClientHandler:
    """Обслуживает одно соединение с клиентом (роботом)."""

    def __init__(self, sock: socket.socket, robot_state: RobotState):
        self.sock = sock
        self.robot = robot_state
        self.sock.setblocking(False)
        self.connected = True
        self._data_size = 0

    def send_msg(self, msg_type: str, fmt: str, *args) -> None:
        """Отправляет сообщение клиенту в протоколе EspClient."""
        try:
            data = struct.pack("<B" + fmt, MESSAGE_TYPES[msg_type], *args)
            data = struct.pack("<H", len(data)) + data
            self.sock.sendall(data)
        except Exception as e:
            print(f"Couldn`t send message: {str(e)}")

    def handle(self) -> None:
        """Обрабатывает входящие данные от клиента."""
        if self._data_size == 0:
            if self._available() < 2:
                return
            self._data_size = struct.unpack("<H", self.sock.recv(2))[0]

        if self._available() < self._data_size:
            return

        data = self.sock.recv(self._data_size)
        self._data_size = 0

        event_type = struct.unpack("<B", data[:1])[0]
        try:
            event = next(k for k, v in MESSAGE_TYPES.items() if v == event_type)
        except StopIteration:
            log(f"Undefined event type {event_type}")
            return

        self._handle_event(event, data[1:])

    def _handle_event(self, event: str, payload: bytes) -> None:
        """Выполняет действие в зависимости от типа события."""
        robot = self.robot
        match event:
            case "GET_MOTORS_SPEED":
                self.send_msg("ANSWER_GET_MOTORS_SPEED", "ff", robot.linear_speed, robot.angular_speed)

            case "GET_LIFT_HEIGHT":
                self.send_msg("ANSWER_GET_LIFT_HEIGHT", "H", int(robot.lift_current))

            case "GET_SERVO_STATE":
                self.send_msg("ANSWER_GET_SERVO_STATE", "BBBB",
                              *robot.servo_current)

            case "GET_ODOMETRY":
                self.send_msg("ANSWER_GET_ODOMETRY", "fff", robot.theta, robot.x, robot.y)

            case "GET_ALL":
                self.send_msg("ANSWER_GET_ALL", "ffHBBBBfff",
                              robot.linear_speed, robot.angular_speed,
                              int(robot.lift_current),
                              *robot.servo_current,
                              robot.theta, robot.x, robot.y)

            case "SET_MOTORS_SPEED":
                robot.linear_speed, robot.angular_speed = struct.unpack("ff", payload)

            case "SET_ODOMETRY":
                robot.x, robot.y, robot.theta = struct.unpack("fff", payload)
                print(f"set odom: {robot.x} {robot.y}")

            case "SET_LIFT_HEIGHT":
                h = struct.unpack("H", payload)[0]
                robot.lift_target = min(h, 200)

            case "SET_SERVO_STATE":
                robot.servo_target = list(struct.unpack("BBBB", payload))

    def _available(self) -> int:
        """Возвращает количество байт, доступных для чтения в сокете."""
        try:
            data = self.sock.recv(1000, socket.MSG_PEEK)
            if not data:
                self.connected = False
            return len(data)
        except BlockingIOError:
            return 0
        except OSError:
            self.connected = False
            return 0


# -------------------------- Основной сервер ESP --------------------------
class EspServer:
    """Управляет сервером робота, состоянием робота и циклами симуляции."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.robot = RobotState()
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen()
        self._server_socket.setblocking(False)
        self._client_handler = None
        self.running = True
        log(f"ESP server started on {self.host}:{self.port}")

    def update_simulation(self) -> None:
        """Обновляет все физические процессы робота."""
        self.robot.update_odometry()
        self.robot.update_servos()
        self.robot.update_lift()

    def accept_client(self) -> None:
        """Принимает нового клиента, если ни одного ещё нет."""
        if self._client_handler is not None:
            return
        try:
            client_sock, addr = self._server_socket.accept()
            log(f"New ESP client: {addr}")
            self._client_handler = EspClientHandler(client_sock, self.robot)
            # Отправить стартовые сообщения
            self._client_handler.send_msg("SEND_START", "B", 1)
            self._client_handler.send_msg("SEND_SIDE", "B", 1)

            self.robot._last_odom_time = time.perf_counter()
        except BlockingIOError:
            pass

    def handle_client(self) -> None:
        """Обрабатывает активного клиента, при необходимости отключает."""
        if self._client_handler is None:
            return
        self._client_handler.handle()
        if not self._client_handler.connected:
            log("Client disconnected")
            self._client_handler = None

    def stop(self):
        """Закрывает серверный сокет."""
        if self._server_socket:
            self._server_socket.close()
            self._server_socket.shutdown()
        self.running = False
        self.robot.angular_speed = 0.0
        self.robot.linear_speed = 0.0


# -------------------------- Главный цикл --------------------------
def main():
    HOST = "127.0.0.1"
    PORT = 8080
    LIDAR_PORT = 8090

    esp_server = EspServer(HOST, PORT)
    lidar_server = LidarServer(HOST, LIDAR_PORT)

    try:
        while esp_server.running:
            # Физика
            esp_server.update_simulation()

            # Клиент ESP
            esp_server.accept_client()
            esp_server.handle_client()

            # Лидар (использует текущий угол робота)
            lidar_server.publish(esp_server.robot.theta, esp_server.robot.x, esp_server.robot.y)

            time.sleep(0.01)   # небольшая пауза для снижения загрузки CPU
    
    except KeyboardInterrupt:
        log("Shutting down...")
        esp_server.stop()
        lidar_server.stop()

    finally:
        esp_server.stop()
        lidar_server.stop()


if __name__ == "__main__":
    main()