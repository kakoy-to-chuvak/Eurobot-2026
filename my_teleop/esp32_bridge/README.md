# esp32_bridge
ROS2 мост для ESP32 робота. Обеспечивает связь между ROS2 и прошивкой ESP32 через TCP сокеты.

## Запуск

```bash
# Базовый запуск
ros2 run your_package esp32_bridge.py

# С указанием параметров
ros2 run your_package esp32_bridge.py --ros-args -p host:=192.168.1.100 -p port:=8080 -p lidar_port:=8090
```

## Параметры

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `host` | string | 127.0.0.1 | IP адрес ESP32 |
| `port` | int | 8080 | Порт для управления роботом |
| `lidar_port` | int | 8090 | Порт для данных лидара |
| `odom_frame` | string | pwb_odom | Имя фрейма одометрии |
| `base_frame` | string | base_link | Имя базового фрейма робота |
| `lidar_frame` | string | lidar | Имя фрейма лидара |
| `lift_frame` | string | lift | Имя фрейма подъёмника |
| `servo_frames` | string[] | [servo0, servo1, servo2, servo3] | Имена фреймов сервоприводов |
| `lidar_xyz` | float[] | [0.0, 0.0, 0.40] | Позиция лидара относительно base_link (м) |
| `lidar_rpy_deg` | float[] | [0.0, 0.0, 0.0] | Ориентация лидара (градусы, roll/pitch/yaw) |
| `lidar_shift_deg` | float | -15.0 | Сдвиг углов скана лидара (градусы) |
| `lidar_mirror` | bool | True | Зеркальное отображение скана |
| `lidar_range_min` | float | 0.05 | Минимальная дальность лидара (м) |
| `lidar_range_max` | float | 4.0 | Максимальная дальность лидара (м) |
| `lift_xyz` | float[] | [0.05, 0.0, 0.0] | Позиция подъёмника относительно base_link (м) |
| `servo0_pos` | float[] | [0.01, -0.075, 0.0] | Позиция серво 0 (м) |
| `servo1_pos` | float[] | [0.01, -0.025, 0.0] | Позиция серво 1 (м) |
| `servo2_pos` | float[] | [0.01, 0.025, 0.0] | Позиция серво 2 (м) |
| `servo3_pos` | float[] | [0.01, 0.075, 0.0] | Позиция серво 3 (м) |

## Публикуемые топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/pwb/odom` | nav_msgs/Odometry | Одометрия робота |
| `/pwb/lidar_scan` | sensor_msgs/LaserScan | Сканирование лидара |
| `/pwb/lift_current_height` | std_msgs/UInt16 | Текущая высота подъёмника (мм) |
| `/pwb/servos_current_angles` | std_msgs/UInt8MultiArray | Текущие углы сервоприводов (0-90°) |

## Подписываемые топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/pwb/cmd_vel` | geometry_msgs/Twist | Целевые скорости робота |
| `/pwb/lift_target_height` | std_msgs/UInt16 | Целевая высота подъёмника (мм) |
| `/pwb/servos_target_angles` | std_msgs/UInt8MultiArray | Целевые углы сервоприводов (0-90°) |

## Трансформации (TF)

| from | to | тип | Описание |
|------|-----|-----|----------|
| map | pwb_odom | static | Фиксированная трансформация |
| pwb_odom | base_link | dynamic | Одометрия |
| base_link | lidar | static | Монтаж лидара |
| base_link | lift | dynamic | Позиция подъёмника |
| lift | servoN | dynamic | Позиция сервопривода (с учётом угла) |

## Особенности

### Автоматическое переподключение
При обрыве связи с ESP32 нода автоматически пытается восстановить соединение каждые 5 секунд.

### Graceful Shutdown
При завершении работы (Ctrl+C) нода корректно закрывает сокеты и останавливает робота.

### Поддержка лидара
- Автоматическое переподключение к серверу лидара
- Проверка CRC пакетов
- Конвертация углов и дистанций в LaserScan


## Требования

- ROS2
- Python 3.8+
- Пакеты: `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`

## Файлы

- `esp32_bridge.py` - главный узел
- `EspClientApi.py` - API для связи с ESP32 и лидаром
