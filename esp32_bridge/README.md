# esp32_bridge

ROS2 мост для ESP32 робота. Обеспечивает связь между ROS2 и прошивкой ESP32 через TCP сокеты.


## Запуск

```bash
ros2 run your_package esp32_bridge.py
```

## Параметры

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `host` | string | 127.0.0.1 | IP адрес ESP32 |
| `port` | int | 8080 | Порт для управления роботом |
| `lidar_port` | int | 8090 | Порт для данных лидара |
| `ping_frequency` | int | 10 | Частота пинга ESP32 для проверки соединения (Гц) |
| `odom_frame` | string | pwb_odom | Имя фрейма одометрии |
| `base_frame` | string | base_link | Имя базового фрейма робота |
| `lidar_frame` | string | lidar | Имя фрейма лидара |
| `lift_frame` | string | lift | Имя фрейма подъёмника |
| `servo_frames` | string[] | [servo0, servo1, servo2, servo3] | Имена фреймов сервоприводов |
| `lidar_xyz` | float[] | [0.0, 0.0, 0.40] | Позиция лидара относительно `base_link` (м) |
| `lidar_shift_deg` | float | -15.0 | Сдвиг углов скана лидара (градусы) |
| `lidar_mirror` | bool | True | Зеркальное отображение скана |
| `lidar_range_min` | float | 0.05 | Минимальная дальность лидара (м) |
| `lidar_range_max` | float | 4.0 | Максимальная дальность лидара (м) |
| `lift_xyz` | float[] | [0.05, 0.0, 0.0] | Позиция подъёмника относительно `base_link` (м) |
| `servo0_pos` | float[] | [0.01, -0.075, 0.0] | Позиция серво 0 (м) |
| `servo1_pos` | float[] | [0.01, -0.025, 0.0] | Позиция серво 1 (м) |
| `servo2_pos` | float[] | [0.01, 0.025, 0.0] | Позиция серво 2 (м) |
| `servo3_pos` | float[] | [0.01, 0.075, 0.0] | Позиция серво 3 (м) |
| `use_lift_tf` | bool | False | Публиковать динамический TF для подъёмника и сервоприводов |

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
| map | pwb_odom | static | Фиксированная трансформация (обнуляет `map`) |
| pwb_odom | base_link | dynamic | Одометрия робота |
| base_link | lidar | dynamic | Публикуется с каждым лидарным кадром для компенсации ошибок одометрии |
| base_link | lift | dynamic | Позиция подъёмника (если `use_lift_tf:=True`) |
| lift | servoN | dynamic | Позиция сервопривода с учётом угла (если `use_lift_tf:=True`) |

## Особенности

### Тип команды скорости
Узел подписан на топик `/pwb/cmd_vel` типа `geometry_msgs/Twist`. Nav2 по умолчанию публикует именно этот тип. Если ваш драйвер ожидает `TwistStamped`, используйте адаптер:
```bash
ros2 run topic_tools relay cmd_vel /pwb/cmd_vel
```

### Автоматическое переподключение
При обрыве связи с ESP32 нода автоматически пытается восстановить соединение.

### Лидар
- Поддержка сырых пакетов лидара с ESP32
- Проверка целостности данных (CRC16)
- Автоматическое переподключение к серверу лидара на ESP32
- Публикация TF `base_link -> lidar` динамически, вместе со сканом

## Файлы

- `esp32_bridge.py` — главный узел
- `EspClientApi.py` — API для TCP-связи с ESP32 (робот и лидар)