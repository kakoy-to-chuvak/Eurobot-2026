# my_nav_launch

Launch-пакет для запуска навигации Nav2 с предварительной загрузкой карты.

## Запуск
```bash
ros2 launch my_nav_launch navigation.launch.py
```
## Что делает launch-файл

1. Запускает map_server с картой из папки map/
2. Активирует map_server через lifecycle_manager
3. Запускает nav2_bringup с параметрами из nav_config/nav_param.yaml

## Файлы конфигурации

nav_param.yaml - параметры Nav2: топики, фреймы, скорости, costmap

map.yaml - конфигурация карты: путь к image, разрешение, начало координат

## Зависимости

- nav2_bringup
- nav2_map_server
- nav2_lifecycle_manager
- ros2launch
