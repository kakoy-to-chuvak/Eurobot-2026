# my_nav_launch

Launch-пакет для запуска навигации Nav2 с гибкой передачей параметров через командную строку.

## Аргументы командной строки

| Аргумент | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| map_file | string | ' ' | Полный путь к YAML файлу карты |
| config_file | string | ' ' | Полный путь к YAML файлу параметров Nav2 |
| use_sim_time | bool | False | Использовать симуляционное время |

## Запуск

### Базовый запуск с указанием карты и конфига

```bash
ros2 launch my_nav_launch navigation.launch.py \
    map_file:=/path/to/your/map.yaml \
    config_file:=/path/to/your/nav_param.yaml
```

### Запуск с симуляционным временем

```bash
ros2 launch my_nav_launch navigation.launch.py \
    map_file:=/path/to/map.yaml \
    config_file:=/path/to/nav_param.yaml \
    use_sim_time:=True
```

## Что делает launch-файл

1. Запускает `map_server` с указанной картой
2. Активирует `map_server` через `lifecycle_manager_map`
3. Запускает `nav2_bringup` с переданными параметрами


## Зависимости

- nav2_bringup
- nav2_map_server
- nav2_lifecycle_manager
- ros2launch