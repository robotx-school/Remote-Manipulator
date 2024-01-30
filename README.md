# Система удалённого управления манипулятором
[🇷🇺 RUSSIAN](README.md)
[🇬🇧 ENGLISH](README.EN.md)

![2024-01-23-211454_hyprshot](https://github.com/robotx-school/Remote-Manipulator/assets/55328925/6466a09f-4f2c-40e0-974b-d5a06af8dd7f)

Система для безопасного управления удалённым манипулятором (без физического доступа к нему).

## Запуск
Если у вас уже установлен ROS2 Humble, если нет рекомендуем ознакомиться с этой [статьёй](https://habr.com/ru/articles/768048/) или [офф.документацией](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Установить достаточно минимальную версию и инструменты разработки(`ros-humble-ros-base`, `ros-dev-tools`). Обязательно установите пакет `ros-humble-cv-bridge`.

### Подготовка
```bash
source /opt/ros/humble/setup.bash # Опционально
cd ~/some_ws/
git clone https://github.com/robotx-school/Remote-Manipulator
```

### Сборка всех пакетов проекта:
```bash
colcon build .
source install/local_setup.bash
```

### Запуск
```bash
ros2 launch Remote-Manipulator/launch/launch_all.py
```

## Настройка
TODO

## FAQ
* Почему не MoveIt???
* Ошибка с python-urx, патч библиотеки для свежего python