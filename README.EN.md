# Система удалённого управления манипулятором
[🇷🇺 RUSSIAN](README.md)
[🇬🇧 ENGLISH](README.EN.md)

![2024-01-23-211454_hyprshot](https://github.com/robotx-school/Remote-Manipulator/assets/55328925/6466a09f-4f2c-40e0-974b-d5a06af8dd7f)

Система для безопасного управления удалённым манипулятором (без физического доступа к нему).

# Запуск
Если у вас уже установлен ROS2 Humble, если нет рекомендуем ознакомиться с этой [https://habr.com/ru/articles/768048/](статьёй) или [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](офф.документацией). Установить достаточно минимальную версию и инструменты разработки(`ros-humble-ros-base`, `ros-dev-tools`)

Build packages:
```bash
colcon build .
```

```bash
source install/local_setup.bash
```

