# Install
## General

### System packages:
```bash
sudo apt install putty putty-tools python3-pip ros-humble-cv-bridge unzip
```

### Project setup:
```bash
cd ~
mkdir test_ws
cd test_ws
git clone https://github.com/robotx-school/Remote-Manipulator
pip3 install -r requirements.txt
colcon build --symlink-install
```

### Configuration:
Edit file: `params/config.yaml`

```
ip: "192.168.1.2"  # robot ip
field_camera_path: # camera path (device from /dev/v4l/by-path/)
```

### Run:
```bash
cd ~/test_ws/Remote-Manipulator
ros2 launch launch/launch_all.py
```

## URX auto fix
```bash
python3 urx_patcher.py
```
