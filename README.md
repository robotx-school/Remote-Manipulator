# Remote Manipulator
Set of packages for ROS (tested on Humble distro) to safe and easy remote manipulator(or another robotic) control.


## Docker run
Build container:
```bash
docker build -t ros2 .
```

Run container with one camera shared from host and source from this folder mounted to `/root/ros2_ws` in container:
```bash
docker run -v ./:/root/ros2_ws --device=/dev/video0:/dev/video0 -it ros2
```

And if you want to run X11 apps inside docker (like `rqt_graph`) use next command to run container:
```bash
docker run -v ./:/root/ros2_ws --device=/dev/video0:/dev/video0 -it --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" ros2
```

docker id: 968
sudo chown -R :<docker_group_id> <shared_folder_path>
sudo chmod -R g+w <shared_folder_path>


Run cmd:
docker run -v /home/stephan/Progs/RemoteManip:/root/ros2_ws --device=/dev/video0:/dev/video0 -it ros2


/usr/local/lib/python3.10/dist-packages/math3d/utils.py

WITH CAMERA AND X11:
