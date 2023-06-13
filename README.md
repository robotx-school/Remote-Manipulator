docker id: 968
sudo chown -R :<docker_group_id> <shared_folder_path>
sudo chmod -R g+w <shared_folder_path>

Build container:
docker build -t ros2 .

Run cmd:
docker run -v /home/stephan/Progs/RemoteManip:/root/ros2_ws --device=/dev/video0:/dev/video0 -it ros2


/usr/local/lib/python3.10/dist-packages/math3d/utils.py