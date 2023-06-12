FROM osrf/ros:humble-desktop
RUN apt update -y && apt upgrade -y && apt install python3-pip -y
RUN pip3 install flask
# RUN source /opt/ros/humble/setup.bash