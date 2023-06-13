FROM osrf/ros:humble-desktop
RUN apt update -y && apt upgrade -y && apt install python3-pip vim -y
RUN pip3 install flask urx termcolor git+https://github.com/SintefManufacturing/python-urx