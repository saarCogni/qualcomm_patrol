FROM cognimbus/pengo-base:melodic
# FROM ros:melodic
# # FROM ubuntu:18.04

# # ENV DEBIAN_FRONTEND=noninteractive

# # # Install ros melodic
# RUN apt update && apt upgrade -y && \
#     apt install -y lsb-core curl git

# # # Install ROS dependencies and packges
# RUN apt install -y \
#     ros-melodic-ecl-exceptions \
#     ros-melodic-ecl-threads \
#     ros-melodic-ecl-streams \
#     ros-melodic-ecl-geometry \
#     ros-melodic-yocs-controllers \
#     ros-melodic-yocs-cmd-vel-mux \
#     ros-melodic-yocs-velocity-smoother \
#     ros-melodic-dwa-local-planner \
#     ros-melodic-kobuki-msgs \
#     ros-melodic-kobuki-driver \
#     ros-melodic-kobuki-dock-drive \
#     ros-melodic-tf \
#     ros-melodic-image-transport \
#     ros-melodic-image-transport-plugins \
#     ros-melodic-resource-retriever \
#     ros-melodic-cv-bridge \
#     # move base installation
#     ros-melodic-move-base \
#     ros-melodic-base-local-planner \
#     ros-melodic-move-base-msgs \
#     # gmapping installation
#     ros-melodic-gmapping \
#     # realesnes installtaion
#     ros-melodic-realsense2-camera \
#     # webcam driver installation
#     ros-melodic-usb-cam \
#     # hokuyo lidar driver installtaion
#     ros-melodic-urg-node \
#     python-pip \ 
#     python-rosdep \
#     ros-melodic-kdl-conversions \
#     ros-melodic-urdf
    


# COPY ./src /qualcomm_patrol_ws/src
# WORKDIR /qualcomm_patrol_ws
# RUN . /opt/ros/melodic/setup.sh && rosdep install --from-paths src --ignore-src -r -y && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
