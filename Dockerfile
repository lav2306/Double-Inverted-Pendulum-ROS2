FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    ros-noetic-effort-controllers \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    python3-pip \
    && pip3 install numpy matplotlib \
    && rm -rf /var/lib/apt/lists/*

COPY . /root/catkin_ws/src/double_inv_pend/

WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
