FROM ros:noetic
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl git ros-noetic-roslint \
    x11-apps mesa-utils \
    libgl-dev libturbojpeg0-dev \
    libegl-dev freeglut3-dev \
    ocl-icd-opencl-dev \
    libxcb1-dev libxcb-keysyms1-dev \
    libxcb-glx0-dev libx11-xcb-dev \
    libxtst-dev libpam0g-dev xauth \
    x11-xkb-utils xkb-data openbox \
    python3-pip python3-yaml gedit \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-robot-steering \
    ros-noetic-teleop-twist-keyboard xterm \
    ros-noetic-move-base \
    ros-noetic-navigation ros-noetic-explore-lite \
    python3-wstool python3-rosdep ninja-build stow python3-catkin-tools \
    ros-noetic-robot-pose-ekf ros-noetic-slam-gmapping

RUN mkdir -p inf3995_ws/src && \
    git clone https://github.com/agilexrobotics/limo_ros.git inf3995_ws/src/limo_ros

COPY requirements.txt ./
RUN pip install -r requirements.txt

WORKDIR /inf3995_ws 

COPY ./ydlidar_ws/* ./src/ydlidar_ws/

COPY launch/* ./src/limo_ros/limo_bringup/launch/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

COPY main.py ./src/
COPY beep.wav ./src/
COPY *.sh ./src/

RUN echo "source /inf3995_ws/devel/setup.sh" >> ~/.bashrc

RUN chmod +x ./src/launch_rover.sh
CMD ./src/launch_rover.sh
ENTRYPOINT /bin/bash
