FROM ros:noetic
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl git ros-noetic-roslint \
    libgl-dev libturbojpeg0-dev \
    libegl-dev freeglut3-dev \
    ocl-icd-opencl-dev \
    python3-pip python3-yaml \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-robot-steering \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-move-base \
    ros-noetic-navigation ros-noetic-explore-lite \
    ros-noetic-cv-bridge \
    ros-noetic-image-geometry \
    ros-noetic-image-transport \
    python3-wstool python3-rosdep ninja-build stow python3-catkin-tools \
    ros-noetic-robot-pose-ekf ros-noetic-slam-gmapping

RUN mkdir -p inf3995_ws/src 

COPY requirements.txt ./
RUN pip install -r requirements.txt

WORKDIR /inf3995_ws 

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && mkdir -p YDLidar-SDK/build \
    && cd YDLidar-SDK/build \
    && cmake .. \
    && make install \
    && cd ../.. \
    && rm -r YDLidar-SDK

ADD .rosinstall .rosinstall

RUN . /opt/ros/noetic/setup.sh && rosinstall .; catkin_make

COPY launch/* ./src/limo_ros/limo_bringup/launch/
COPY main.py ./src/
COPY beep.wav ./src/
COPY *.sh ./src/

RUN echo "source /inf3995_ws/devel/setup.sh" >> ~/.bashrc

RUN chmod +x ./src/launch_rover.sh
CMD ./src/launch_rover.sh
ENTRYPOINT /bin/bash
