FROM ros:noetic
RUN apt-get update && apt-get install -y \
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
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-robot-steering \
    ros-noetic-teleop-twist-keyboard xterm \
    ros-noetic-move-base \
    ros-noetic-navigation ros-noetic-explore-lite \
    python3-wstool python3-rosdep ninja-build stow python3-catkin-tools

RUN curl -LO https://github.com/VirtualGL/virtualgl/archive/refs/tags/3.0.2.tar.gz && \
    tar xvf 3.0.2.tar.gz && \
    cmake -B build-virtualgl -S virtualgl-3.0.2 && \
    cmake --build build-virtualgl --parallel --target install && \
    rm -rf 3.0.2.tar.gz build-virtualgl virtualgl-3.0.2
RUN curl -LO https://github.com/TurboVNC/turbovnc/archive/refs/tags/3.0.2.tar.gz && \
    tar xvf 3.0.2.tar.gz && \
    cmake -B build-turbovnc -S turbovnc-3.0.2 -DTVNC_BUILDHELPER=OFF -DTVNC_BUILDVIEWER=OFF && \
    cmake --build build-turbovnc --target install && \
    rm -rf 3.0.2.tar.gz build-turbovnc turbovnc-3.0.2 && \
    echo 'export PATH="$PATH:/opt/VirtualGL/bin:/opt/TurboVNC/bin"' >> /root/.profile

RUN mkdir -p inf3995_ws/src && \
    git clone https://github.com/agilexrobotics/ugv_gazebo_sim inf3995_ws/src/ugv_gazebo_sim && \
    git clone https://github.com/willRicard/simple_quad_gazebo.git inf3995_ws/src/simple_quad/

COPY requirements.txt ./
RUN pip install -r requirements.txt

RUN mkdir catkin_ws
WORKDIR /catkin_ws
RUN wstool init src
RUN wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
RUN wstool update -t src

COPY package.xml ./src/cartographer/

RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

RUN src/cartographer/scripts/install_abseil.sh

COPY ./cartographer/* ../catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make_isolated -DCMAKE_CXX_STANDARD=17 --install --use-ninja"

WORKDIR /inf3995_ws
RUN rosdep install --from-paths src --ignore-src -r -y

COPY wall_generator.py ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/worlds/
COPY *.rviz ./src/ugv_gazebo_sim/limo/limo_description/rviz/
COPY limos.launch ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/launch/
COPY personalised.world ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/worlds/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_CXX_STANDARD=17"

COPY main*.py ./src/
COPY *.sh ./src/

RUN echo "source ../catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc

CMD bash
