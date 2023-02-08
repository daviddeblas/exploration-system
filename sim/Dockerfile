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
    x11-xkb-utils xkb-data openbox
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
RUN apt-get install -y ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-robot-steering \
    ros-noetic-teleop-twist-keyboard xterm \
    python3-pip python3-yaml
RUN mkdir -p inf3995_ws/src && \
    git clone https://github.com/agilexrobotics/ugv_gazebo_sim inf3995_ws/src/ugv_gazebo_sim

COPY requirements.txt ./
RUN pip install -r requirements.txt

WORKDIR /inf3995_ws
RUN rosdep install --from-paths src --ignore-src -r -y

COPY . ./src/ugv_gazebo_sim/limo/limo_gazebo_sim/launch/
CMD bash
