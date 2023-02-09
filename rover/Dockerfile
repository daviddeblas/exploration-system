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

RUN apt-get install -y ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-robot-steering \
    ros-noetic-teleop-twist-keyboard xterm \
    python3-pip python3-yaml

WORKDIR /app

COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY . ./
RUN chmod +x start_docker.sh
CMD ["./start_docker.sh"]