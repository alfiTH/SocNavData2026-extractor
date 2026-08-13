FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04

# Evitar prompts interactivos de tzdata y otras dependencias
ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalar herramientas básicas y añadir el repositorio de ROS Noetic
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# 2. Ahora sí, instalar ROS (ros-base) y todos los paquetes de tu arquitectura
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    python3-catkin-tools \
    python3-vcstool \
    python3-osrf-pycommon \
    python3-pip \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-slam-toolbox \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-amcl \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-robot-localization \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-xacro \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rqt-tf-tree \
    ros-noetic-rqt-reconfigure \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \ 
    ros-noetic-joint-state-publisher \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-roslint \
    ros-noetic-image-transport-plugins \
    ros-noetic-people-msgs \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-topic \
    ros-noetic-rqt-plot \
    ros-noetic-rqt-console \
    ros-noetic-rqt-runtime-monitor \
    ros-noetic-rqt-multiplot \
    ros-noetic-tf \
    ros-noetic-tf2-tools \
    ros-noetic-plotjuggler \
    ros-noetic-plotjuggler-ros \
    libpcap-dev \
    python3-rosdep \
    git \
    autopoint \
    gtk-doc-tools \
    libgtk-3-dev \
    git \
    cmake \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Smaller/general Python dependencies
RUN python3 -m pip install \
    pyserial \
    scipy \
    lap \
    cython_bbox \
    pandas \
    tqdm \
    seaborn \
    imagehash \
    PyOpenGL \
    PyOpenGL-accelerate

# GPU packages
RUN python3 -m pip install \
    cupy-cuda12x \
    torch==2.4.1 \
    torchvision==0.19.1

# Application packages
RUN python3 -m pip install \
    ultralytics \
    pyside6

# RUN pip3 install pyserial scipy cupy torch torchvision lap cython_bbox pandas matplotlib tqdm seaborn ultralytics imagehash pyside6 PyOpenGL PyOpenGL-accelerate

# Crear el workspace en el contenedor
WORKDIR /root/catkin_ws
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# El script de entrada (Entrypoint)
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
