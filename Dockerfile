FROM osrf/ros:noetic-desktop-full

# Instalação de pacotes necessários
RUN apt-get update && apt-get install -y \
    nano \
    git \
    && apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y\
    && apt install ros-noetic-gazebo-ros-control ros-noetic-ros-controllers -y\
    && apt-get update
    
    
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
RUN sudo apt-get remove --purge ibus
    
# Add user
RUN useradd -rm -d /home/ubuntu -p $(perl -e 'print crypt($ARGV[0], "password")' '') -s /bin/bash -g root -G sudo -u 1001 ubuntu

USER root

RUN mkdir -p /home/ubuntu/ur5_ws/src

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make > build_log_1.txt 2>&1 || tail -n 100 build_log_1.txt"

RUN cd /home/ubuntu/ur5_ws/src \
 && git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git \
 && git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel \
 && git clone https://github.com/ros-controls/ros_control.git -b noetic-devel \
 && git clone https://github.com/ros-controls/ros_controllers.git -b noetic-devel \
 && git clone https://github.com/ros/ros_comm.git -b noetic-devel

RUN cd /home/ubuntu/ur5_ws && rosdep update && rosdep install -y --from-paths . --ignore-src

WORKDIR /home/ubuntu/ur5_ws

RUN /bin/bash -c "cd /home/ubuntu/ur5_ws && source /opt/ros/noetic/setup.bash && catkin_make > build_log_2.txt 2>&1 || tail -n 100 build_log_2.txt"

RUN echo "cd /home/ubuntu/ur5_ws && source devel/setup.bash" >> /root/.bashrc

RUN echo "Imagem criada com sucesso."

