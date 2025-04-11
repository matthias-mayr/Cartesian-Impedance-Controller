# If you would like to use lightweight image, you can use the following base image:
# FROM ros:humble-ros-base

FROM osrf/ros:humble-desktop-full 

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG BUILD_FRANKA_ROS=false

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        libeigen3-dev \
        libboost-all-dev \
        vim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

WORKDIR /ros2_ws
COPY . /ros2_ws/src/Cartesian-Impedance-Controller

RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws

RUN sudo apt update
	
RUN rosdep update && \
    rosdep install --from-paths src/Cartesian-Impedance-Controller --ignore-src --rosdistro=$ROS_DISTRO -y

RUN if [ "$BUILD_FRANKA_ROS" = "true" ]; then \
      echo "Installing franka_ros2 dependencies and cloning repository..."; \
      sudo apt-get install -y --no-install-recommends \
      	   ros-humble-franka-description \
           ros-humble-ros-gz \
           ros-humble-sdformat-urdf \
           ros-humble-joint-state-publisher-gui \
           ros-humble-ros2controlcli \
           ros-humble-controller-interface \
           ros-humble-hardware-interface-testing \
           ros-humble-ament-cmake-clang-format \
           ros-humble-ament-cmake-clang-tidy \
           ros-humble-controller-manager \
           ros-humble-ros2-control-test-assets \
           libignition-gazebo6-dev \
           libignition-plugin-dev \
           ros-humble-hardware-interface \
           ros-humble-control-msgs \
           ros-humble-backward-ros \
           ros-humble-generate-parameter-library \
           ros-humble-realtime-tools \
           ros-humble-joint-state-publisher \
           ros-humble-joint-state-broadcaster \
           ros-humble-moveit-ros-move-group \
           ros-humble-moveit-kinematics \
           ros-humble-moveit-planners-ompl \
           ros-humble-moveit-ros-visualization \
           ros-humble-joint-trajectory-controller \
           ros-humble-moveit-simple-controller-manager \
           ros-humble-rviz2 \	
           ros-humble-xacro && \
      sudo apt-get clean && \
      git clone https://github.com/frankaemika/franka_ros2.git /ros2_ws/src/franka_ros2 && \
      vcs import /ros2_ws/src < /ros2_ws/src/franka_ros2/franka.repos --recursive --skip-existing && \
      rosdep install --from-paths src/franka_ros2 --ignore-src --rosdistro=$ROS_DISTRO -y; \
    else \
      echo "Skipping franka_ros2 integration - building only the Cartesian Impedance Controller."; \
    fi

RUN /bin/bash src/Cartesian-Impedance-Controller/scripts/install_dependencies.sh

RUN echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> /home/$USERNAME/.bashrc

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

CMD ["/bin/bash"]
