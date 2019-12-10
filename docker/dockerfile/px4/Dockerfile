# Specify base image
FROM px4_sitl_docker_base:latest

# Setup timezone
USER root
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# Upgrade all existing packages
RUN set -x \
        && apt-get update \
        && apt-get upgrade -y

# Add some packages we need later on
RUN set -x \
        && apt-get install -y apt-transport-https ca-certificates git vim sudo htop curl wget mesa-utils \
        && apt-get install -y software-properties-common libnss3 dirmngr gnupg2 lsb-release tmux apt-utils \
        && rm -rf /var/lib/apt/lists/*

# Setup so that no password is needed when sudoing
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Setup and install ROS Melodic
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update \
        && apt-get install -y ros-melodic-desktop-full \
        && apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential \
        && rm -rf /var/lib/apt/lists/* \
        && rosdep init

# Create a new user named "user" en add it to the sudoers
RUN useradd -ms /bin/bash user \
    && echo "user:user" | chpasswd && adduser user sudo \
    && usermod -aG audio user

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install gazebo9 libignition-math2 python-jinja2 libignition-math2-dev protobuf-compiler \
 libeigen3-dev libopencv-dev build-essential genromfs ninja-build exiftool \
 astyle python-argparse python-empy python-toml python-numpy python-dev python-pip \
 ros-melodic-mavros ros-melodic-mavros-extras python-catkin-tools python-rosinstall-generator -y

# Upgrade and install some packages
RUN apt-get upgrade libignition-math2-dev -y
RUN pip install --upgrade pip
RUN pip install pandas jinja2 pyserial pyyaml pymavlink

# Switch to user "user" and set working directory to it's home directory
USER user
WORKDIR /home/user

# Setup ROS
RUN rosdep update \
        && echo "source /opt/ros/melodic/setup.bash" >> /home/user/.bashr

# Make some required folders
WORKDIR /home/user
RUN mkdir -p Projects/catkin_ws/src 
RUN mkdir -p Programs/PyCharm
USER root
RUN mkdir -p /opt/PXL/catkin_ws/src

# Clone mavlink/mavros
RUN git clone https://github.com/mavlink/mavros.git /opt/PXL/catkin_ws/src/mavros
RUN /bin/bash -c 'cd /opt/PXL/catkin_ws/src/mavros ; git checkout 0.31.0; cd ~'
RUN bash /opt/PXL/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Clone PX4/Firmware
USER user
RUN git clone https://github.com/PX4/Firmware.git ~/Programs/Firmware
RUN /bin/bash -c 'cd ~/Programs/Firmware; git checkout 34b03d56593815e16a29ba5dcef9b66208285c98; cd ~'

# Clone subrepositories 
WORKDIR /home/user/Programs/Firmware
RUN git submodule update --init --recursive

# Run catkin_make
USER root
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /opt/PXL/catkin_ws; catkin build'
USER user
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/Projects/catkin_ws; catkin build'

# Build sitl code
USER user
WORKDIR /home/user/Programs/Firmware
RUN DONT_RUN=1 make px4_sitl_default gazebo

# Add setup.bash file to .bashrc of user "user" (environment source)
RUN echo "source /opt/PXL/catkin_ws/devel/setup.bash" >> /home/user/.bashrc
RUN echo "source /home/user/Projects/catkin_ws/devel/setup.bash --extend" >> /home/user/.bashrc

# Add setup_gazebo.bash file to .bashrc of user "user" (environment source)
USER root
COPY ./scripts/setup_gazebo.bash /scripts/setup_gazebo.bash
RUN ["chmod", "+x", "/scripts/setup_gazebo.bash"]
USER user
RUN echo "source /scripts/setup_gazebo.bash" >> /home/user/.bashrc
COPY ./scripts/ros_package_path.bash /scripts/ros_package_path.bash
RUN echo "source /scripts/ros_package_path.bash" >> /home/user/.bashrc

# Setup PyCharm
USER user
WORKDIR /home/user/Programs/PyCharm
RUN wget https://download.jetbrains.com/python/pycharm-community-2019.1.3.tar.gz
RUN tar xvf ./pycharm-community-2019.1.3.tar.gz
RUN rm ./pycharm-community-2019.1.3.tar.gz

# Setup tmux
RUN git clone https://github.com/jimeh/tmux-themepack.git ~/.tmux-themepack  \
&& git clone https://github.com/tmux-plugins/tmux-resurrect ~/.tmux-resurrect
COPY ./.tmux.conf /home/user/.tmux.conf

# Setup serial connection
USER root
RUN usermod -a -G dialout user
RUN apt-get remove modemmanager -y
RUN apt-get update
RUN apt install gstreamer1.0-plugins-bad gstreamer1.0-libav libfuse2 fuse libpulse-mainloop-glib0 -y

# Setup QGroundControl
RUN wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
RUN chmod +x ./QGroundControl.AppImage

USER user
WORKDIR /home/user
RUN cd Projects
RUN cd Projects/catkin_ws/src && git clone https://github.com/PXLRoboticsLab/PX4_Control.git && cd /home/user/Projects/catkin_ws && catkin build

USER root
# Set some decent colors if the container needs to be accessed via /bin/bash.
RUN echo LS_COLORS=$LS_COLORS:\'di=1\;33:ln=36\' >> ~/.bashrc \
&& echo export LS_COLORS >> ~/.bashrc \
&& touch ~/.sudo_as_admin_successful # To surpress the sudo message at run.

# Set initial commands
USER root
COPY ./scripts/init_commands.sh /scripts/init_commands.sh
RUN ["chmod", "+x", "/scripts/init_commands.sh"]

# Complete building process of docker container
USER user
STOPSIGNAL SIGTERM
ENTRYPOINT ["/scripts/init_commands.sh"]
CMD /bin/bash

