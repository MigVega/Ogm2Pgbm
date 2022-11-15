FROM osrf/ros:melodic-desktop-full

ARG user_home=/root
ARG ck_dir=$user_home/catkin_ws
ARG ck_src_dir=$ck_dir/src

COPY ros_entrypoint.sh /

RUN true \
	&& echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
	&& echo "source $ck_dir/devel_isolated/setup.bash" >> ~/.bashrc \
        && mkdir -p $ck_dir

# install cartographer according to the page(https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)
RUN apt-get update \
    && apt-get install -y ninja-build stow vim ros-melodic-map-server 


WORKDIR $ck_dir
#-----
#RUN wstool init src \
#    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
#    && wstool update -t src 
#-----
# Instead of directly cloning using modified vecrsion
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y git

ADD cartographer_git $ck_dir
#-----

RUN sudo rosdep fix-permissions \   
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 
RUN ./src/cartographer/scripts/install_abseil.sh \
    && . /opt/ros/melodic/setup.sh \ 
    && catkin_make_isolated --install --use-ninja

ADD cartographer_ros $ck_dir/src/cartographer_ros/cartographer_ros

# Prepare ENV and start running
WORKDIR $ck_dir
ENV CATKIN_WS $ck_dir
#ENV NVIDIA_VISIBLE_DEVICES \
#    ${NVIDIA_VISIBLE_DEVICES:-all}
#ENV NVIDIA_DRIVER_CAPABILITIES \
#    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# add package gmcl

RUN cd src \
    && git clone https://github.com/adler-1994/gmcl.git \
    && git clone --branch melodic-devel https://github.com/ros-planning/navigation.git 

ADD gmcl $ck_dir/src/gmcl
ADD amcl $ck_dir/src/navigation/amcl
RUN . /opt/ros/melodic/setup.sh \ 
    && apt-get update && apt-get install -y ros-melodic-costmap-2d \
    && mv src/navigation/amcl src \
    && rm -rf src/navigation \
    && catkin_make_isolated --install --use-ninja

RUN apt-get install python-pip -y && pip install evo --upgrade --no-binary evo

# Add package ogm2pgbm
RUN apt-get update && apt-get install -y python-tk ros-melodic-slam-toolbox bc && pip install scikit-image
ADD ogm2pgbm $ck_dir/src/ogm2pgbm
ADD ogm2pgbm/demobag /root/.ros/
RUN . /opt/ros/melodic/setup.sh && catkin_make_isolated --install --use-ninja --pkg ogm2pgbm

# Add package slam_toolbox
ADD slam_toolbox /opt/ros/melodic/share/slam_toolbox/
ADD slam_toolbox/basemap /root/.ros/
COPY slam_toolbox/preprocessing/bag.py /opt/ros/melodic/lib/python2.7/dist-packages/rosbag/bag.py

# Add package evo and pdflatex engine
RUN apt-get update && apt-get install -y texlive-latex-base texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra
#ADD evo /usr/local/lib/python2.7/dist-packages/evo # not available in repoo
RUN apt-get install -y libnotify-bin dvipng && pip install ruamel.yaml==0.15.1 \
    && git clone https://github.com/uzh-rpg/rpg_trajectory_evaluation.git ~/catkin_ws/src/rpg_trajectory_evaluation \
    && git clone https://github.com/catkin/catkin_simple.git ~/catkin_ws/src/catkin_simple

# Add package gmcl_carto and post processing scripts 
ADD gmcl_carto $ck_dir/src/gmcl_carto
#ADD post_processing $ck_dir/src/post_processing # not available in repo

RUN chmod -R 777 ~/catkin_ws/src/ogm2pgbm/scripts
RUN chmod -R 777 ~/catkin_ws/install_isolated/share/ogm2pgbm/scripts

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

CMD ["/bin/bash" "-c" "/bin/bash"] 
