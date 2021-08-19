# https://sdk.rethinkrobotics.com/wiki/Custom_IKFast_for_your_Baxter - follow instruction to generate IK file
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y lsb-release gnupg2 build-essential && apt-get clean all
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-noetic-desktop-full
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
# RUN "source /opt/ros/noetic/setup.bash"
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init
RUN rosdep update

RUN apt-get install -y ros-noetic-moveit ros-noetic-collada*
RUN apt-get install -y cmake-curses-gui
RUN mkdir -p repos 
WORKDIR "/repos"
RUN git clone https://github.com/Tencent/rapidjson.git
WORKDIR "/repos/rapidjson"
RUN mkdir build 
WORKDIR "/repos/rapidjson/build"
RUN cmake .. -DRAPIDJSON_BUILD_DOC=OFF -DRAPIDJSON_BUILD_EXAMPLES=OFF -DRAPIDJSON_BUILD_TESTS=OFF
RUN make && make install
WORKDIR "/repos"
RUN git config --global user.name "FIRST_NAME LAST_NAME" && git config --global user.email "MY_NAME@example.com"
RUN git clone https://github.com/crigroup/openrave-installation
WORKDIR /repos/openrave-installation
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2 get-pip.py 
RUN pip install pybind11 lxml
RUN /repos/openrave-installation/install-dependencies.sh
RUN /repos/openrave-installation/install-osg.sh
RUN /repos/openrave-installation/install-fcl.sh
RUN /repos/openrave-installation/install-openrave.sh
WORKDIR "/"
