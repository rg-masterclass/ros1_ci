FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
  git

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

RUN mkdir -p /noetic_ws/src

RUN git clone --recursive https://github.com/rigbetellabs/tortoisebot.git -b noetic /noetic_ws/src/tortoisebot

COPY tortoisebot_waypoints/tortoisebot_waypoints /noetic_ws/src/tortoisebot_waypoints
COPY tortoisebot_waypoints/tortoisebot_waypoints_interface /noetic_ws/src/tortoisebot_waypoints_interface

ADD ros_entrypoint.sh /

WORKDIR /noetic_ws

RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /noetic_ws && catkin_make"

RUN echo "source /noetic_ws/devel/setup.bash" >> ~/.bashrc

RUN rm -f /usr/bin/python && ln -s /usr/bin/python3 /usr/bin/python

ENTRYPOINT ["bash", "/ros_entrypoint.sh"]
CMD ["bash"]
