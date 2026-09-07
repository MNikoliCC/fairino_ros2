FROM osrf/ros:humble-desktop

ARG UID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -d /dockeruser -m \
            -u $UID -U \
            -s /usr/bin/bash \
            -c "Docker User" dockeruser && \
    echo 'dockeruser ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    usermod -a -G video dockeruser && \
    usermod -a -G dialout dockeruser && \
    usermod -a -G plugdev dockeruser


# COPY cyclonedds.xml /etc/cyclonedds.xml
# ENV CYCLONEDDS_URI=/etc/cyclonedds.xml
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /dockeruser/.bashrc

USER dockeruser
WORKDIR /dockeruser/ros2_ws