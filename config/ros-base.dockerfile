# ------------------------------------------------------------------------------
# Create the base image from the ROS2 Docker image
# ------------------------------------------------------------------------------
# Use the osrf/ros2 as base: https://hub.docker.com/r/osrf/ros2/
#
# Because it is a nightly/experimental build, we pulled a version
# then stored a local copy of the image in the Tech Adv PC:
#
#   "Id": "sha256:d91028452cb46f9214dfd78e5e71d70667753206eeb1a911b54c786652f6a0d6",
#   "RepoTags": [
#       "localhost:5006/ros2:nightly"
#   "Created": "2019-10-18T00:36:34.582204186Z",
FROM 192.168.1.65:5006/ros2:nightly

# ------------------------------------------------------------------------------
# Setup the image's identity
# ------------------------------------------------------------------------------
# TODO
#  - Make the name consistent with the service name
#  - Replace the hardcoded version with a version.txt
LABEL NAME="service" \
      VERSION="0.0.1" \
      DESC="MyApp"

# ------------------------------------------------------------------------------
# Setup all the environment dependencies
# ------------------------------------------------------------------------------
# For the APT keys, get it from the osrf/ros2 Dockerfile.
# https://hub.docker.com/r/osrf/ros2/dockerfile
ENV DEBIAN_FRONTEND=noninteractive
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && \
    apt-get install -y apt-utils debconf-utils && \
    apt-get install -y software-properties-common && \
    apt-get install -y build-essential && \
    apt-get install -y python3.7 python3.7-dev && \
    apt-get install -y --reinstall locales && \
    apt-get install -y less tree && \
    apt-get install -y usbutils && \
    python3.7 -m pip install pip --upgrade && \
    python3.7 -m pip install wheel && \
    export LANGUAGE=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    export LC_ALL="" && \
    locale-gen en_US.UTF-8 && \
    dpkg-reconfigure locales
ENV PY python3.7

# ------------------------------------------------------------------------------
# Setup rti-connect-dds-5.3.1
# ------------------------------------------------------------------------------
# Reference:
#   https://github.com/ros2/ros2_documentation/blob/master/source/Installation/Linux-Development-Setup.rst#id8
#
# It is listed as an "optional" dependency of ROS2, but running the node without it produces
# the following warning:
#   [connext_cmake_module] Warning: The location at which Connext was found when the workspace was built
#       [[/opt/rti.com/  rti_connext_dds-5.3.1]] does not point to a valid directory, and the NDDSHOME
#       environment variable has not been set. Support for Connext will not be available.
#
# Note that this install step takes a while to complete (~30mins.).
# If you are OK with seeing lots of warnings, comment these out.
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get install -y -q rti-connext-dds-5.3.1

# ------------------------------------------------------------------------------
# Setup the app
# ------------------------------------------------------------------------------
ENV WS /ws/ros-base
RUN test -d ${WS} || mkdir -pv ${WS}
WORKDIR ${WS}
COPY src/ros-base/requirements.txt ./
COPY src/ros-base/run-node.sh ./
COPY src/ros-base/pkgs ./
RUN tree -L 3 ${WS}

# ------------------------------------------------------------------------------
# Setup all the app-specific dependencies
# ------------------------------------------------------------------------------
WORKDIR ${WS}
RUN ${PY} -m pip install -r requirements.txt
# ..add app-specific setup steps here..

# ------------------------------------------------------------------------------
# Build the ROS nodes
# ------------------------------------------------------------------------------
WORKDIR ${WS}
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
CMD ${PY}
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash; colcon build;"

# ------------------------------------------------------------------------------
# Setup container entrypoints
# ------------------------------------------------------------------------------
WORKDIR ${WS}
RUN ln -s ${PWD}/run-node.sh /run-node.sh
RUN chmod +x /run-node.sh
STOPSIGNAL SIGTERM
