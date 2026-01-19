FROM docker.io/ros:humble-ros-base as dev

# Install git for pulling the base repository
RUN apt update
RUN apt install -y \
	git \
	curl \
	lsb-release \
	gnupg

RUN curl http://packages.osrfoundation.org/gazebo.key | apt-key add -
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Install build dependencies
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt install -y \
	libgz-transport13 \
	libgz-transport13-dev \
	astyle \
	build-essential \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	libfuse2 \
	libxml2-dev \
	libxml2-utils \
	make \
	ninja-build \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	rsync \
	shellcheck \
	unzip \
	ros-humble-fastrtps \
	ros-humble-rmw-fastrtps-cpp \
	fastddsgen




# Copy source from host and install Python requirements
COPY ../Tools/setup/requirements.txt /requirements.txt
RUN python3 -m pip install -r /requirements.txt

