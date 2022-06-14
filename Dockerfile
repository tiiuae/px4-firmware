FROM ros:galactic-ros-base AS builder

ENV LANG C.UTF-8
ENV LANGUAGE C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential git wget cmake lsb-core ninja-build \
    ros-galactic-gazebo-ros \
    libopencv-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-jinja2 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone -q https://github.com/mavlink/c_library_v2.git /usr/local/include/mavlink/v2.0 && \
    cd /usr/local/include/mavlink/v2.0 && git checkout -q f03d91ae932dd718d81f851b584dbd21cffafdd2 && \
    rm -rf /usr/local/include/mavlink/v2.0/.git

WORKDIR /px4_sitl_gazebo
COPY . .

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/galactic/setup.bash && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j $(nproc)

WORKDIR /artifacts

RUN mkdir -p plugins \
  && mkdir -p models \
  && mkdir -p scripts \
  && cp -r /px4_sitl_gazebo/models/ssrc_fog_x models/ssrc_fog_x \
  && cp /px4_sitl_gazebo/scripts/jinja_gen.py scripts/jinja_gen.py \
  && find /px4_sitl_gazebo/build/*.so -exec cp {} plugins \;


FROM busybox

COPY --from=builder /artifacts /artifacts
