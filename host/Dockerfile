FROM osrf/ros:dashing-desktop
RUN apt-get update && apt-get install -y \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src

COPY . ./../src

SHELL ["/bin/bash", "-c"]