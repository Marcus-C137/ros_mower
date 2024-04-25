#!/bin/bash
docker build -t mower:latest .
docker run -it --net=host --rm -v ./..:/src/ws ros:dashing /bin/bash 