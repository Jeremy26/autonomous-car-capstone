#!/bin/bash
docker volume create sdcnd-capstone-volume-builder
docker build -f Dockerfile.builder -t vwiart/sdcnd-capstone-builder .
docker run --rm -it \
    -v sdcnd-capstone-volume-builder:/app/ros \
    -v %cd%/ros/src:/app/ros/src:ro \
    -v %cd%/ros/launch:/app/ros/launch:ro \
    --name sdcnd-capstone-builder \
    vwiart/sdcnd-capstone-builder /bin/bash