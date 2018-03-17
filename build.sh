#!/bin/bash
docker volume create sdcnd-capstone-volume-builder
docker build -f Dockerfile.builder -t vwiart/sdcnd-capstone-builder .
docker run --rm -it \
    -v sdcnd-capstone-volume-builder:/app/ros \
    -v $(pwd)/ros/src:/app/ros/src:ro \
    -v $(pwd)/ros/launch:/app/ros/launch:ro \
    -v $(pwd)/resources/build.sh:/app/build.sh:ro \
    --name sdcnd-capstone-builder \
    vwiart/sdcnd-capstone-builder /bin/bash