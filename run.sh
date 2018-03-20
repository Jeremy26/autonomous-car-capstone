#!/bin/bash
docker volume create sdcnd-capstone-volume
docker build -f Dockerfile.builder -t vwiart/sdcnd-capstone .
docker run --rm -it \
    -p 4567:4567 \
    -v sdcnd-capstone-volume:/app/ros \
    -v $(pwd)/ros/src:/app/ros/src:ro \
    -v $(pwd)/ros/launch:/app/ros/launch:ro \
    -v $(pwd)/data:/app/data \
    -v $(pwd)/resources/run.sh:/app/run.sh:ro \
    --name sdcnd-capstone-runner \
    vwiart/sdcnd-capstone /bin/bash
