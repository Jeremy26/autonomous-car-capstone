@echo off
docker volume create --name sdcnd-capstone-volume-builder
docker build -f Dockerfile.builder -t vwiart/sdcnd-capstone .
docker run --rm -it ^
    -p 4567:4567 ^
    -v sdcnd-capstone-volume:/app/ros ^
    -v %cd%/ros/src:/app/ros/src:ro ^
    -v %cd%/ros/launch:/app/ros/launch:ro ^
    -v %cd%/data:/app/data ^
    -v %cd%/resources/run.sh:/app/run.sh:ro ^
    -v %cd%/logs:/root/.ros/log ^
    --name sdcnd-capstone-runner ^
    vwiart/sdcnd-capstone /bin/bash
