#!/usr/bin/env bash

user_id=$(id -u)
image_name=ign-gazebo-base
image_plus_tag=$image_name:$(date +%Y_%b_%d_%H%M)

docker build --rm -t $image_plus_tag --build-arg user_id=$user_id -f ./docker/Dockerfile.base .
docker tag $image_plus_tag $image_name:latest
