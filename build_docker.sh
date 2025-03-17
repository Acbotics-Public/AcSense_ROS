#!/usr/bin/env bash

git submodule update --init --recursive

if [[ ! $(docker buildx ls | grep multi-platform-builder ) ]]; then

    docker buildx create \
        --use --platform=linux/arm64,linux/amd64,linux/arm/v8,linux/arm/v7,linux/arm/v6 \
        --name multi-platform-builder \
        --driver-opt network=host \
        --config /etc/buildkit/buildkitd.toml \
        --bootstrap

fi

DOCKER_IMAGE_NAME=acsense_ros
DOCKER_IMAGE_URL=localhost:5000/${DOCKER_IMAGE_NAME}

VERSION=0.0.1

docker run --rm --privileged multiarch/qemu-user-static:register --reset 2&> /dev/null
sleep 1
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes 2&> /dev/null
sleep 1

# Execute x86_64 build and load locally
docker buildx build \
    --platform linux/amd64  \
    --file docker/humble.dockerfile \
    --tag ${DOCKER_IMAGE_NAME}:${VERSION} \
    --tag ${DOCKER_IMAGE_NAME}:latest \
    --load \
    .

# Execute multi-platform build and push to local / self-hosted registry
docker buildx build \
    --platform linux/amd64,linux/arm64  \
    --file docker/humble.dockerfile \
    --tag ${DOCKER_IMAGE_URL}:${VERSION} \
    --tag ${DOCKER_IMAGE_URL}:latest \
    --push \
    .

docker run --rm --privileged multiarch/qemu-user-static:register --reset 2&> /dev/null
sleep 1

echo "Done"

# docker build -f ./docker/humble.dockerfile -t acsense_ros .
