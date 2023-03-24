#!/bin/bash
CURDIR=`pwd`
docker build -t kasm-jammy:dev -f docker/dockerfile-kasm-core-jammy .
docker build -t suave:dev --build-arg BASE_IMAGE=kasm-jammy:dev -f docker/dockerfile-kasm-ubuntu-jammy-desktop-minimal .
cd $CURDIR
