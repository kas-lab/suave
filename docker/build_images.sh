#!/bin/bash
CURDIR=`pwd`
DIR=`dirname $0`
docker build -t kasm_jammy:dev -f ./dockerfile-kasm-core-jammy .
docker build -t suave:dev --build-arg BASE_IMAGE=kasm-jammy:dev -f ./dockerfile-kasm-ubuntu-jammy-desktop-minimal .
cd $CURDIR
