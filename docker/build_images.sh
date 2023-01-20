#!/bin/bash
CURDIR=`pwd`
DIR=`dirname $0`
cd $DIR/core-image/
docker build -t egalberts/jammy:1.0.1 -f ./dockerfile-kasm-core-jammy .
cd ..
docker build -t egalberts/desktop-jammy-minimal:dev -f ./dockerfile-kasm-ubuntu-jammy-desktop-minimal .
cd $CURDIR
