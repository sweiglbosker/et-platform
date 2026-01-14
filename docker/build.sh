#!/bin/bash

set -e
set -x

docker build --build-arg BUILD_JOBS=$(nproc) -t et-platform -f docker/Dockerfile .
