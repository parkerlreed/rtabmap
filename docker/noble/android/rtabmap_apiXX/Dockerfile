# Image: introlab3it/rtabmap:androidXX

# The dependencies image built by ../Dockerfile. Overridable so that a locally
# rebuilt deps image (e.g. one with an extra dependency) can be used without
# having to re-tag it.
ARG BASE_IMAGE=introlab3it/rtabmap:android-noble-deps
FROM ${BASE_IMAGE}

ARG API_VERSION=23

# Copy current source code
COPY . /root/rtabmap-tango

WORKDIR /root/rtabmap-tango

RUN /bin/bash -c "./docker/noble/android/rtabmap_apiXX/rtabmap.bash /opt/android $API_VERSION"
