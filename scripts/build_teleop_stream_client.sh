#!/bin/bash

source deps/emsdk/emsdk_env.sh
emcmake cmake -B src/teleoperation/streaming/embuild -G Ninja -DCMAKE_BUILD_TYPE=Release src/teleoperation/streaming/
cmake --build src/teleoperation/streaming/embuild
