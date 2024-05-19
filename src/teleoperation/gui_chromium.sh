#!/bin/bash

# Our stream is encoded with the HVEC (H.265) codec
# Chromium currently only supports this when using VA-API hardware acceleration
# It uses the WebCodecs API to decode
# You can easily test if your setup works with this URL: https://w3c.github.io/webcodecs/samples/video-decode-display/
readonly FLAGS="--silent-launch --enable-features=VaapiVideoDecodeLinuxGL"
readonly ADDRESS="http://localhost:8080"
chromium ${FLAGS} --app=${ADDRESS}/$1
