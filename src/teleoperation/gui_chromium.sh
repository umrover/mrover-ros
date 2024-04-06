#!/bin/bash

# Our stream is encoded with the HVEC (H.265) codec
# Chromium currently only supports this when using VA-API hardware acceleration
# It uses the WebCodecs API to decode
# You can easily test if your setup works with this URL: https://w3c.github.io/webcodecs/samples/video-decode-display/
chromium --enable-features=VaapiVideoDecodeLinuxGL --app=http://localhost:8080
