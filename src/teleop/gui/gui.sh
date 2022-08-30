#!/bin/bash

#Ensure proper working directory
roscd mrover/src/teleop/gui
#Ensure proper NODE_PATH
export NODE_PATH=/usr/lib/nodejs:/usr/share/nodejs

#Install web packages if no node_modules folder
if [ ! -d "./node_modules/" ] 
then
    yarnpkg install
fi

yarnpkg run serve