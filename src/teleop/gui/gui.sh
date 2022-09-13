#!/bin/bash

#Ensure proper NODE_PATH
export NODE_PATH=/usr/lib/nodejs:/usr/share/nodejs

#Check for yarn vs yarnpkg
yarn_executable=yarn
yarn_executable_path=$(which "$yarn_executable")
yarnpkg_executable=yarnpkg
yarnpkg_executable_path=$(which "$yarnpkg_executable")

#Install web packages if no node_modules folder
if [ ! -d "./node_modules/" ] 
then
    if [ ! -z "$yarn_executable_path" ]; then
        yarn install
    elif [ ! -z "$yarnpkg_executable_path" ]; then
        yarnpkg install 
    else
        echo "Please run sudo apt install yarn"
        exit 1
    fi
fi


if [ ! -z "$yarn_executable_path" ]; then
    yarn run serve
elif [ ! -z "$yarnpkg_executable_path" ]; then
    yarnpkg run serve 
else
    echo "Please run sudo apt install yarn"
    exit 1
fi