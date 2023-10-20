#!/bin/bash

FOLDER=$(dirname $0)
JOY_CONFIG_FILE=$FOLDER/config/joy_params.yaml
JOYSTICK_CONFIG_FILE=$FOLDER/config/joystick_params.yaml

# Configuration file
if test -f "$JOY_CONFIG_FILE"; then
    echo "$JOY_CONFIG_FILE already exists"
else
    echo "$JOY_CONFIG_FILE doesn't exist, creating"
    cp ${FOLDER}/config/joy_params.sample.yaml ${JOY_CONFIG_FILE}
fi

# Configuration file
if test -f "$JOYSTICK_CONFIG_FILE"; then
    echo "$JOYSTICK_CONFIG_FILE already exists"
else
    echo "$JOYSTICK_CONFIG_FILE doesn't exist, creating"
    cp ${FOLDER}/config/joystick_params.sample.yaml ${JOYSTICK_CONFIG_FILE}
fi

if [ $# -eq 0 ]; then
    echo "Build of ast_joystick. Building as latest"
    docker build -t ast_joystick:latest --build-arg NUM_THREADS=8 .
else
    echo "Build of ast_joystick. Building as $1"
    docker build -t ast_joystick:$1 --build-arg NUM_THREADS=8 --build-arg TAG=$1 .
fi