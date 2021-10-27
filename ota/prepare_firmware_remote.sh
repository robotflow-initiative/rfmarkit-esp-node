#!/bin/bash
PROJECT_NAME=esp32-imu-node
PWD=$(pwd)
PORT=5138
ROOT_DIR=..
BUILD_DIR=$ROOT_DIR/build/

# Remote deployment
REMOTE_HOST=liyutong@10.52.21.125
REMOTE_DIR=/home/liyutong/Tasks/imu-node-deploy/ota/
REMOTE_PYTHON=/home/liyutong/miniconda3/envs/default/bin/python
# Remote python must have http module
# TODO: Use apache image to serve firmware

set -e

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

# Change to script directory
cd $DIR
echo "Changing to $DIR"

# Build new firmware if desired
if [[ $# -ge 1 && $1 = "build" ]]; then

cd ..
idf.py build
cd $DIR

fi

# Copy & Rename
cp $BUILD_DIR/$PROJECT_NAME.bin ./firmware.bin
cp $ROOT_DIR/version.txt ./version.txt

# Upload to remote
scp ./firmware.bin ./version.txt $REMOTE_HOST:$REMOTE_DIR

# Start python http server on the remote

ssh $REMOTE_HOST -t "cd $REMOTE_DIR; $REMOTE_PYTHON -m http.server $PORT"

# python -m http.server $PORT

# Change back to previous directory
cd $PWD