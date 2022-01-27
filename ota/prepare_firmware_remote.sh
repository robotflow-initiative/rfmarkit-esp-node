#!/bin/bash
PROJECT_NAME=imu-esp-node
PWD=$(pwd)
ROOT_DIR=..
BUILD_DIR=$ROOT_DIR/build/

# Remote deployment
REMOTE_HOST=speit@10.53.21.164
REMOTE_PORT=5138
REMOTE_DIR=/home/speit/imu-node-deploy/ota/
REMOTE_PYTHON=/usr/bin/python3
# Remote python must have http module

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
echo "Cache firmware"
cp $BUILD_DIR/$PROJECT_NAME.bin ./firmware.bin
cp $ROOT_DIR/version.txt ./version.txt

# Upload to remote
echo "Uploading to remote"
scp ./firmware.bin ./version.txt $REMOTE_HOST:$REMOTE_DIR

# Start python http server on the remote
echo "Uploading to remote"
ssh $REMOTE_HOST -t "cd $REMOTE_DIR; $REMOTE_PYTHON -m http.server $REMOTE_PORT"

# python -m http.server $REMOTE_PORT

# Change back to previous directory
cd $PWD