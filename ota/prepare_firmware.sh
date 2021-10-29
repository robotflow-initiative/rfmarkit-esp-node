#!/bin/bash
PROJECT_NAME=esp32-imu-node
PWD=$(pwd)
PORT=5138
ROOT_DIR=..
BUILD_DIR=$ROOT_DIR/build/

set -e

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"


cd $DIR
echo "Changing to $DIR"

# Build new firmware if desired
if [[ $# -ge 1 && $1 = "build" ]]; then

cd ..
idf.py build
cd $DIR

fi

cp $BUILD_DIR/$PROJECT_NAME.bin ./firmware.bin

# Update version

cp $ROOT_DIR/version.txt ./version.txt

# python -c "f=open('version.txt', 'a+');f.seek(0);content=f.readline();ver=int(content.split('./')[0])+1 if content != '' else 1;f.truncate(0);f.write(str(ver)+'\n');f.close();"

# Start python http server

python -m http.server $PORT

# Change back to previous directory
cd $PWD