#!/bin/bash

rootDir=$(dirname $0)
rootDir=$(cd ${rootDir}/.. && pwd)
targetDir=$(cd ${rootDir} && basename $(pwd))

if [[ ! -e /dev/input/rfid || ! -c /dev/input/rfid ]]
then
  echo "Did not find /dev/input/rfid"
fi

if [[ ! -e /dev/spidev0.0 || ! -c /dev/spidev0.0 ]]
then
  echo "Did not find /dev/spidev0.0"
fi

docker run --privileged -it --mount src="${rootDir}",target=/${targetDir},type=bind --mount src=/dev,target=/dev,type=bind ${targetDir}:latest /bin/bash
