#!/bin/bash

rootDir=$(dirname $0)
rootDir=$(cd ${rootDir}/.. && pwd)
targetDir=$(cd ${rootDir} && basename $(pwd))

if [[ -e /dev/input/rfid && -c /dev/input/rfid ]]
then
  docker run -it --mount src="${rootDir}",target=/${targetDir},type=bind --device=/dev/input/rfid:/dev/input/rfid:r ${targetDir}:latest /bin/bash
else
  echo "Did not find /dev/input/rfid, skipping --device"
  docker run -it --mount src="${rootDir}",target=/${targetDir},type=bind ${targetDir}:latest /bin/bash
fi
