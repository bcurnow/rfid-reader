#!/bin/bash

rootDir=$(dirname $0)
rootDir=$(cd ${rootDir}/.. && pwd)
targetDir=$(cd ${rootDir} && basename $(pwd))

docker run -it -p 5000:5000 --mount src="${rootDir}",target=/${targetDir},type=bind --device=/dev/input/rfid:/dev/input/rfid:rwm ${targetDir}:latest /bin/bash
