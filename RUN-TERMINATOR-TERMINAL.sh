#!/bin/bash

# terminatorを立ち上げるためのプログラム




mkdir -p ~/.config/terminator/
if [ ! -f ~/.config/terminator/config.backup ]; then
  cp ~/.config/terminator/config ~/.config/terminator/config.backup
fi

cp ./terminator_config/config ~/.config/terminator/config

if [ "${1}" = "zed" ]; then
    terminator -l rtabmap_zed --working-directory ${PWD} &
    sleep 1

elif [ "${1}" = "rs" ]; then
    terminator -l rtabmap_rs --working-directory ${PWD} &
    echo "rs"
    sleep 1
fi 


