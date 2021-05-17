#!/bin/bash

# terminatorを立ち上げるためのプログラム




mkdir -p ~/.config/terminator/
if [ ! -f ~/.config/terminator/config.backup ]; then
  cp ~/.config/terminator/config ~/.config/terminator/config.backup
fi

cp ./terminator_config/config ~/.config/terminator/config


if [ "${1}" = "zed" ]; then
    terminator -l mapping_rtabmap2 --working-directory ${PWD} &
    sleep 1
fi 


