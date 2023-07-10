#!/bin/bash
set -x
NEW_EXE_FILE=/home/hr4c/hr4c_core/bin/hr4c_core_rt.new
if [ -e $NEW_EXE_FILE ]; then
    echo "New exe file exists. Replace it."
    rm /home/hr4c/hr4c_core/bin/hr4c_core_rt
    mv /home/hr4c/hr4c_core/bin/hr4c_core_rt.new /home/hr4c/hr4c_core/bin/hr4c_core_rt
fi
NEW_CONFIG_FILE=/home/hr4c/hr4c_core/bin/config.yaml.new
if [ -e $NEW_CONFIG_FILE ]; then
    echo "New config file exists. Replace it."
    rm /home/hr4c/hr4c_core/bin/config.yaml
    mv /home/hr4c/hr4c_core/bin/config.yaml.new /home/hr4c/hr4c_core/bin/config.yaml
fi
byobu list-sessions | grep startup || byobu new-session -d -s startup
byobu list-windows -t startup | grep hr4c_core || byobu new-window -t startup -n 'hr4c_core'
byobu send-keys -t startup:hr4c_core "sleep 10; cd /home/hr4c/hr4c_core/bin/; sudo ./hr4c_core_rt" C-m
