#!/bin/sh

mkdir -p `python3 -m site --user-site`
ln -sf `pwd`/../../hr4c_api `python3 -m site --user-site`
