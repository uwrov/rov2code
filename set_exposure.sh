#!/bin/bash

v4l2-ctl -d 4 -c exposure_auto=1,exposure_absolute=$1,brightness=0,gain=100
