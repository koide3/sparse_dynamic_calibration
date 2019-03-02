#!/bin/bash
rosbag record -O $1 -e "/camera/(camera_info|image_raw/compressed)"
