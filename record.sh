#!/bin/bash

rosbag record -a --exclude="/iiwa/state/(.*)"
