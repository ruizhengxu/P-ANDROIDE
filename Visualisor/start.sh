#!/bin/bash

sleep 20
rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"