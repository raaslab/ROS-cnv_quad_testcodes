#!/bin/bash
ROS_NAMESPACE=/ardrone/front/ rosrun image_proc image_proc &

rosbag record -O ~/bag_ardroneimage_rect /ardrone/front/image_raw /ardrone/front/camera_info /ardrone/front/image_rect_color /ardrone/front/image_rect


