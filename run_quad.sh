#!/bin/bash
ROS_NAMESPACE=/ardrone/front/ rosrun image_proc image_proc &

rosrun ardrone_autonomy ardrone_driver 

#rosrun hough_test hough_test_node &

	



