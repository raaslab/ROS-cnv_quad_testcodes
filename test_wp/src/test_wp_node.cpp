#include "ros/ros.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "opencv2/core/core.hpp"
#include <cstdlib>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_wp");


  ros::NodeHandle n;
  ros::ServiceClient wp_client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
  mavros_msgs::WaypointPush wp;

  mavros_msgs::Waypoint curr_wp ;

  curr_wp.frame = 3;
  curr_wp.command = 16;
  curr_wp.is_current = (bool)true;
  curr_wp.autocontinue = (bool)true;
  curr_wp.param1 = 0.0;
  curr_wp.param2 = 5.0;
  curr_wp.param3 = -0.0;
  curr_wp.param4 = 0.0;
  curr_wp.x_lat = 37.228405;
  curr_wp.y_long = -80.422354;
  curr_wp.z_alt = 50;



  wp.request.waypoints.push_back(curr_wp);

/*  wp.request.waypoints[0].command = 16;
  wp.request.waypoints[0].is_current = (bool)true;
  wp.request.waypoints[0].autocontinue = (bool)true;
  wp.request.waypoints[0].param1 = 0.0;
  wp.request.waypoints[0].param2 = 5.0;
  wp.request.waypoints[0].param3 = -0.0;
  wp.request.waypoints[0].param4 = 0.0;
  wp.request.waypoints[0].x_lat = 37.228405;
  wp.request.waypoints[0].y_long = -80.422354;
  wp.request.waypoints[0].z_alt = 50;*/
  //srv.request.a = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);
/*
  "waypoints:
  - {frame: 3, command: 16, is_current: True, autocontinue: True, param1: 0.0, param2: 5.0,
   param3: -0.0, param4: 0.0, x_lat: 37.228405, y_long: -80.422964, z_alt: 30}"
*/

  wp_client.call(wp);
/*
  if (wp.response.success)
  {
    ROS_INFO("wp_transfered: %ld", (uint)(wp.response.wp_transfered));
  }
  else
  {
    ROS_ERROR("Failed to call");
    return 1;
  }*/

  return 0;
}


