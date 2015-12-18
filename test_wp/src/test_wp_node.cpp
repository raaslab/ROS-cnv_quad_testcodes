/*COMMAND FIELD	MISSION PLANNER FIELD	DESCRIPTION
param1	Delay	Hold time at mission waypoint in decimal seconds – MAX 65535 seconds. (Copter/Rover only)
param2	Acceptance radius in meters (when plain inside the sphere of this radius, the waypoint is considered reached) (Plane only).
param3	0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
param4	Desired yaw angle at waypoint target.(rotary wing)
param5	Lat	Target latitude. If zero, the Copter will hold at the current latitude.
param6	Lon	Target longitude. If zero, the Copter will hold at the current longitude.
param7	Alt	Target altitude. If zero, the Copter will hold at the current altitude.*/

/*# see enum MAV_FRAME
uint8 frame
uint8 FRAME_GLOBAL = 0
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_MISSION = 2
uint8 FRAME_GLOBAL_REL_ALT = 3
uint8 FRAME_LOCAL_ENU = 4*/

/*
 * # http://mavlink.org/messages/common#ENUM_MAV_CMD

# some common MAV_CMD
uint16 CMD_DO_SET_MODE = 176
uint16 CMD_DO_JUMP = 177
uint16 CMD_DO_CHANGE_SPEED = 178
uint16 CMD_DO_SET_HOME = 179
uint16 CMD_DO_SET_RELAY = 181
uint16 CMD_DO_REPEAT_RELAY = 182
uint16 CMD_DO_SET_SERVO = 183
uint16 CMD_DO_REPEAT_SERVO = 184
uint16 CMD_DO_CONTROL_VIDEO = 200
uint16 CMD_DO_SET_ROI = 201
uint16 CMD_DO_MOUNT_CONTROL = 205
uint16 CMD_DO_SET_CAM_TRIGG_DIST = 206
uint16 CMD_DO_FENCE_ENABLE = 207
uint16 CMD_DO_PARACHUTE = 208
uint16 CMD_DO_INVERTED_FLIGHT = 210
uint16 CMD_DO_MOUNT_CONTROL_QUAT = 220
uint16 CMD_PREFLIGHT_CALIBRATION = 241
uint16 CMD_MISSION_START = 300
uint16 CMD_COMPONENT_ARM_DISARM = 400
uint16 CMD_START_RX_PAIR = 500
uint16 CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
uint16 CMD_DO_TRIGGER_CONTROL = 2003

# Waypoint related commands
uint16 NAV_WAYPOINT = 16
uint16 NAV_LOITER_UNLIM = 17
uint16 NAV_LOITER_TURNS = 18
uint16 NAV_LOITER_TIME = 19
uint16 NAV_RETURN_TO_LAUNCH = 20
uint16 NAV_LAND = 21
uint16 NAV_TAKEOFF = 22
 *
 */


#include "ros/ros.h"
#include <mavros_msgs/WaypointPush.h>
#include "mavros_msgs/Waypoint.h"
#include "opencv2/core/core.hpp"
#include <cstdlib>
#include "geodetic_conv.hpp"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_wp");


	ros::NodeHandle n;
	ros::ServiceClient wp_client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
	mavros_msgs::WaypointPush wp;
	mavros_msgs::Waypoint curr_wp[1];
	geodetic_converter::GeodeticConverter g_geodetic_converter;
	double x[4] = {50,50,0,0};
	double y[4] = {0,50,50,0};
	double z = 15;
	double lat, lon, alt;

	curr_wp[0].frame = 3;
	curr_wp[0].command = 16;
	curr_wp[0].is_current = (bool)false;
	curr_wp[0].autocontinue = (bool)true;
	curr_wp[0].param1 = 2.0;
	curr_wp[0].param2 = 5.0;
	curr_wp[0].param3 = -0.0;
	curr_wp[0].param4 = 0.0;
	curr_wp[0].x_lat = 0;   // will provide in the for loop below
	curr_wp[0].y_long = 0;
	curr_wp[0].z_alt = 0;

	g_geodetic_converter.initialiseReference(37.228405, -80.422964, 630);
	//wp.request.waypoints.push_back(curr_wp[0]);

	//curr_wp[0].z_alt = 100;



	for (int i = 0; i < 4; i++)
	{
		g_geodetic_converter.enu2Geodetic(x[i], y[i], z+630, &lat, &lon, &alt);
		curr_wp[0].x_lat = lat;
		curr_wp[0].y_long = lon;
		//alt = alt - 630;
		curr_wp[0].z_alt = 17;
		if(i==0)
		{
			curr_wp[0].is_current = (bool)true;
		}
		else
		{
			curr_wp[0].is_current = (bool)false;
		}
		wp.request.waypoints.push_back(curr_wp[0]);
	}




	/*
	"waypoints:
	- {frame: 3, command: 16, is_current: True, autocontinue: True, param1: 0.0, param2: 5.0,
	param3: -0.0, param4: 0.0, x_lat: 37.228405, y_long: -80.422964, z_alt: 30}"
	*/





	//g_geodetic_converter.geodetic2Enu(37.228405, -80.422964, 660, &x, &y, &z);

	g_geodetic_converter.enu2Geodetic(0, 0, 0, &lat, &lon, &alt);

	 float offset_x = 0.0;
	 float offset_y = 0.0;
	 float  offset_z = 10.0;
	 float sides = 360;
	 float radius = 20;



	//ROS_INFO("GPS home enu %f, %f, %f", x, y, z);
	ROS_INFO("home %f, %f, %f", lat, lon, alt);
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

	wp_client.call(wp);


	return 0;
}

