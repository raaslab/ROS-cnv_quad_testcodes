#!/bin/bash
#rosservice call /mavros/mission/push "waypoints:
#- {frame: 3, command: 16, is_current: True, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.228405, y_long: -80.422964, z_alt: 30}
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.228973, y_long: -80.420215, z_alt: 30}
#
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.226944, y_long: -80.421159, z_alt: 30}
#
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.226282, y_long: -80.424115, z_alt: 30}" 
#
rosservice call /mavros/mission/push "waypoints:
- {frame: 3, command: 16, is_current: True, autocontinue: True, param1: 0.0, param2: 5.0,
 param3: -0.0, param4: 0.0, x_lat: 37.228405, y_long: -80.422964, z_alt: 30}"



#rosservice call /mavros/mission/push "waypoints:
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.228973, y_long: -80.420215, z_alt: 30}" 
#
#
#rosservice call /mavros/mission/push "waypoints:
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.226944, y_long: -80.421159, z_alt: 30}" 
#
#
#rosservice call /mavros/mission/push "waypoints:
#- {frame: 3, command: 16, is_current: false, autocontinue: True, param1: 0.0, param2: 5.0,
#  param3: -0.0, param4: 0.0, x_lat: 37.226282, y_long: -80.424115, z_alt: 30}" 





#37.228405, -80.422964
#37.228973, -80.420215
#37.226944, -80.421159
#37.226282, -80.424115
# command 16 means : MAV_CMD_NAV_WAYPOINT
# frame :MAV_FRAME_GLOBAL_RELATIVE_ALT
#    <entry value="16" name="MAV_CMD_NAV_WAYPOINT">
##<description>Navigate to MISSION.</description>
##index="1">Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)</param>
##index="2">Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)</param>
##index="3">0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.</param>
##index="4">Desired yaw angle at MISSION (rotary wing)</param>
##index="5">Latitude</param>
##index="6">Longitude</param>
##index="7">Altitude</param>
#me: 3
#    command: 16
#    is_current: True
#    autocontinue: True
#    param1: 0.0
#    param2: 5.0
#    param3: -0.0
#    param4: 0.0
#    x_lat: 37.2284088135
#    y_long: -80.422958374
#    z_alt: 30.0

