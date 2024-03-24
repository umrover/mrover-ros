uint8 NO_FIX = 0
uint8 FLOATING_FIX = 1
uint8 RTK_FIX = 2

std_msgs/Header header
sensor_msgs/NavSatFix coord
uint8 fix_type