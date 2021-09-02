###############################################################################
# Type of path that should be generated from current 
# arm pose to desired end effector pose
uint8 PATH = 0  # end effector can move in any arbitrary orientation
uint8 LINEAR = 1  # end effector moves in straight line

uint8 path_type
###############################################################################
# Constraints
# Leave it empty if you do not want to plan with constraints
# When planning with constraints arm will move slower
string item_id 
###############################################################################
# Requested end effector pose
geometry_msgs/Pose pose
###############################################################################
