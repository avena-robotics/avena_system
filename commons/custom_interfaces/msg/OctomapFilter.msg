std_msgs/Header header
sensor_msgs/Image cam1_rgb
sensor_msgs/Image cam2_rgb
sensor_msgs/Image cam1_depth
sensor_msgs/Image cam2_depth
sensor_msgs/PointCloud2 merged_ptcld_filtered
string[] cam1_masks
int32[] cam1_labels
string[] cam2_masks
int32[] cam2_labels
Item[] items
string occupancy_grid
int32[] selected_items_ids
sensor_msgs/PointCloud2 filtered_scene_octomap