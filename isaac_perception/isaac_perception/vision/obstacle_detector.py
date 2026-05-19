# Detect obstacles in the rover driving area.
#
# TODO(real perception):
# - Use depth image, LiDAR point cloud, or occupancy data to detect obstacles.
# - Output should map into PerceptionResult.obstacle_detected and
#   PerceptionResult.obstacle_distance.
# - Navigation manager can later use this result for avoidance/replanning.
