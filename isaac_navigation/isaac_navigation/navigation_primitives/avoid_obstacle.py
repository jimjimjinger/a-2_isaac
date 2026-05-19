# Avoid obstacles during rover navigation.
#
# TODO(real navigation):
# - Implement obstacle avoidance used by navigation_manager_node.
# - Input candidates: current rover pose, target pose, obstacle distance/map,
#   terrain traversability, and selected RL action.
# - Output should be a safe drive target or low-level command for the executor.
