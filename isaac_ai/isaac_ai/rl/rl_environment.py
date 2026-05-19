# Define the reinforcement-learning environment wrapper for rover driving.
#
# TODO(real RL training):
# - Wrap Isaac Sim observations/actions into the RL library environment API.
# - Reset should call simulation reset logic; step should apply rover control,
#   advance simulation, collect observations, compute reward, and report done.
# - Keep trained policy outputs compatible with SelectedDriveAction.
