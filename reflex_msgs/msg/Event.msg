# message for reporting finger events
string completion_msg		# Verbose, human-readable string
string completed_action		# Specify 'goto', 'guarded_move', etc. here
int32 finger_index			# 0, 1, 2
int32 completion_cause		# Use this for success/failure signalling, different for each maneuver. No set protocol yet
time timestamp