# message for ReFlex Fingers
float32 proximal		# radians, measured from all open = 0, to pi = closed
float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link
bool[9] contact			# binary, 0 = proximal, 8 = fingertip
float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)
