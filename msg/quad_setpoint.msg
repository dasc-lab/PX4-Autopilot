# quadrotor setpoint in ENU frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32[3] pos
float32[3] vel	
float32[3] acc	
float32[3] b1d # desired heading vector (body x axis direction)
float32[3] ang_vel
float32[3] ang_acc
