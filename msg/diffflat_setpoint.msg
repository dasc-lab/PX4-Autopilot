# differential flatness setpoint in ENU frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32[3] pos	
float32[3] vel	
float32[3] acc	
float32[3] jerk	
float32[3] snap	
float32 yaw		
float32 yaw_rate	
float32 yaw_acc	


