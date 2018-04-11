from math import sqrt


STOP_DISTANCE_RATIO = 1.5
MAX_VELOCITY_PERCENT = 0.99
MAX_ACCELERATION = 5.0
MAX_DECELERATION = 5.0
CAR_LENGTH = 2


def distance(waypoints, wp1, wp2):
	'''
	returns the total cumulative distance between two waypoints
	Params:
		- `waypoints` : list of all waypoints
		- `wp1` : index of waypoint1
		- `wp2` : index of waypoint2
	'''
	dist = 0
	dl = lambda a, b: sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	for i in range(wp1, wp2+1):
		dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
		wp1 = i
	return dist


def accelerate(waypoints, current_velocity, target_velocity):
	'''
	returns a list of way points on which an acceleration is being made.
	Params:
		- `waypoints` : list of all waypoints
		- `current_velocity` : current velocity of the car
		- `target_velocity` : target velocity of the car
	'''
	if current_velocity >= target_velocity:
		for i in range(len(waypoints)):
			waypoints[i].twist.twist.linear.x = target_velocity
		return waypoints
	
	distance_to_cover = distance(waypoints, 0, len(waypoints)-1)
	acceleration = abs((target_velocity ** 2) - (current_velocity ** 2)) / (2 * distance_to_cover)

	acceleration = min(acceleration, MAX_ACCELERATION)
	for i in range(len(waypoints)):
		prev_waypoint_velocity = current_velocity
		if i > 0:
			prev_waypoint_velocity = waypoints[i - 1].twist.twist.linear.x

		velocity = target_velocity
		if not(i == len(waypoints) - 1):
			dist = distance(waypoints, i, i + 1)
			velocity = sqrt((2 * acceleration * dist) + (prev_waypoint_velocity**2))
		
		waypoints[i].twist.twist.linear.x = min(velocity, target_velocity)

	return waypoints

def decelerate(waypoints, car_index, target_index):
	'''
	returns a list of way points on which an deceleration is being made.
	Params:
		- `waypoints` : waypoints to be updates
		- `car_index` : position of the car
		- `target_index` : position of the target
	'''

	# find the index of traffic_light in waypoints set
	stop_index = max(0, target_index - (car_index + CAR_LENGTH))

	for i in range(len(waypoints)):
		velocity = 0.0
		if i <= stop_index:
			distance_to_stop_line = distance(waypoints, i, stop_index)
			velocity = sqrt(2 * MAX_DECELERATION * distance_to_stop_line)

			#reduce velocity even more to account for brake lag
			velocity = velocity * 0.8
			if velocity < 1.0:
				velocity = 0.0
			
		waypoints[i].twist.twist.linear.x = min(velocity, waypoints[i].twist.twist.linear.x)

	return waypoints

def stop(waypoints, wp1, wp2):
	'''
	sets waypoint velocities of waypoints in range (wp1,wp2) to zero
	Params:
		- `waypoints` : set of waypoints
		- `wp1` : index of waypoint1
		- `wp2` : index of waypoint2
	'''
	for i in range(wp1, wp2+1):
		waypoint = waypoints[i]
		waypoint.twist.twist.linear.x = 0.0

	return waypoints
