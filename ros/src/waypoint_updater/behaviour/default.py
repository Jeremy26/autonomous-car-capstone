import math

from behaviour import State


class DefaultBehaviour(object):

    def __init__(self, base_waypoints, look_ahead_waypoints):
        self.look_ahead_waypoints=look_ahead_waypoints
        self.base_waypoints=base_waypoints
        
        self.current_position=None
        self.current_behaviour = State.STANDBY

    def update(self, current_position, current_velocity):
        '''update the state of the behaviour.
            Params:
                - `current_position` : current position of the car
                - `current_velocity` : current velocity of the car
        '''        
        self.current_position = current_position.pose.position if current_position else None
        self.current_velocity = current_velocity

    def get_nearest_waypoint_index(self, car):
        '''Returns the nearest waypoint index from the current pose.'''
        min_distance = 10000000

        for i in range(len(self.base_waypoints)):
            wp = self.base_waypoints[i].pose.pose.position
            dist = math.sqrt((wp.x - car.x) ** 2 + (wp.y - car.y) ** 2 + (wp.z - car.z) ** 2)

            if dist < min_distance:
                min_distance = dist
                index_to_return = i
        return index_to_return

    def standing_by_(self):
        '''standing_by_ handles the STANDBY state transition.
        The following transition are available:
            - ACCELLERATING: if the current_position is initialized
        '''
        if self.current_position is not None:
            self.current_behaviour = State.ACCELERATING
        return

    def accelerating_(self):
        '''standing_by_ handles the ACCELERATING state transition.
        The following transition are available:
        '''
        nearest_waypoint_index = self.get_nearest_waypoint_index(self.current_position)

        waypoints = self.base_waypoints[nearest_waypoint_index:]
        wp_len = len(waypoints)
        if wp_len > self.look_ahead_waypoints:
            return waypoints[:self.look_ahead_waypoints]

        return waypoints + self.base_waypoints[:self.look_ahead_waypoints - wp_len]

    def braking_(self):
        '''standing_by_ handles the BRAKING state transition.
        The following transition are available:
        '''
        return None

    def process(self):
        '''process the trajectory depending on the current behaviour state.'''
        processor = self.standing_by_
        if self.current_behaviour == State.ACCELERATING:
            processor = self.accelerating_
        elif self.current_behaviour == State.BRAKING:
            processor = self.braking_
        
        return processor()
            

        
