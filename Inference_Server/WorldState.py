

FRONT_DISTANCE_THRESHOLD = 10
SIDE_DISTANCE_THRESHOLD = 10


class WorldState:
    def __init__(self, num_lanes):
        self.current_lane = 0
        self.num_lanes = num_lanes

        self.num_lanes_right = 0
        self.num_lanes_left = 0
        self.steering_angle = 0
        self.num_points = 0

        self.distance_to_nearest_car_in_my_lane = 0
        self.distance_to_nearest_car_in_left_lane = 0
        self.distance_to_nearest_car_in_right_lane = 0

        self.is_changing_lane = False

    def can_change_lane(self):
        if self.distance_to_nearest_car_in_my_lane < FRONT_DISTANCE_THRESHOLD:
            if self.distance_to_nearest_car_in_right_lane > SIDE_DISTANCE_THRESHOLD:
                return True, 'r'
            elif self.distance_to_nearest_car_in_left_lane > SIDE_DISTANCE_THRESHOLD:
                return True, 'l'
            else:
                return False, 'n'
        else:
            return False, 'n'
