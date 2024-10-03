import numpy as np

from ace.vehicles import VehicleData
from ace.steering_utils import (
    distance_from_point_to_line_segment,
    calculate_steering_angle,
)


class SteeringGeometry:
    def __init__(self, data_path: str):
        self.vehicle_data = VehicleData(data_path)

    def max_steering_angle(self) -> float:
        return self.steering_angle(1.0)

    def min_steering_angle(self) -> float:
        return self.steering_angle(-1.0)

    def steering_angle(self, normalised_controller_input: float) -> float:
        steering_wheel_input = (
            normalised_controller_input * self.vehicle_data.steering_lock
        )
        steering_angle_left_tyre = (
            self.left_neutral_tyre_angle()
            - self.left_tyre_angle(steering_wheel_input)
            + self.left_rack_motion_difference(steering_wheel_input)
        )
        steering_angle_right_tyre = -(
            self.right_neutral_tyre_angle()
            - self.right_tyre_angle(steering_wheel_input)
            + self.right_rack_motion_difference(steering_wheel_input)
        )
        return (steering_angle_left_tyre + steering_angle_right_tyre) / 2

    def left_neutral_tyre_angle(self) -> float:
        return self.tyre_angle(self.left_inner_tie_rod_to_steer_axis_neutral_distance())

    def left_tyre_angle(self, steering_wheel_input: float) -> float:
        return self.tyre_angle(
            self.distance_from_left_inner_tie_rod_to_steer_axis(steering_wheel_input)
        )

    def right_neutral_tyre_angle(self) -> float:
        return self.tyre_angle(
            self.right_inner_tie_rod_to_steer_axis_neutral_distance()
        )

    def right_tyre_angle(self, steering_wheel_input: float) -> float:
        return self.tyre_angle(
            self.distance_from_right_inner_tie_rod_to_steer_axis(steering_wheel_input)
        )

    def tyre_angle(self, distance_form_tie_rod_to_steering_axis: np.array) -> float:
        tyre_angle = calculate_steering_angle(
            self.steering_link_length(),
            distance_form_tie_rod_to_steering_axis,
            self.tie_rod_length(),
        )
        return tyre_angle

    def right_rack_motion_difference(self, steering_wheel_input: float) -> float:
        right_diff = calculate_steering_angle(
            self.right_inner_tie_rod_to_steer_axis_neutral_distance(),
            self.distance_from_right_inner_tie_rod_to_steer_axis(steering_wheel_input),
            self.tie_rod_movement(steering_wheel_input),
        )
        return -right_diff if steering_wheel_input > 0 else right_diff

    def right_inner_tie_rod_to_steer_axis_neutral_distance(self) -> np.array:
        return self.distance_from_right_tyre_axis(
            self.vehicle_data.right_inner_tie_rod_location
        )

    def distance_from_right_tyre_axis(self, point: np.array) -> np.array:
        distance_from_tyre_axis = distance_from_point_to_line_segment(
            point,
            self.vehicle_data.right_tyre_axis_top_location,
            self.vehicle_data.right_tyre_axis_bottom_location,
        )
        return distance_from_tyre_axis

    def distance_from_right_inner_tie_rod_to_steer_axis(
        self, steering_wheel_input: float
    ) -> np.array:
        tie_rod_movement = self.tie_rod_displacement(steering_wheel_input)
        right_inner_tie_rod_location = (
            self.vehicle_data.right_inner_tie_rod_location - tie_rod_movement
        )
        return self.distance_from_right_tyre_axis(right_inner_tie_rod_location)

    def left_rack_motion_difference(self, steering_wheel_input: float) -> np.array:
        left_diff = calculate_steering_angle(
            self.left_inner_tie_rod_to_steer_axis_neutral_distance(),
            self.distance_from_left_inner_tie_rod_to_steer_axis(steering_wheel_input),
            self.tie_rod_movement(steering_wheel_input),
        )
        return -left_diff if steering_wheel_input < 0 else left_diff

    def left_inner_tie_rod_to_steer_axis_neutral_distance(self) -> np.array:
        return self.distance_from_left_tyre_axis(
            self.vehicle_data.left_inner_tie_rod_location
        )

    def distance_from_left_tyre_axis(self, point: np.array) -> np.array:
        distance_from_tyre_axis = distance_from_point_to_line_segment(
            point,
            self.vehicle_data.left_tyre_axis_top_location,
            self.vehicle_data.left_tyre_axis_bottom_location,
        )
        return distance_from_tyre_axis

    def distance_from_left_inner_tie_rod_to_steer_axis(
        self,
        steering_wheel_input: float,
    ) -> np.array:
        tie_rod_movement = self.tie_rod_displacement(steering_wheel_input)
        left_inner_tie_rod_location = (
            self.vehicle_data.left_inner_tie_rod_location - tie_rod_movement
        )
        return self.distance_from_left_tyre_axis(left_inner_tie_rod_location)

    def tie_rod_length(self) -> np.array:
        inner_tie_rod = self.vehicle_data.left_inner_tie_rod_location
        outer_tie_rod = self.vehicle_data.left_outer_tie_rod_location
        return np.linalg.norm(inner_tie_rod - outer_tie_rod)

    def steering_link_length(self) -> np.array:
        return self.distance_from_left_tyre_axis(
            self.vehicle_data.left_outer_tie_rod_location
        )

    def tie_rod_displacement(self, steering_wheel_input: float) -> np.array:
        return np.array([self.tie_rod_movement(steering_wheel_input), 0, 0])

    def tie_rod_movement(self, steering_wheel_input: float) -> float:
        rack_rotation = steering_wheel_input / self.vehicle_data.steering_ratio
        tie_rod_movement = (
            rack_rotation * self.vehicle_data.rack_movement_to_steering_angle_ratio
        )
        return tie_rod_movement


if __name__ == "__main__":
    car_geom = SteeringGeometry("./data/audi_r8_lms_2016")
    print("------ Neutral tyre angle ------")
    print(car_geom.left_neutral_tyre_angle() * 180 / np.pi)
    print(car_geom.right_neutral_tyre_angle() * 180 / np.pi)
    print("------ Min tyre angle ------")
    print(f"Average tyre angle: {car_geom.min_steering_angle() * 180 / np.pi}")
    print("------ Max tyre angle ------")
    print(f"Average tyre angle: {car_geom.max_steering_angle() * 180 / np.pi}")
    print(car_geom.steering_angle(0.5) * 180 / np.pi)
    print(car_geom.steering_angle(-0.5) * 180 / np.pi)
