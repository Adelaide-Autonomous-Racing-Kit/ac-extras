import copy
import math
import pprint
from typing import Dict

import numpy as np

from ace.utils import load_vehicle_data


class VehicleData:
    def __init__(self, data_path: str):
        self.__setup(data_path)

    @property
    def max_steering_radians(self) -> float:
        controls = self["CONTROLS"]
        max_steering_degrees = controls["STEER_LOCK"] / controls["STEER_RATIO"]
        return max_steering_degrees / 180 * math.pi

    @property
    def front_suspension(self) -> Dict:
        return self._data["FRONT"]

    @property
    def rear_suspension(self) -> Dict:
        return self._data["REAR"]

    @property
    def car_control(self) -> Dict:
        return self._data["CONTROLS"]
    
    @property
    def wheelbase(self) -> float:
        return self._data["BASIC"]["WHEELBASE"]
    
    @property
    def width(self) -> float:
        return self._data["COLLIDER_0"]["SIZE"][0]
    
    @property
    def steering_ratio(self) -> float:
        return self.car_control["STEER_RATIO"]
    
    @property
    def steering_lock(self) -> float:
        return self.car_control["STEER_LOCK"]
    
    @property
    def rack_movement_to_steering_angle_ratio(self) -> float:
        return self.car_control["LINEAR_STEER_ROD_RATIO"]
    
    @property
    def right_inner_tie_rod_location(self) -> np.array:
        return self.front_suspension['WBCAR_STEER']
    
    @property
    def left_inner_tie_rod_location(self) -> np.array:
        return self._flip_x_coordinate(self.right_inner_tie_rod_location)
    
    @property
    def right_outer_tie_rod_location(self) -> np.array:
        return self.front_suspension['WBTYRE_STEER']
    
    @property
    def left_outer_tie_rod_location(self) -> np.array:
        return self._flip_x_coordinate(self.right_outer_tie_rod_location)
    
    @property
    def right_tyre_axis_top_location(self) -> np.array:
        return self.front_suspension['WBTYRE_TOP']
    
    @property
    def left_tyre_axis_top_location(self) -> np.array:
        return self._flip_x_coordinate(self.right_tyre_axis_top_location)
    
    @property
    def right_tyre_axis_bottom_location(self) -> np.array:
        return self.front_suspension['WBTYRE_BOTTOM']
    
    @property
    def left_tyre_axis_bottom_location(self) -> np.array:
        return self._flip_x_coordinate(self.right_tyre_axis_bottom_location)
    
    def _flip_x_coordinate(self, to_flip: np.array) -> np.array:
        to_flip = copy.deepcopy(to_flip)
        to_flip[0] *= -1
        return to_flip


    def __getitem__(self, key: str) -> str:
        return self._data[key]

    def __repr__(self) -> str:
        return pprint.pformat(self._data, indent=1)

    def __setup(self, data_path: str):
        self._data = load_vehicle_data(data_path)


if __name__ == "__main__":
    x = VehicleData("./data/audi_r8_lms_2016")
    print(x)
