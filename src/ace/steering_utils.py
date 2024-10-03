import numpy as np


def distance_from_point_to_line_segment(
    point: np.array,
    line_a: np.array,
    line_b: np.array,
) -> np.array:
    # normalized tangent vector
    d = np.divide(line_b - line_a, np.linalg.norm(line_b - line_a))
    # signed parallel distance components
    s = np.dot(line_a - point, d)
    t = np.dot(point - line_b, d)
    # clamped parallel distance
    h = np.maximum.reduce([s, t, 0])
    # perpendicular distance component
    c = np.cross(point - line_a, d)
    return np.hypot(h, np.linalg.norm(c))


def calculate_steering_angle(
    distance_steer_knuckle_to_steer_axis: np.array,
    distance_rack_to_steer_axis: np.array,
    tie_rod_length: np.array,
) -> np.array:
    numerator = (
        distance_steer_knuckle_to_steer_axis**2
        + distance_rack_to_steer_axis**2
        - tie_rod_length**2
    )
    denominator = 2 * distance_steer_knuckle_to_steer_axis * distance_rack_to_steer_axis
    # Floating point precision becomes an issues when numerator and denominator are close
    numerator = np.round(numerator, decimals=14)
    denominator = np.round(denominator, decimals=14)
    angle = np.arccos(numerator / denominator)
    return angle
