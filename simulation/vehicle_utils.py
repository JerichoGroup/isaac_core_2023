import math

def get_wheel_degrees_from_radius(radius: float, wheelbase: float = 2.1115 ) -> float: 
    # weelbase is in Meters
    radians_steering_angle = math.atan(wheelbase / radius)
    return math.degrees(radians_steering_angle)


def get_angular_velocity_from_linear(vehicle_speed_kmh: float, wheel_radius: float = 0.3) -> float:
    # wheel_radius is in Meters
    linear_velocity_ms = vehicle_speed_kmh / 3.6
    angular_velocity_radians = linear_velocity_ms / wheel_radius
    angular_velocity_degrees = (180 / math.pi) * angular_velocity_radians
    return angular_velocity_degrees
