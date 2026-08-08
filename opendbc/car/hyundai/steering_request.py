from opendbc.car.lateral import common_fault_avoidance


# EPS faults if torque is requested above 90 degrees for more than 1 second.
# Keep every threshold slightly below the EPS limit.
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def steering_request_fault_avoidance_counter_valid(counter: object) -> bool:
  return type(counter) is int and 0 <= counter < MAX_ANGLE_FRAMES + MAX_ANGLE_CONSECUTIVE_FRAMES


def apply_steering_request_fault_avoidance(
  steering_angle_deg: float,
  lateral_active: bool,
  angle_limit_counter: int,
) -> tuple[int, bool]:
  return common_fault_avoidance(
    abs(steering_angle_deg) >= MAX_ANGLE,
    lateral_active,
    angle_limit_counter,
    MAX_ANGLE_FRAMES,
    MAX_ANGLE_CONSECUTIVE_FRAMES,
  )
