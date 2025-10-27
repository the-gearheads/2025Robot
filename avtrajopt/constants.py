import math
import casadi as ca

N_samples_default = 150
T_max = 10 # seconds
dt_initial_guess = 0.005 # s
IN_TO_M = 0.0254
elevator_min_len = 36 * IN_TO_M ## + 1 * IN_TO_M
elevator_max_len = 80 * IN_TO_M

pivot_min = math.radians(13.5)
pivot_max = math.radians(90.1)

endeff_x_min = -1e-6
endeff_x_max = 40 * IN_TO_M
endeff_y_min = 0

elevator_max_accel = 3 # m/s^2
pivot_max_accel = 3 # rad/s^2

elevator_max_vel = 0.9
pivot_max_vel = 1.3

def pivot_accel_scaling(elevator_length):
  """
  Calculates the maximum pivot acceleration, scaled by the elevator length.
  This function works with both standard floats and CasADi symbolic variables.
  """
  # CasADi's operators are overloaded, so `**` works for both numeric and symbolic types.
  return pivot_max_accel * (elevator_min_len / elevator_length)**2
