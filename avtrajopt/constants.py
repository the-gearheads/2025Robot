import math

N_samples_default = 100
T_max = 10 # seconds
dt_initial_guess = 0.005 # 50ms. bit big but works
IN_TO_M = 0.0254
elevator_min_len = 36 * IN_TO_M ## + 1 * IN_TO_M
elevator_max_len = 80 * IN_TO_M

pivot_min = math.radians(13.5)
pivot_max = math.radians(90.1)

endeff_x_min = -1e-6
endeff_x_max = 40 * IN_TO_M
endeff_y_min = 0

# elevator_max_accel = 1.3 # m/s^2
# pivot_max_accel = 3 # rad/s^2
elevator_max_accel = 3.0 # m/s^2
pivot_max_accel = 3 # rad/s^2

elevator_max_vel = 0.9
pivot_max_vel = 1.3

def pivot_accel_scaling(elevator_length):
  from jormungandr.autodiff import pow
  if type(elevator_length) == float:
    return pivot_max_accel * math.pow(elevator_min_len / elevator_length, 2)
  return pivot_max_accel * pow(elevator_min_len / elevator_length, 2) # honestly no idea if this is correct but deepseek suggested it? really just a heuristic so it doesnt matter much until we do real testing

  
