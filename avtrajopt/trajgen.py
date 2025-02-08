import argparse
import math
import os
from jormungandr.optimization import OptimizationProblem
from jormungandr.autodiff import sin, cos
import json
import constants

def get_end_eff_pos(ext_len, theta):
  x = ext_len * cos(theta)
  y = ext_len * sin(theta)
  return x, y

def get_elevator_len_arm_angle(x, y):
  return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

def main(input_file, output_dir):
  with open(input_file, 'r') as f:
    data = json.load(f)
  
  N = data.get("N", constants.N_samples_default)

  p = OptimizationProblem()
  pivot = p.decision_variable(2, N + 1) # angle (rad), angular velocity (rad/s)
  elevator = p.decision_variable(2, N + 1) # length (m), velocity (m/s)
  accel = p.decision_variable(2, N + 1) # pivot accel (rad/s^2), elevator accel (m/s^2)
  dt = p.decision_variable(1, N + 1) # time step, constrained to be equal

  # Gotta make it go where we want
  waypoints = data["waypoints"]
  assert len(waypoints) >= 2

  if "vel" not in waypoints[0]:
    raise ValueError("First waypoint must have a velocity")
  
  if "vel" not in waypoints[-1]:
    raise ValueError("Last waypoint must have a velocity")


  sample_transition_points = [] # Points where midwaypoints begin
  for i, waypoint in enumerate(waypoints):
    sample = waypoint["at_sample"] * N
    if i != 0 and i < len(waypoints) - 1:
      sample_transition_points.append(sample)
    endeff_pos = waypoint["pose"]
    vels = waypoint.get("vel", None)
    pose_pivot_elevator = get_elevator_len_arm_angle(endeff_pos[0], endeff_pos[1])
    if pose_pivot_elevator[0] < constants.elevator_min_len:
      raise ValueError(f"Elevator length is too short to reach {waypoint} ({pose_pivot_elevator[0]} < {constants.elevator_min_len})")
    if pose_pivot_elevator[0] > constants.elevator_max_len:
      raise ValueError(f"Elevator length is too long to reach {waypoint} ({pose_pivot_elevator[0]} > {constants.elevator_max_len})")
    if pose_pivot_elevator[1] < constants.pivot_min:
      raise ValueError(f"Pivot angle is too small to reach {waypoint} ({pose_pivot_elevator[1]} < {constants.pivot_min})")
    if pose_pivot_elevator[1] > constants.pivot_max:
      raise ValueError(f"Pivot angle is too large to reach {waypoint} ({pose_pivot_elevator[1]} > {constants.pivot_max})")
    p.subject_to(pivot[0, int(sample)] == pose_pivot_elevator[1])
    p.subject_to(elevator[0, int(sample)] == pose_pivot_elevator[0])
    if vels is not None:
      p.subject_to(pivot[1, int(sample)] == vels[0])
      p.subject_to(elevator[1, int(sample)] == vels[1])


  for k in range(N + 1):
    p.subject_to(pivot[0, k] >= constants.pivot_min)
    p.subject_to(pivot[0, k] <= constants.pivot_max)
    p.subject_to(elevator[0, k] >= constants.elevator_min_len)
    p.subject_to(elevator[0, k] <= constants.elevator_max_len)

    pos = get_end_eff_pos(elevator[0, k], pivot[0, k])
    p.subject_to(pos[0] >= constants.endeff_x_min)
    p.subject_to(pos[0] <= constants.endeff_x_max)
    p.subject_to(pos[1] >= constants.endeff_y_min)

    if k > 0 and k not in sample_transition_points:
      p.subject_to(dt[0, k] == dt[0, k-1])
    p.subject_to(dt[0, k] > 0)
    dt[0, k].set_value(constants.dt_initial_guess)

  for k in range(N):
    pivot_k = pivot[:, k]
    pivot_k1 = pivot[:, k + 1]

    elevator_k = elevator[:, k]
    elevator_k1 = elevator[:, k + 1]

    accel_k = accel[:, k]

    # Dynamics
    p.subject_to(pivot_k1[0] == pivot_k[0] + dt[0, k] * pivot_k[1] + 0.5 * dt[0, k] ** 2 * accel_k[0])
    p.subject_to(pivot_k1[1] == pivot_k[1] + dt[0, k] * accel_k[0])

    p.subject_to(elevator_k1[0] == elevator_k[0] + dt[0, k] * elevator_k[1] + 0.5 * dt[0, k] ** 2 * accel_k[1])
    p.subject_to(elevator_k1[1] == elevator_k[1] + dt[0, k] * accel_k[1])

    # Acceleration limits
    elevator_k[0].set_value(constants.elevator_min_len) # avoid singularities at the initial 0 length state
    pivot_accel_k = constants.pivot_accel_scaling(elevator_k[0])
    p.subject_to(accel_k[0] >= -pivot_accel_k)
    p.subject_to(accel_k[0] <= pivot_accel_k)
    p.subject_to(accel_k[1] >= -constants.elevator_max_accel)
    p.subject_to(accel_k[1] <= constants.elevator_max_accel)
  
  # Now we gotta minimize time
  total_time = sum(dt[0, k] for k in range(N + 1))
  p.subject_to(total_time <= constants.T_max)
  p.minimize(total_time)
  
  print(f"Solving trajectory {data["name"]}")
  p.solve(diagnostics = True)
  # yay
  # now we gotta save it
  trajectory_data = {
    "name": data["name"],
    "total_time": total_time.value(),
    "N_total": N,
    "waypoints": waypoints
  }

  trajectory_data["samples"] = []
  time = 0
  for k in range(N + 1):
    endeff_pos = get_end_eff_pos(elevator[0, k], pivot[0, k])
    trajectory_data["samples"].append({
      "time": time,
      "sample_num": k,
      "pivot_angle": pivot[0, k].value(),
      "elevator_length": elevator[0, k].value(),
      "pivot_velocity": pivot[1, k].value(),
      "elevator_velocity": elevator[1, k].value(),
      "pivot_accel": accel[0, k].value(),
      "elevator_accel": accel[1, k].value(),
      "endeff_pos": [endeff_pos[0].value(), endeff_pos[1].value()]
    })
    time += dt[0, k].value()
  with open(f"{output_dir}/{os.path.splitext(os.path.basename(input_file))[0]}.agentraj", "w") as f:
    json.dump(trajectory_data, f, indent = 2)



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate a trajectory for the elevator")
  parser.add_argument("input_file", type=str, help="The input file containing the waypoints")
  parser.add_argument("output_dir", type=str, help="The output directory to save the trajectory")
  args = parser.parse_args()
  main(args.input_file, args.output_dir)