import argparse
import math
import os
import json
import casadi as ca
import constants

def get_end_eff_pos(ext_len, theta):
  # Use CasADi's symbolic sin and cos
  x = ext_len * ca.cos(theta)
  y = ext_len * ca.sin(theta)
  return x, y

def get_elevator_len_arm_angle(x, y):
  # This function is used for numeric inputs from the config file, so math is fine.
  return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

def lerp(a, b, t):
  return a + (b - a) * t

def round_traj(traj, ndigits):
  # This function remains unchanged, as it operates on numeric results.
  new_traj = traj.copy()
  new_traj["total_time"] = round(new_traj["total_time"], ndigits)
  for sample in new_traj["samples"]:
    sample["time"] = round(sample["time"], ndigits)
    sample["pivot_angle"] = round(sample["pivot_angle"], ndigits)
    sample["elevator_length"] = round(sample["elevator_length"], ndigits)
    sample["pivot_velocity"] = round(sample["pivot_velocity"], ndigits)
    sample["elevator_velocity"] = round(sample["elevator_velocity"], ndigits)
    sample["pivot_accel"] = round(sample["pivot_accel"], ndigits)
    sample["elevator_accel"] = round(sample["elevator_accel"], ndigits)
  for waypoint in new_traj["waypoints"]:
    waypoint["pose"][0] = round(waypoint["pose"][0], ndigits)
    waypoint["pose"][1] = round(waypoint["pose"][1], ndigits)
    if "vel" in waypoint:
      waypoint["vel"][0] = round(waypoint["vel"][0], ndigits)
      waypoint["vel"][1] = round(waypoint["vel"][1], ndigits)
  return new_traj

def main(input_file, output_dir, debug = True):
  with open(input_file, 'r') as f:
    data = json.load(f)

  N = data.get("N", constants.N_samples_default)

  # === CASADI PROBLEM SETUP ===
  opti = ca.Opti()

  pivot = opti.variable(2, N + 1) # [0,:]: angle (rad), [1,:]: angular velocity (rad/s)
  elevator = opti.variable(2, N + 1) # [0,:]: length (m), [1,:]: velocity (m/s)
  accel = opti.variable(2, N + 1) # [0,:]: pivot accel (rad/s^2), [1,:]: elevator accel (m/s^2)
  dt = opti.variable(1, N + 1) # time step, constrained to be equal across segments

  # === WAYPOINT CONSTRAINTS ===
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

    # Pre-computation checks remain the same
    if endeff_pos[0] < constants.endeff_x_min:
      raise ValueError(f"End effector x position is too small to reach {waypoint} ({endeff_pos[0]} < {constants.endeff_x_min})")
    if endeff_pos[0] > constants.endeff_x_max:
      raise ValueError(f"End effector x position is too large to reach {waypoint} ({endeff_pos[0]} > {constants.endeff_x_max})")
    if endeff_pos[1] < constants.endeff_y_min:
      raise ValueError(f"End effector y position is too small to reach {waypoint} ({endeff_pos[1]} < {constants.endeff_y_min})")
    if pose_pivot_elevator[0] < constants.elevator_min_len:
      raise ValueError(f"Elevator length is too short to reach {waypoint} ({pose_pivot_elevator[0]} < {constants.elevator_min_len})")
    if pose_pivot_elevator[0] > constants.elevator_max_len:
      raise ValueError(f"Elevator length is too long to reach {waypoint} ({pose_pivot_elevator[0]} > {constants.elevator_max_len})")
    if pose_pivot_elevator[1] < constants.pivot_min:
      raise ValueError(f"Pivot angle is too small to reach {waypoint} ({pose_pivot_elevator[1]} < {constants.pivot_min})")
    if pose_pivot_elevator[1] > constants.pivot_max:
      raise ValueError(f"Pivot angle is too large to reach {waypoint} ({pose_pivot_elevator[1]} > {constants.pivot_max})")

    # Set waypoint constraints
    opti.subject_to(pivot[0, int(sample)] == pose_pivot_elevator[1])
    opti.subject_to(elevator[0, int(sample)] == pose_pivot_elevator[0])

    if vels is not None:
      opti.subject_to(pivot[1, int(sample)] == vels[0])
      opti.subject_to(elevator[1, int(sample)] == vels[1])

  # === STATE AND KINEMATIC CONSTRAINTS ===
  for k in range(N + 1):
    opti.subject_to(opti.bounded(constants.pivot_min, pivot[0, k], constants.pivot_max))
    opti.subject_to(opti.bounded(constants.elevator_min_len, elevator[0, k], constants.elevator_max_len))
    opti.subject_to(opti.bounded(-constants.elevator_max_vel, elevator[1, k], constants.elevator_max_vel))
    opti.subject_to(opti.bounded(-constants.pivot_max_vel, pivot[1, k], constants.pivot_max_vel))

    # End effector position constraints using the symbolic helper function
    pos = get_end_eff_pos(elevator[0, k], pivot[0, k])
    opti.subject_to(pos[0] >= constants.endeff_x_min)
    opti.subject_to(pos[0] <= constants.endeff_x_max)
    opti.subject_to(pos[1] >= constants.endeff_y_min)

    # Time step constraints
    if k > 0 and k not in sample_transition_points:
      opti.subject_to(dt[0, k] == dt[0, k-1])
    opti.subject_to(dt[0, k] > 0)
    opti.subject_to(dt[0, k] < constants.T_max / N)
    opti.set_initial(dt[0, k], constants.dt_initial_guess)

  # === DYNAMICS CONSTRAINTS ===
  for k in range(N):
    pivot_k = pivot[:, k]
    pivot_k1 = pivot[:, k + 1]

    elevator_k = elevator[:, k]
    elevator_k1 = elevator[:, k + 1]

    accel_k = accel[:, k]

    # Dynamics (implicit trapezoid integration)
    opti.subject_to(pivot_k1[0] == pivot_k[0] + dt[0, k] * pivot_k[1] + 0.5 * dt[0, k] ** 2 * accel_k[0])
    opti.subject_to(pivot_k1[1] == pivot_k[1] + dt[0, k] * accel_k[0])

    opti.subject_to(elevator_k1[0] == elevator_k[0] + dt[0, k] * elevator_k[1] + 0.5 * dt[0, k] ** 2 * accel_k[1])
    opti.subject_to(elevator_k1[1] == elevator_k[1] + dt[0, k] * accel_k[1])

    # Acceleration limits
    opti.set_initial(elevator_k[0], (constants.elevator_min_len + constants.elevator_max_len) / 2) # Initial guess

    if data.get("use_pivot_accel_scaling", True):
      pivot_accel_k_limit = constants.pivot_accel_scaling(elevator_k[0])
    else:
      pivot_accel_k_limit = constants.pivot_max_accel

    opti.subject_to(opti.bounded(-pivot_accel_k_limit, accel_k[0], pivot_accel_k_limit))
    opti.subject_to(opti.bounded(-constants.elevator_max_accel, accel_k[1], constants.elevator_max_accel))

  # Final acceleration must be zero
  opti.subject_to(accel[0, N] == 0)
  opti.subject_to(accel[1, N] == 0)

  # === OBJECTIVE FUNCTION ===
  # Use ca.sum2 to sum over the columns of the 1x(N+1) dt matrix
  total_time = ca.sum2(dt)
  opti.subject_to(total_time <= constants.T_max)
  J_time = total_time
  J = J_time

  # Penalize control effort (sum of squared accelerations)
  # ca.sumsqr correctly returns a scalar, so this part is fine.
  ctrl_effort = ca.sumsqr(accel[0, :] / constants.pivot_max_accel) + \
                ca.sumsqr(accel[1, :] / constants.elevator_max_accel)

  # Funky cost function weighing from original code
  ctrl_effort /= ((N+1) * 4)

  J += ctrl_effort
  opti.minimize(J)

  # === SOLVE THE PROBLEM ===
  if debug:
    print(f"Solving trajectory {data['name']}")

  # Configure solver options
  solver_opts = {
      'ipopt.print_level': 5 if debug else 0,
      'ipopt.tol': 1e-10,
      'print_time': debug
  }
  opti.solver('ipopt', solver_opts)

  try:
    sol = opti.solve()
    success = True
  except RuntimeError as e:
    print(f"Solver failed for trajectory {data['name']} with error: {e}")
    success = False
    return -1

  if success:
    # Use sol.value() to get the numerical result of a symbolic variable
    final_total_time = sol.value(total_time)
    print(f"ctrl effort: {sol.value(ctrl_effort)}, J: {sol.value(J)}, J_time = {sol.value(J_time)} total_time: {final_total_time}")

    # === SAVE RESULTS ===
    trajectory_data = {
      "name": data["name"],
      "total_time": final_total_time,
      "N_total": N,
      "waypoints": waypoints
    }

    trajectory_data["samples"] = []
    time = 0
    # Retrieve all values at once for efficiency
    pivot_val = sol.value(pivot)
    elevator_val = sol.value(elevator)
    accel_val = sol.value(accel)
    dt_val = sol.value(dt)

    for k in range(N + 1):
      trajectory_data["samples"].append({
        "time": time,
        "sample_num": k,
        "pivot_angle": pivot_val[0, k],
        "elevator_length": elevator_val[0, k],
        "pivot_velocity": pivot_val[1, k],
        "elevator_velocity": elevator_val[1, k],
        "pivot_accel": accel_val[0, k],
        "elevator_accel": accel_val[1, k],
      })
      # --- THIS IS THE ONLY LINE THAT NEEDS TO CHANGE ---
      time += dt_val[k]

    trajectory_data = round_traj(trajectory_data, 6)
    output_path = f"{output_dir}/{os.path.splitext(os.path.basename(input_file))[0]}.agentraj"
    with open(output_path, "w") as f:
      json.dump(trajectory_data, f, indent = 2)
    print(f"Successfully generated trajectory and saved to {output_path}")
    return 0
  else:
    print(f"Failed to solve trajectory {data['name']}. Status: {opti.return_status()}")
    return -1


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate a trajectory for the armvator using CasADi")
  parser.add_argument("input_file", type=str, help="The input file containing the waypoints")
  parser.add_argument("output_dir", type=str, help="The output directory to save the trajectory")
  args = parser.parse_args()
  main(args.input_file, args.output_dir)
