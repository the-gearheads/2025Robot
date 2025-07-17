import argparse
import math
import os
from jormungandr.optimization import Problem, ExitStatus
from jormungandr.autodiff import sin, cos, abs
import json
import constants

def get_end_eff_pos(ext_len, theta):
  x = ext_len * cos(theta)
  y = ext_len * sin(theta)
  return x, y

def get_elevator_len_arm_angle(x, y):
  return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

def lerp(a, b, t):
  return a + (b - a) * t

# begin giant ai generated function that looks sane to me
def resample_traj(trajectory_data, new_dt):
	"""
	Resample a trajectory with a constant, larger time step.
	
	Args:
		trajectory_data: Original trajectory data structure
		new_dt: New time step (larger than original to reduce sample count)
		
	Returns:
		Resampled trajectory data
	"""
	# Create a new trajectory data structure
	resampled_traj = {
		"name": trajectory_data["name"],
		"total_time": trajectory_data["total_time"],
		"waypoints": trajectory_data["waypoints"]
	}
	
	original_samples = trajectory_data["samples"]
	total_time = trajectory_data["total_time"]
	
	# Generate new time points with constant spacing
	time_points = []
	current_time = 0.0
	while current_time < total_time:
		time_points.append(current_time)
		current_time += new_dt
	# Ensure the last point is exactly at total_time
	if abs(time_points[-1] - total_time) > 1e-10:
		time_points.append(total_time)
	
	# Create new samples at each time point
	resampled_traj["samples"] = []
	for sample_num, current_time in enumerate(time_points):
		# Find the two neighboring samples in the original trajectory
		next_idx = next((i for i, sample in enumerate(original_samples) 
						if sample["time"] >= current_time), len(original_samples)-1)
		prev_idx = max(0, next_idx - 1)
		
		# Get the bracketing samples
		prev_sample = original_samples[prev_idx]
		next_sample = original_samples[next_idx]
		
		# Handle case where current_time exactly matches an original sample time
		if abs(prev_sample["time"] - current_time) < 1e-10:
			new_sample = prev_sample.copy()
			new_sample["sample_num"] = sample_num
			resampled_traj["samples"].append(new_sample)
		elif next_idx == prev_idx or abs(prev_sample["time"] - next_sample["time"]) < 1e-10:
			# Edge case: at the boundary or duplicate times
			new_sample = next_sample.copy()
			new_sample["sample_num"] = sample_num
			resampled_traj["samples"].append(new_sample)
		else:
			# Interpolate between prev_sample and next_sample
			t1 = prev_sample["time"]
			t2 = next_sample["time"]
			t = (current_time - t1) / (t2 - t1)  # Interpolation parameter
			dt = current_time - t1
			
			# Create new interpolated sample
			new_sample = {
				"time": current_time,
				"sample_num": sample_num
			}
			
			# Kinematic interpolation for pivot
			p1 = prev_sample["pivot_angle"]
			v1 = prev_sample["pivot_velocity"]
			a1 = prev_sample["pivot_accel"]
			new_sample["pivot_angle"] = p1 + v1*dt + 0.5*a1*dt*dt
			new_sample["pivot_velocity"] = v1 + a1*dt
			
			# Use lerp for acceleration
			a2 = next_sample["pivot_accel"]
			new_sample["pivot_accel"] = lerp(a1, a2, t)
			
			# Kinematic interpolation for elevator
			p1 = prev_sample["elevator_length"]
			v1 = prev_sample["elevator_velocity"]
			a1 = prev_sample["elevator_accel"]
			new_sample["elevator_length"] = p1 + v1*dt + 0.5*a1*dt*dt
			new_sample["elevator_velocity"] = v1 + a1*dt
			
			# Use lerp for acceleration
			a2 = next_sample["elevator_accel"]
			new_sample["elevator_accel"] = lerp(a1, a2, t)
			
			resampled_traj["samples"].append(new_sample)
	
	# Update the total number of samples
	resampled_traj["N_total"] = len(resampled_traj["samples"]) - 1
	
	return resampled_traj
# end giant ai generated function that looks sane to me

def round_traj(traj, ndigits):
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

  p = Problem()
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
    p.subject_to(pivot[0, int(sample)] == pose_pivot_elevator[1])
    p.subject_to(elevator[0, int(sample)] == pose_pivot_elevator[0])
    # populate an initial position guess from the last waypoint to this one
    # if i > 0:
    #   start_sample = int(waypoints[i - 1]["at_sample"] * N)
    #   this_sample = int(sample)
    #   start_elev_arm_angle = (waypoints[i - 1]["pose"][0], waypoints[i - 1]["pose"][1])
    #   this_elev_arm_angle = pose_pivot_elevator
    #   for k in range(start_sample + 1, this_sample):
    #     t = (k - start_sample) / (this_sample - start_sample)
    #     pivot[0, k].set_value(lerp(start_elev_arm_angle[1], this_elev_arm_angle[1], t))
    #     elevator[0, k].set_value(lerp(start_elev_arm_angle[0], this_elev_arm_angle[0], t))
    if vels is not None:
      p.subject_to(pivot[1, int(sample)] == vels[0])
      p.subject_to(elevator[1, int(sample)] == vels[1])


  for k in range(N + 1):
    if True:
      p.subject_to(pivot[0, k] > constants.pivot_min)
      p.subject_to(pivot[0, k] < constants.pivot_max)
    p.subject_to(elevator[0, k] >= constants.elevator_min_len)
    p.subject_to(elevator[0, k] <= constants.elevator_max_len)
    p.subject_to(elevator[1, k] <= constants.elevator_max_vel)
    p.subject_to(elevator[1, k] >= -constants.elevator_max_vel)
    p.subject_to(pivot[1, k] <= constants.pivot_max_vel)
    p.subject_to(pivot[1, k] >= -constants.pivot_max_vel)

    pos = get_end_eff_pos(elevator[0, k], pivot[0, k])
    p.subject_to(pos[0] >= constants.endeff_x_min)
    p.subject_to(pos[0] <= constants.endeff_x_max)
    p.subject_to(pos[1] >= constants.endeff_y_min)

    if k > 0 and k not in sample_transition_points:
      p.subject_to(dt[0, k] == dt[0, k-1])
    p.subject_to(dt[0, k] > 0)
    p.subject_to(dt[0, k] < constants.T_max / N)
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
    elevator_k[0].set_value((constants.elevator_min_len + constants.elevator_max_len) / 2) # avoid singularities at the initial 0 length state. being at the boundary also sucks
    if data.get("use_pivot_accel_scaling", True):
    # if False:
      pivot_accel_k = constants.pivot_accel_scaling(elevator_k[0])
    else:
      pivot_accel_k = constants.pivot_max_accel
    p.subject_to(accel_k[0] >= -pivot_accel_k)
    p.subject_to(accel_k[0] <= pivot_accel_k)
    p.subject_to(accel_k[1] >= -constants.elevator_max_accel)
    p.subject_to(accel_k[1] <= constants.elevator_max_accel)
  
  p.subject_to(accel[0, N] == 0)
  p.subject_to(accel[1, N] == 0)
  
  # Now we gotta minimize time
  total_time = sum(dt[0, k] for k in range(N + 1))
  p.subject_to(total_time <= constants.T_max) # despite the individual dt constraint this makes it better behaved??
  J_time = total_time
  J = J_time

  # # wrong for multiwaypoint
  # end_wp = waypoints[len(waypoints) - 1]
  # end_state = get_elevator_len_arm_angle(end_wp["pose"][0], end_wp["pose"][1])
  # displacement_pivot_elevator = p.decision_variable(2)
  use_new_ctrl_effort_penalization = True # so much better but lowkey makes the solver tip on the edge of instability
  ctrl_effort = (accel[0, N] / constants.pivot_max_accel)**2 + (accel[1, N] / constants.elevator_max_accel)**2
  for k in range(N):
    if not use_new_ctrl_effort_penalization:
      J += (pivot[0, k] - pivot[0, k-1])**2
      J += (elevator[0, k] - elevator[0, k-1])**2
    ctrl_effort += (accel[0, k] / constants.pivot_max_accel)**2 + (accel[1, k] / constants.elevator_max_accel)**2
    # J += (end_state[0] - elevator[0, k]) ** 2
    # J += (end_state[1] - pivot[0, k]) ** 2
    # displacement_pivot_elevator[0] += abs(elevator[0, k-1] - elevator[0, k])
    # displacement_pivot_elevator[1] += abs(pivot[0, k-1] - pivot[0, k])
  ctrl_effort /= ((N+1) * 4) # funky cost function weighing to try and get time and ctrl_effort to be of similar magnitudes. goal is for ctrl_effort to roughly be about, idk, 1/10th as big as J_time
  if use_new_ctrl_effort_penalization:
    J += ctrl_effort
  p.minimize(J)

  if debug:
    print(f"Solving trajectory {data["name"]}")
  exit_status = p.solve(diagnostics = debug, tolerance=1e-10)
  if True:
    print(f"ctrl effort: {ctrl_effort.value()}, J: {J.value()}, J_time = {J_time.value()} total_time: {total_time.value()}")
    # print(f"pivot vel: {pivot[1, N].value()} elev vel: {elevator[1, N].value()}")
  if exit_status not in [ExitStatus.SUCCESS]:
    print(f"Failed to solve trajectory {data["name"]}. Exit condition: {exit_status}")
    return -1
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
    })
    time += dt[0, k].value()
  # trajectory_data = resample_traj(trajectory_data, 0.015) # Broken (nonzero end vel)
  trajectory_data = round_traj(trajectory_data, 6)
  with open(f"{output_dir}/{os.path.splitext(os.path.basename(input_file))[0]}.agentraj", "w") as f:
    json.dump(trajectory_data, f, indent = 2)
  return 0



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate a trajectory for the armvator")
  parser.add_argument("input_file", type=str, help="The input file containing the waypoints")
  parser.add_argument("output_dir", type=str, help="The output directory to save the trajectory")
  args = parser.parse_args()
  main(args.input_file, args.output_dir)
