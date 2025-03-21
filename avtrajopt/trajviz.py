import matplotlib.pyplot as plt
import argparse
import math
import constants
import json

def endeff_pos(ext_len, theta):
  x = ext_len * math.cos(theta)
  y = ext_len * math.sin(theta)
  return x, y


def plot_traj(traj_path):
  with open(traj_path, 'r') as f:
    data = json.load(f)
  
  samples = data["samples"]
  waypoint_poses = [waypoint["pose"] for waypoint in data["waypoints"]]

  pivot_angles = [sample["pivot_angle"] for sample in samples]
  pivot_velocities = [sample["pivot_velocity"] for sample in samples]
  elevator_lengths = [sample["elevator_length"] for sample in samples]
  elevator_velocities = [sample["elevator_velocity"] for sample in samples]
  pivot_accelerations = [sample["pivot_accel"] for sample in samples]
  elevator_accelerations = [sample["elevator_accel"] for sample in samples]
  endeff_poses = [endeff_pos(sample["elevator_length"], sample["pivot_angle"]) for sample in samples]
  endeff_x = [pose[0] for pose in endeff_poses]
  endeff_y = [pose[1] for pose in endeff_poses]
  time = [sample["time"] for sample in samples]

  max_pivot_accel = [constants.pivot_accel_scaling(elevator_lengths[k]) for k in range(len(samples))]
  min_pivot_accel = [-x for x in max_pivot_accel]

  plt.figure(figsize=(16, 12))
  plt.suptitle(f"{traj_path}", fontsize=14)

  # Pivot state
  plt.subplot(3, 2, 1)
  plt.plot(time, [math.degrees(angle) for angle in pivot_angles], label="Pivot Angle (deg)")
  plt.xlabel("Time (s)")
  plt.ylabel("Angle (deg)")
  plt.legend()
  plt.grid()

  plt.subplot(3, 2, 2)
  plt.plot(time, pivot_velocities, label="Pivot Velocity (rad/s)", color="orange")
  plt.xlabel("Time (s)")
  plt.ylabel("Velocity (rad/s)")
  plt.legend()
  plt.grid()

  # Elevator state
  plt.subplot(3, 2, 3)
  plt.plot(time, elevator_lengths, label="Elevator Length (m)", color="green")
  plt.xlabel("Time (s)")
  plt.ylabel("Length (m)")
  plt.legend()
  plt.grid()

  plt.subplot(3, 2, 4)
  plt.plot(time, elevator_velocities, label="Elevator Velocity (m/s)", color="red")
  plt.xlabel("Time (s)")
  plt.ylabel("Velocity (m/s)")
  plt.legend()
  plt.grid()

  # End-effector position
  plt.subplot(3, 2, 5)
  cmap = plt.cm.viridis
  for i in range(len(endeff_x) - 1):
    plt.plot(endeff_x[i:i+2], endeff_y[i:i+2], color=cmap(i/len(endeff_x)), lw=2)
  plt.scatter(0, 0, color="black", label="Origin", zorder=5)
  for i, waypoint in enumerate(waypoint_poses):
    name = "Start" if i == 0 else "End" if i == len(waypoint_poses) - 1 else f"Mid-waypoint #{i}"
    plt.scatter(waypoint[0], waypoint[1], label=name, zorder=5)
  plt.xlabel("X Position (m)")
  plt.ylabel("Y Position (m)")
  plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
  plt.grid()
  plt.title("End-effector Position")
  plt.gca().set_aspect('equal', adjustable='box')

  # Accelerations
  plt.subplot(3, 2, 6)
  plt.plot(time, pivot_accelerations, label="Pivot Acceleration (rad/s^2)", color="purple")
  plt.plot(time, elevator_accelerations, label="Elevator Acceleration (m/s^2)", color="cyan")
  plt.plot(time, max_pivot_accel, color="purple", linestyle=":", label="Max Pivot Acceleration")
  plt.plot(time, min_pivot_accel, color="purple", linestyle=":", label="Min Pivot Acceleration")
  plt.xlabel("Time (s)")
  plt.ylabel("Acceleration")
  plt.legend(loc='upper right', bbox_to_anchor=(-0.05, 1))
  plt.grid()

  plt.tight_layout(rect=[0, 0, 1, 0.96])
  plt.show()

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Visualize multiple trajectories")
  parser.add_argument("traj", type=str, nargs='+', help="Paths to trajectory files")
  args = parser.parse_args()
  
  for traj in args.traj:
    plot_traj(traj)
