import matplotlib.pyplot as plt
import argparse
import math
import constants
import json

# note: we can't assume constant dt as future iterations will change that to allow for proper impl of >2 waypoints
def plot_traj(traj):
  with open(traj, 'r') as f:
    data = json.load(f)
  
  samples = data["samples"]
  waypoint_poses = [waypoint["pose"] for waypoint in data["waypoints"]]

  pivot_angles = [sample["pivot_angle"] for sample in samples]
  pivot_velocities = [sample["pivot_velocity"] for sample in samples]
  elevator_lengths = [sample["elevator_length"] for sample in samples]
  elevator_velocities = [sample["elevator_velocity"] for sample in samples]
  pivot_accelerations = [sample["pivot_accel"] for sample in samples]
  elevator_accelerations = [sample["elevator_accel"] for sample in samples]
  endeff_x = [sample["endeff_pos"][0] for sample in samples]
  endeff_y = [sample["endeff_pos"][1] for sample in samples]
  time = [sample["time"] for sample in samples]

  max_pivot_accel = [constants.pivot_accel_scaling(elevator_lengths[k]) for k in range(len(samples))]
  min_pivot_accel = [-x for x in max_pivot_accel]

  # Pivot state
  plt.figure(figsize=(16, 12))
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
  # plt.scatter(endeff_x[0], endeff_y[0], color="blue", label="Start Point", zorder=5)
  # plt.scatter(endeff_x[-1], endeff_y[-1], color="red", label="End Point", zorder=5)
  for i, waypoint in enumerate(waypoint_poses):
    if i == 0:
      name = "Start"
    elif i == len(waypoint_poses) - 1:
      name = "End"
    else:
      name = f"Mid-waypoint #{i}"
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

  plt.tight_layout()
  plt.show()

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Visualize trajectory")
  parser.add_argument("traj", type=str, help="Path to trajectory file")
  args = parser.parse_args()
  plot_traj(args.traj)

