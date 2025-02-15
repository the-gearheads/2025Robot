import os
import constants
import json
import math

front_x = 20 * 0.0254
back_x = 0

samples_per_meter_travelled = 200

# all randomly guessed placeholders
places = {
  "L2": {"pose": [back_x+0.01, 0.92], "vel": [0, 0]},
  "L3": {"pose": [back_x+0.01, 1.6], "vel": [0, 0]},
  "L4": {"pose": [back_x+0.01, 1.8], "vel": [0, 0]},
  "I-": {"pose": [back_x+0.2, 1], "vel": [-0.2, -0.2]},
  "I+": {"pose": [back_x+0.2, 1], "vel": [0.2, 0.2]},
  "HP": {"pose": [front_x, 1], "vel": [0, 0]},
  "NET": {"pose": [back_x+0.01, constants.elevator_max_len - 0.09], "vel": [0, 0]},
}

trajs_to_generate = [
  "I- >> L2",
  "I- >> L3",
  "I- >> L4",
  "I- >> NET",
  "I- >> I+",
  "I+ >> I-",
  "HP >> L2",
  "HP >> L3",
  "HP >> L4",
  "HP >> NET",
  "NET >> HP",
  "NET >> L2",
  "NET >> L3",
  "NET >> L4",
  "L2 >> HP",
  "L2 >> NET",
  "L2 >> L3",
  "L2 >> L4",
  "L3 >> HP",
  "L3 >> NET",
  "L3 >> L2",
  "L3 >> L4",
  "L4 >> HP",
]


def get_elevator_len_arm_angle(x, y):
  return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

def generate_ungenerated_traj_file(line: str):
  line = line.strip().replace(" ", "")
  start_end = line.split(sep=">>")
  start = start_end[0]
  end = start_end[1]
  print(start_end)
  start_json = places[start].copy()
  end_json = places[end].copy()
  start_json["at_sample"] = 0
  end_json["at_sample"] = 1
  start_state = get_elevator_len_arm_angle(start_json["pose"][0], start_json["pose"][1])
  end_state = get_elevator_len_arm_angle(end_json["pose"][0], end_json["pose"][1])
  traj_json = {
    "name": f"{start}>>{end}",
    "waypoints": [start_json, end_json],
    # "N": int(math.sqrt((start_json["pose"][0] -j end_json["pose"][0])**2 + (start_json["pose"][1] - end_json["pose"][1])**2) * samples_per_meter_travelled),
  }

  if abs(end_state[1] - start_state[1]) < math.radians(2): # it borks out sometimes if pivot doesnt change
    traj_json["use_pivot_accel_scaling"] = False

  with open(f"{os.path.dirname(__file__)}/trajs/{start}>>{end}.atraj", "w") as f:
    f.write(json.dumps(traj_json, indent=2))

for traj in trajs_to_generate:
  generate_ungenerated_traj_file(traj)