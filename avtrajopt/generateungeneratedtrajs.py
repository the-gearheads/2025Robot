import os
import constants
import json
import math
import itertools

front_x = 20 * 0.0254
back_x = 0

samples_per_meter_travelled = 200

def endeff_pos(ext_len, theta_deg):
  x = (constants.elevator_min_len + ext_len) * math.cos(math.radians(theta_deg))
  y = (constants.elevator_min_len + ext_len) * math.sin(math.radians(theta_deg))
  return x, y


# mostly randomly guessed placeholders
places = {
  "L1": {"pose": endeff_pos(0.06, 50), "vel": [0, 0]}, # placeholder
  "L2": {"pose": endeff_pos(0.20, 50), "vel": [0, 0]},
  "L3": {"pose": endeff_pos(0.06, 86), "vel": [0, 0]},
  "L4": {"pose": endeff_pos(0.88, 86), "vel": [0, 0]},

  "HP": {"pose": [front_x, 1], "vel": [0, 0]},
  "GROUND_INTAKE": {"pose": [front_x, 0.92], "vel": [0, 0]},
  "PROCESSOR": {"pose": [front_x, 0.8], "vel": [0, 0]},
  "NET": {"pose": [back_x+0.01, constants.elevator_max_len - 0.09], "vel": [0, 0]},
}


def get_elevator_len_arm_angle(x, y):
  return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

def generate_ungenerated_traj_file(start: str, end: str):
  print([start, end])
  start_json = places[start].copy()
  end_json = places[end].copy()
  start_json["at_sample"] = 0
  end_json["at_sample"] = 1
  start_state = get_elevator_len_arm_angle(start_json["pose"][0], start_json["pose"][1])
  end_state = get_elevator_len_arm_angle(end_json["pose"][0], end_json["pose"][1])
  traj_json = {
    "name": f"{start},{end}",
    "waypoints": [start_json, end_json],
    # "N": int(math.sqrt((start_json["pose"][0] -j end_json["pose"][0])**2 + (start_json["pose"][1] - end_json["pose"][1])**2) * samples_per_meter_travelled),
  }

  if abs(end_state[1] - start_state[1]) < math.radians(2): # it borks out sometimes if pivot doesnt change
    traj_json["use_pivot_accel_scaling"] = False

  with open(f"{os.path.dirname(__file__)}/trajs/{start},{end}.atraj", "w") as f:
    f.write(json.dumps(traj_json, indent=2))

def generate_armvatorposition_java():
  positions = ""
  for i, place in enumerate(places):
    positions += f"  {place}(new Translation2d({places[place]["pose"][0]}, {places[place]["pose"][1]}))"
    if i != len(places) - 1:
      positions += ","
    else:
      positions += ";"
    positions += "\n"
  with open(f"{os.path.dirname(__file__)}/ArmvatorPosition.java.template", "r") as f:
    template = f.read()
  with open(f"{os.path.dirname(__file__)}/../src/main/java/frc/robot/util/ArmvatorPosition.java", "w") as f:
    f.write(template.replace("{{ENDEFF_POSITIONS}}", positions))

generate_armvatorposition_java()
for start, end in itertools.permutations(places.keys(), 2):
  generate_ungenerated_traj_file(start, end)