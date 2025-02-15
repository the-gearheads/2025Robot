import trajgen
import os
import argparse


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate a trajectory for the armvator")
  parser.add_argument("input_dir", type=str, help="The input directory for the trajectories")
  parser.add_argument("output_dir", type=str, help="The output directory for the trajectory")
  args = parser.parse_args()

  for file in os.listdir(args.input_dir):
    if file.endswith(".atraj"):
      print(f"Generating trajectory for {file}")
      if trajgen.main(os.path.join(args.input_dir, file), args.output_dir) != 0:
        print(f"(was generating trajectory for {file})")
        break
