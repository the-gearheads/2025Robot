import trajgen
import os
import argparse
import concurrent.futures
from tqdm import tqdm


def generate(file, input_dir, output_dir, debug):
    filepath = os.path.join(input_dir, file)
    if debug:
      print(f"Generating trajectory for {file}")
    result = trajgen.main(filepath, output_dir, debug)
    if result != 0:
        print(f"(was generating trajectory for {file})")
    return file, result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a trajectory for the armvator")
    parser.add_argument("input_dir", type=str, help="The input directory for the trajectories")
    parser.add_argument("output_dir", type=str, help="The output directory for the trajectory")
    parser.add_argument("max_workers", type=int, help="Max # of workers, 0 for all", default=0)
    args = parser.parse_args()

    if args.max_workers == 0:
      args.max_workers = None

    files = [f for f in os.listdir(args.input_dir) if f.endswith(".atraj")]
    debug = args.max_workers == 1

    with concurrent.futures.ProcessPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {
            executor.submit(generate, f, args.input_dir, args.output_dir, debug): f
            for f in files
        }

        for future in tqdm(concurrent.futures.as_completed(futures), total=len(futures)):
            file, result = future.result()
            if result != 0:
                print(f"Error occurred while processing {file}, stopping further execution.")
                break
