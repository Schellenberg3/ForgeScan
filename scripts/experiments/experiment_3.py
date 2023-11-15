"""
Runs experiments for sphere and box geometries across all levels of noise for ordered and
un-ordered spherical sampling.
"""

import argparse
import pathlib
import subprocess
import random


## ----------------------- LOCATE EXECUTABLE FILES AND GROUND TRUTH DATA ----------------------- ##

HDF5_EXTENSION  = ".h5"
EXECUTABLE_NAME = 'RunExperiment'
EXECUTABLE_PATH = None
TRUTH_FILES = ["rotor-blade", "bunny"]

# Find the project root and binary directory and the executable (regardless of its file extension).
PROJECT_ROOT_PATH = pathlib.Path(__file__).parent.resolve().parent.parent
BINARIES_PATH     = PROJECT_ROOT_PATH / 'bin'
assert BINARIES_PATH.exists() and \
       BINARIES_PATH.is_dir(),    \
       f"Cannot find binary directory: {BINARIES_PATH}"

for item in BINARIES_PATH.iterdir():
    if item.is_file() and item.name.find(EXECUTABLE_NAME) == 0:
        EXECUTABLE_PATH = item
        break

assert EXECUTABLE_PATH is not None and \
       EXECUTABLE_PATH.exists() and    \
       EXECUTABLE_PATH.is_file(),      \
       F"Cannot find executable in: {EXECUTABLE_PATH}"


# Find the GroundTruth directory and all of the HSF5 scene files in it.
GROUND_TRUTH_PATH = PROJECT_ROOT_PATH / "share" / "Experiments" / "GroundTruth"
assert GROUND_TRUTH_PATH.exists() and \
       GROUND_TRUTH_PATH.is_dir(),    \
       f"Cannot find GroundTruth directory: {GROUND_TRUTH_PATH}"

GROUND_TRUTH_FILES: list[pathlib.Path] = []
for item in GROUND_TRUTH_PATH.iterdir():
    if item.is_file() and item.name.endswith(HDF5_EXTENSION):
        fname = item.name.removesuffix(HDF5_EXTENSION)
        if fname in TRUTH_FILES:
            GROUND_TRUTH_FILES.append(item)

assert len(GROUND_TRUTH_FILES) == 2, \
       f"Found {len(GROUND_TRUTH_FILES)} of 2 expected files in: {GROUND_TRUTH_PATH} \
         \n\tfiles were: {GROUND_TRUTH_FILES}]"


## ----------------------------------- DEFINE FILE CONSTANTS ----------------------------------- ##


STDIN_NEWLINE = "\n"

# Exploration space
REGULAR_RERUNS = 1
RANDOM_RERUNS  = 10

VIEW_RADIUS = 2.5
REJECTION_RATE = 0.0

# Use RealSense uncertainty model
DIST = 0.02 * VIEW_RADIUS

RANDOM_SEED = False

N_VIEWS = [9]


# ([d1], [d2], [noise percentage])
#     tsdf        = [-d1, +d1]
#     probability = [-d1, +d2]
DIST_AND_NOISE: list[tuple[float, float, float]] = [
    (
        0.06, 0.03, 0.0
    ),
    (
        0.06, 0.03, 0.02
    ),
    (
        0.18, 0.10, 0.06
    )
]


# ([Name], [Intrinsic args])
INTRINSICS: list[tuple[str, str, int]] = [
    (
        f"RealSense_D455_Noise_{int(DIST_AND_NOISE[0][2]*100)}",
        f"--d455 1.0 --noise {DIST_AND_NOISE[0][2]}",
        0
    ),
    (
        f"RealSense_D455_Noise_{int(DIST_AND_NOISE[1][2]*100)}",
        f"--d455 1.0 --noise {DIST_AND_NOISE[1][2]}",
        1
    ),
    (
        f"RealSense_D455_Noise_{int(DIST_AND_NOISE[2][2]*100)}",
        f"--d455 1.0 --noise {DIST_AND_NOISE[2][2]}",
        2
    ),
]


# ([Name], [Policy arg string], [Number of re-runs])
METHODS: list[tuple[str, str, str]] = [
    (
        "Occplane",
        f"--type occplane --keep-top-n 3 --complete-after 9 --radius {str(VIEW_RADIUS)} " ,
        RANDOM_RERUNS
    )
]


## ------------------------------ SCRIPT METHODS AND ENTRY POINT ------------------------------- ##


def call_process(fpath: pathlib.Path, scene: pathlib.Path, intr: list[tuple[str, str, int]], policy_name: str,
                 policy: str, n_views: int, seed: int = 0, parsed_args: argparse.Namespace = None):
    stdin  = ""
    stdin += str(fpath) + STDIN_NEWLINE
    stdin += ("y" if parsed_args.save_images else "don't save images") + STDIN_NEWLINE
    stdin += str(scene) + STDIN_NEWLINE
    stdin += str(REJECTION_RATE) + STDIN_NEWLINE
    stdin += intr[1] + STDIN_NEWLINE
    stdin += policy
    stdin += " --n-views " + str(n_views)

    # Occplane does not accept `seed` as an option yet.
    stdin += " --seed "  + str(seed) + STDIN_NEWLINE

    # Add data channels
    stdin += f"--name probability   --type Probability  --d-min -{DIST_AND_NOISE[intr[2]][0]} --d-max {DIST_AND_NOISE[intr[2]][1]} --dtype float" + STDIN_NEWLINE
    stdin += f"--name TSDF          --type TSDF         --d-min -{DIST_AND_NOISE[intr[2]][0]} --d-max {DIST_AND_NOISE[intr[2]][0]} --dtype float" + STDIN_NEWLINE
    stdin +=  "--name binary --type binary" + STDIN_NEWLINE

    # Add metrics
    stdin += STDIN_NEWLINE

    proc = subprocess.Popen(executable=EXECUTABLE_PATH, args=[], text=True,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    stdout = proc.communicate(stdin)
    if parsed_args is not None and parsed_args.pause:
        print(stdin)
        print(stdout[0])
        input("End of process...")


def main(parsed_args: argparse.Namespace) -> None:
    """
    Program entry point.
    """
    start = parsed_args.start_at
    n = 0
    N = len(INTRINSICS) * len(GROUND_TRUTH_FILES) * \
        len(N_VIEWS) * sum([m[2] for m in METHODS])
    print(f"Generating {N} experiments, beginning at experiment {start}...")

    fpath_base = PROJECT_ROOT_PATH / "share" / "Experiments" / "Experiment_3" / "Results"
    for intr in INTRINSICS:
        for scene in GROUND_TRUTH_FILES:
            for policy in METHODS:
                for n_views in N_VIEWS:
                    for rep in range(1, policy[2] + 1):
                        n += 1
                        if (n < start):
                            continue

                        seed = random.randrange(1, 1e8) if RANDOM_SEED else rep

                        fpath = pathlib.Path(intr[0], scene.name.removesuffix(HDF5_EXTENSION),
                                             policy[0], str(n_views), str(rep))
                        print(f"({n} / {N}) {fpath}")

                        fpath = fpath_base / fpath
                        if fpath.exists() is False:
                            fpath.mkdir(parents=True)
                        fpath /= "results" + HDF5_EXTENSION

                        if fpath.exists() is True and parsed_args.no_override:
                            print("\tAlready exists. Skipping experiment...")
                            continue

                        call_process(fpath, scene, intr, policy[0], policy[1], n_views, seed, parsed_args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Sweep Run Experiment',
        description='This script runs the Run Experiment script for a wide sweep of parameters.'
    )
    parser.add_argument(
        "-p", "--pause",
        action="store_true",
        default=False,
        help="Optionally pause after each experiment and display stdin/stdout."
    )
    parser.add_argument(
        "--no-override",
        action="store_true",
        default=False,
        help="Optionally skip an experiment if a file already exists for it."
    )
    parser.add_argument(
        "--start-at",
        type=int,
        default=0,
        help="Start some number into the sweep."
    )
    parser.add_argument(
        "--save-images",
        action="store_true",
        default=False,
        help="Saves images for each view."
    )

    args = parser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt as e:
        print("\nExiting early on KeyboardInterrupt.")

