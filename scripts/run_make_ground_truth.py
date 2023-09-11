import argparse
import pathlib
import subprocess
import math

EXECUTABLE_NAME = 'MakeGroundTruth'
EXECUTABLE_PATH = None

# Find the project root and binary directory and the executable (regardless of its file extension).
PROJECT_ROOT_PATH = pathlib.Path(__file__).parent.resolve().parent
BINARIES_PATH     = PROJECT_ROOT_PATH / 'bin'
assert(BINARIES_PATH.exists() and 
       BINARIES_PATH.is_dir() and 
      "Cannot find binary directory.")

for item in BINARIES_PATH.iterdir():
    if item.is_file() and item.name.find(EXECUTABLE_NAME) == 0:
        EXECUTABLE_PATH = item
        break

assert(EXECUTABLE_PATH is not None and 
       EXECUTABLE_PATH.exists() and 
       EXECUTABLE_PATH.is_file() and
       "Cannot find executable.")

# File constants
SPHERE_TRUTH_NAME = "sphere"
BOX_TRUTH_NAME    = "box"
BIN_TRUTH_NAME    = "bin"
BAR_TRUTH_NAME    = "bar"

STDIN_NEWLINE     = "\n" 


def get_grid_properties(nx: float = 101, ny: float = 101, nz: float = 101, resolution: float = 0.02) -> str:
    """
    Returns an stdin string to use a uniformly shaped cube as the reconstruction grid.
    Adds newline to finish stdin.
    """
    return f" --nx {nx} --ny {ny} --nz {nz} --resolution {resolution} "


def get_rotation(rx: float = 0, ry: float = 0, rz: float = 0, degrees: bool = False) -> str:
    """
    Returns a stdin string to set the rotation.
    Note, no new line is added.
    """
    return f" --rx {rx} --ry {ry} --rz {rz}" + (" --degrees " if degrees is True else " ")


def get_translation(x: float = 0, y: float = 0, z: float = 0) -> str:
    """
    Returns a stdin string to set the translation.
    Note, no new line is added.
    """
    return f" --x {x} --y {y} --z {z} "


def confirm_generate_data() -> str:
    """
    Returns a stdin string to confirm the generation of the occupancy and TSDF data.
    """
    return "y" + STDIN_NEWLINE + "y" + STDIN_NEWLINE


def get_sphere() -> str:
    """
    Returns a stdin string to add the ground truth test sphere to the scene.
    """
    stdin_shape = ""

    radius = math.pi / 6

    stdin_shape += f" --name truth_sphere --shape sphere --radius {radius} "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_box() -> str:
    """
    Returns a stdin string to add the ground truth test box to the scene.
    """
    stdin_shape = ""

    length = math.pi / 6
    width  = math.e  / 2.5
    height = (length + width) / 2

    stdin_shape = ""

    stdin_shape += f" --name truth_box --shape box --l {length} --w {width} --h {height} "
    stdin_shape += get_rotation(23.81, 15.92, 0, degrees=True)
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_bin() -> str:
    """
    Returns a stdin string to add the ground truth test bin to the scene.
    """
    stdin_shape = ""

    # Base dimensions
    bx = 1.20
    by = 0.86
    bz = 0.25

    # Wall dimensions
    wz = 1
    wt = 0.30


    # Dimensioning helpers
    dx = 0.5 * bx

    dy = 0.5 * by


    # Base 
    stdin_shape += f" --name base --shape box --l {bx} --w {by} --h {bz} "
    stdin_shape += get_translation(0, 0, -0.5*bz)
    stdin_shape += STDIN_NEWLINE # End shape

    # Walls on X
    stdin_shape += f" --name wall_nx --shape box --l {wt} --w {by} --h {wz} "
    stdin_shape += get_translation(-dx, 0, 0.5*wz)
    stdin_shape += STDIN_NEWLINE # End shape
    stdin_shape += f" --name wall_px --shape box --l {wt} --w {by} --h {wz} "
    stdin_shape += get_translation(dx, 0, 0.5*wz)
    stdin_shape += STDIN_NEWLINE # End shape

    # Walls on Y
    stdin_shape += f" --name wall_ny --shape box --l {bx} --w {wt} --h {wz} "
    stdin_shape += get_translation(0, -dy, 0.5*wz)
    stdin_shape += STDIN_NEWLINE # End shape
    stdin_shape += f" --name wall_py --shape box --l {bx} --w {wt} --h {wz} "
    stdin_shape += get_translation(0, dy, 0.5*wz)
    stdin_shape += STDIN_NEWLINE # End shape

    return stdin_shape


def get_bar() -> str:
    """
    Returns a stdin string to add the ground truth test bar to the scene.
    """
    stdin_shape = ""

    stdin_shape = ""

    # Height
    rx = 0.25

    # Rod dimensions
    ry = 0.20
    rz = 0.75

    # Connector dimensions
    cz = 0.4
    ct = 0.1
    cy = ry + cz

    # End sphere radius
    rs = 0.5 * min(rx, ry)

    # Dimensioning helpers
    dy = 0.5*(cz + ct)

    dz0 = 0.5*rz + rs
    dz1 = rs + rz + 0.5*ct
    dz2 = dz1 + cz
    dz3 = rs + rz + 0.5*(cz + ct)


    # Sphere
    stdin_shape += f" --name sphere --shape sphere --radius {rs} "
    stdin_shape += get_translation(0, 0, rs)
    stdin_shape += STDIN_NEWLINE # End shape

    # Rod 
    stdin_shape += f" --name rod --shape box --l {rx} --w {ry} --h {rz} "
    stdin_shape += get_translation(0, 0, dz0)
    stdin_shape += STDIN_NEWLINE # End shape

    # Connector walls on Z
    stdin_shape += f" --name wall_nz --shape box --l {rx} --w {cy} --h {ct} "
    stdin_shape += get_translation(0, 0, dz1)
    stdin_shape += STDIN_NEWLINE # End shape
    stdin_shape += f" --name wall_pz --shape box --l {rx} --w {cy} --h {ct} "
    stdin_shape += get_translation(0, 0, dz2)
    stdin_shape += STDIN_NEWLINE # End shape

    # Connector walls on Y
    stdin_shape += f" --name wall_ny --shape box --l {rx} --w {ct} --h {cz} "
    stdin_shape += get_translation(0, -dy, dz3)
    stdin_shape += STDIN_NEWLINE # End shape
    stdin_shape += f" --name wall_py --shape box --l {rx} --w {ct} --h {cz} "
    stdin_shape += get_translation(0, dy, dz3)
    stdin_shape += STDIN_NEWLINE # End shape

    return stdin_shape


def get_reconstruction_grid(parsed_args: argparse.Namespace) -> str:
    """
    Returns a stdin string to set the grid properties, rotation, and translation based on the user's
    provided ground truth name.
    """
    stdin = ""
    if (parsed_args.name == SPHERE_TRUTH_NAME):
        stdin += get_grid_properties()       + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)       + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == BOX_TRUTH_NAME):
        stdin += get_grid_properties()       + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)       + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == BIN_TRUTH_NAME):
        stdin += get_grid_properties()         + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)         + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -0.5) + STDIN_NEWLINE
    elif (parsed_args.name == BAR_TRUTH_NAME):
        stdin += get_grid_properties(121, 161, 321, 0.005)  + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)             + STDIN_NEWLINE
        stdin += get_translation(-0.3, -0.4, -0.1) + STDIN_NEWLINE
    else:
        raise ValueError("Shape name is invalid.")
    return stdin


def get_scene(parsed_args: argparse.Namespace) -> str:
    """
    Returns a stdin string to add shapes to the scene based on the user's specified ground truth name.
    """
    if (parsed_args.name == SPHERE_TRUTH_NAME):
        stdin_shape = get_sphere()
    elif (parsed_args.name == BOX_TRUTH_NAME):
        stdin_shape = get_box()
    elif (parsed_args.name == BIN_TRUTH_NAME):
        stdin_shape = get_bin()
    elif (parsed_args.name == BAR_TRUTH_NAME):
        stdin_shape = get_bar()
    else:
        raise ValueError("Shape name is invalid.")
    stdin_shape += STDIN_NEWLINE
    return stdin_shape



def main(parsed_args: argparse.Namespace) -> None:
    """
    Program entry point.
    """
    fpath: pathlib.Path = parsed_args.dir
    if fpath.exists() is False:
        fpath.mkdir(parents=True)
    fpath /= parsed_args.name
    print(fpath)

    proc = subprocess.Popen(executable=EXECUTABLE_PATH, args=[], text=True,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    stdin  = str(fpath) + STDIN_NEWLINE
    stdin += get_reconstruction_grid(parsed_args)
    stdin += get_scene(parsed_args)
    stdin += confirm_generate_data()
    print(stdin)
    stdout = proc.communicate(stdin)
    print(stdout[0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Run Make Ground Truth',
        description='This script provides convenience to call the MakeGroundTruth executable with '
                    'specifically configured shapes and grid properties.'
    )
    parser.add_argument(
        "-n", "--name",
        type=str,
        required=False,
        default="sphere",
        help="Which collection of shapes to use."
    )
    parser.add_argument(
        "-d", "--dir",
        type=pathlib.Path,
        required=False,
        default=PROJECT_ROOT_PATH / "share" / "GroundTruth",
        help="Where to save the generated data."
    )

    args = parser.parse_args()
    main(args)
