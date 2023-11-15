import argparse
import pathlib
import subprocess

EXECUTABLE_NAME = 'MakeGroundTruth'
EXECUTABLE_PATH = None

# Find the project root and binary directory and the executable (regardless of its file extension).
PROJECT_ROOT_PATH = pathlib.Path(__file__).parent.resolve().parent
BINARIES_PATH     = PROJECT_ROOT_PATH / 'bin'
assert BINARIES_PATH.exists() and \
       BINARIES_PATH.is_dir(),    \
       f"Cannot find binary directory at: {BINARIES_PATH}"

for item in BINARIES_PATH.iterdir():
    if item.is_file() and item.name.find(EXECUTABLE_NAME) == 0:
        EXECUTABLE_PATH = item
        break

assert EXECUTABLE_PATH is not None and \
       EXECUTABLE_PATH.exists() and    \
       EXECUTABLE_PATH.is_file(),      \
       f"Cannot find executable at: {EXECUTABLE_PATH}"

# File constants
SPHERE        = "sphere"
BOX           = "box"
BOX_ROTATED   = "box-rotated"
BIN           = "bin"
BUNNY         = "bunny"
ROTOR_BLADE   = "rotor-blade"
ROTOR_BLADE_REAL   = "rotor-blade-real"
STDIN_NEWLINE = "\n"


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
    return " y " + STDIN_NEWLINE + " y " + STDIN_NEWLINE


def get_sphere() -> str:
    """
    Returns a stdin string to add the sphere mesh to the scene.
    """
    stdin_shape  =  "--file sphere.stl "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_box() -> str:
    """
    Returns a stdin string to add the box mesh to the scene.
    """
    stdin_shape  = " --file box.stl "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_box_rotated() -> str:
    """
    Returns a stdin string to add the box mesh to the scene.
    Performs rotation on the mesh so voxel are not aligned with any of the mesh faces or edges.
    """
    stdin_shape  = " --file box.stl "
    stdin_shape += get_rotation(23.81, 15.92, 0, degrees=True)
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_bin() -> str:
    """
    Returns a stdin string to add the bin mesh to the scene.
    """
    stdin_shape  = " --file bin.stl "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_bunny() -> str:
    """
    Returns a stdin string to add the stanford bunny mesh to the scene.
    Scales the mesh to fit the sizing of everything else.
    """
    stdin_shape  = " --file bunny.stl "
    stdin_shape += get_translation(-0.2, 0, -0.4)
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_rotor_blade() -> str:
    """
    Returns a stdin string to add the rotor blade to the scene.
    Scales the mesh to fit the sizing of everything else.
    """
    stdin_shape  = " --file rotor_blade.stl --scale 0.3 "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_rotor_blade_real() -> str:
    """
    Returns a stdin string to add the rotor blade to the scene.
    Scales the mesh to fit the sizing of the real part
    """
    stdin_shape  = " --file rotor_blade.stl --scale 0.02 "
    stdin_shape += STDIN_NEWLINE # End shape
    return stdin_shape


def get_reconstruction_grid(parsed_args: argparse.Namespace) -> str:
    """
    Returns a stdin string to set the grid properties, rotation, and translation based on the user's
    provided ground truth name.
    """
    stdin = ""
    if (parsed_args.name == SPHERE):
        stdin += get_grid_properties(resolution=0.01) + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)                + STDIN_NEWLINE
        stdin += get_translation(-0.5, -0.5, -0.5)    + STDIN_NEWLINE
    elif (parsed_args.name == BOX):
        stdin += get_grid_properties()       + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)       + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == BOX_ROTATED):
        stdin += get_grid_properties()       + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)       + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == BIN):
        stdin += get_grid_properties()         + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)         + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == BUNNY):
        stdin += get_grid_properties()         + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)         + STDIN_NEWLINE
        stdin += get_translation(-1, -1, -1) + STDIN_NEWLINE
    elif (parsed_args.name == ROTOR_BLADE):
        stdin += get_grid_properties(nx=151, ny=451, nz=101, resolution=0.01) + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)            + STDIN_NEWLINE
        stdin += get_translation(-0.75, -2.25, -0.5) + STDIN_NEWLINE
    elif (parsed_args.name == ROTOR_BLADE_REAL):
        stdin += get_grid_properties(nx=201, ny=501, nz=101, resolution=0.0005) + STDIN_NEWLINE
        stdin += get_rotation(0, 0, 0)            + STDIN_NEWLINE
        stdin += get_translation(-0.05, -0.125, -0.025) + STDIN_NEWLINE
    else:
        raise ValueError("Mesh name is invalid.")
    return stdin


def get_scene(parsed_args: argparse.Namespace) -> str:
    """
    Returns a stdin string to add shapes to the scene based on the user's specified ground truth name.
    """
    if (parsed_args.name == SPHERE):
        stdin_shape = get_sphere()
    elif (parsed_args.name == BOX):
        stdin_shape = get_box()
    elif (parsed_args.name == BOX_ROTATED):
        stdin_shape = get_box_rotated()
    elif (parsed_args.name == BIN):
        stdin_shape = get_bin()
    elif (parsed_args.name == BUNNY):
        stdin_shape = get_bunny()
    elif (parsed_args.name == ROTOR_BLADE):
        stdin_shape = get_rotor_blade()
    elif (parsed_args.name == ROTOR_BLADE_REAL):
        stdin_shape = get_rotor_blade_real()
    else:
        raise ValueError("Mesh file is invalid.")
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
        help="Which mesh to load."
    )
    parser.add_argument(
        "-d", "--dir",
        type=pathlib.Path,
        required=False,
        default=PROJECT_ROOT_PATH / "share" / "Experiments" / "GroundTruth",
        help="Where to save the generated data."
    )

    args = parser.parse_args()
    main(args)
