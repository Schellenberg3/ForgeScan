import pathlib
import os

import matplotlib.pyplot as plt
import matplotlib as mpl
import h5py


CURRENT_DIR = pathlib.Path(os.path.dirname(__file__))
FORGESCAN_SHARE_PARAVIEW_DIR = pathlib.Path.joinpath(CURRENT_DIR.parent, pathlib.Path("share/ParaView"))


def main():
    """
    Main function.
    Draws each Sensor's view position and camera normal direction.
    """
    h5_fpath = FORGESCAN_SHARE_PARAVIEW_DIR.joinpath("demo_policy.h5")
    print("Is file?", h5_fpath.is_file(), "\nFile path:", h5_fpath)
    demo_policy_h5 = h5py.File(h5_fpath, "r")
    sensor_record_g : h5py.Group = demo_policy_h5["Metrics"]["SensorRecord"]

    extrinsics     = []
    total_first    = []
    total_views    = []
    total_updates  = []
    max_var_update = []
    for value in sensor_record_g.values():
        # Transpose of the pose matrix is required due to HighFive bug when writing Eigen matrices.
        extrinsics.append(    value.attrs["pose"].transpose())
        total_first.append(   value.attrs["total_first"])
        total_views.append(   value.attrs["total_views"])
        total_updates.append( value.attrs["total_updates"])
        max_var_update.append(value.attrs["max_variance_update"])

    ax = plt.figure().add_subplot(projection="3d")

    ## Plot the views
    cmap = mpl.colormaps['Spectral']
    len_adj = len(extrinsics) - 1
    for (i, extr) in enumerate(extrinsics):
        x, y, z = extr[:3, 3]
        u, v, w = extr[:3, 2]
        ax.text(x, y, z, i, color=cmap(i/len_adj))
        ax.quiver(x, y, z, u, v, w, length=1, normalize=True, color=cmap(i/len_adj))

    plt.show()

if __name__ == "__main__":
    main()
