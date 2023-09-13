import argparse
import pathlib

import h5py
import matplotlib.pyplot as plt

HDF5_EXTENSION  = ".h5"


def accuracy(true_positive: int,  true_negative: int,
             false_positive: int, false_negative: int) -> float:
    """
    Returns the accuracy value for the given confusion matrix.
    """
    return (true_positive + true_negative) / float(true_positive + false_positive + true_negative + false_negative)


def precision(true_positive: int, false_positive: int) -> float:
    """
    Returns the precision value for the given confusion matrix.
    """
    return true_positive / float(true_positive + false_positive)


def get_metric_occupancy_confusion_group(hdf5_path: pathlib.Path) -> h5py.Group:
    """
    Opens an HDF5 file and access the location of the Confusion Matrix data.
    """
    h5_file  = h5py.File(hdf5_path, "r")
    h5_group = h5_file["Metric"]["OccupancyConfusion"]
    return h5_group


def plot_raw_confusion(hdf5_path: pathlib.Path, data: h5py.Dataset, labels: list[str],
                       parsed_args: argparse.Namespace = None):
    """
    Creates a plot of the true/false positive/negative values.
    May plot the count of unknown values if the flag `--raw-unknown` was provided.
    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    fig.suptitle(hdf5_path)
    ax.set_title("Raw Confusion Values")
    ax.set_xlabel("Views Added")
    ax.set_ylabel("Voxel Count")

    ax.plot(data[:, 0], data[:, 1], label=labels[1])
    ax.plot(data[:, 0], data[:, 2], label=labels[2])
    ax.plot(data[:, 0], data[:, 3], label=labels[3])
    ax.plot(data[:, 0], data[:, 4], label=labels[4])

    if parsed_args and parsed_args.raw_unknown:
        ax.plot(data[:, 0], data[:, 5], label=labels[5])

    ax.legend()

    if parsed_args and parsed_args.display_only:
        plt.show()
    else:
        image_fpath = hdf5_path.parent
        image_fpath /= "acc_pre.png"
        plt.savefig(image_fpath)
    plt.close()


def plot_acc_pre(hdf5_path: pathlib.Path, data: h5py.Dataset,
                 parsed_args: argparse.Namespace = None):
    """
    Creates a plot of the accuracy and precision at each step.
    """
    acc = [ accuracy(data[i, 1], data[i, 2], data[i, 3], data[i, 4]) for i in range(data.shape[0])]
    pre = [precision(data[i, 1], data[i, 3]) for i in range(data.shape[0])]


    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    fig.suptitle(hdf5_path)
    ax.set_title("Reconstruction Accuracy and Precision")
    ax.set_xlabel("Views Added")

    ax.plot(data[:, 0], acc, "r:", linewidth=2, label='Accuracy')
    ax.plot(data[:, 0], pre, "b",  linewidth=2, label='Precision')

    ax.legend()

    if parsed_args and parsed_args.display_only:
        plt.show()
    else:
        image_fpath = hdf5_path.parent
        image_fpath /= "acc_pre.png"
        plt.savefig(image_fpath)
    plt.close()


def plot_file(hdf5_path: pathlib.Path, parsed_args: argparse.Namespace):
    """
    Opens an HDF5 file and accesses the data before calling the plotting functions.
    """
    confusion_group = get_metric_occupancy_confusion_group(hdf5_path)
    confusion_data: h5py.Dataset = confusion_group.get("data")
    confusion_labels: list[str]  = confusion_data.attrs["header"]
    if parsed_args.plot_raw:
        plot_raw_confusion(hdf5_path, confusion_data, confusion_labels, parsed_args)

    plot_acc_pre(hdf5_path, confusion_data, parsed_args)


def main(parsed_args: argparse.Namespace):
    """
    Program entry point.
    """
    fpath: pathlib.Path = parsed_args.file
    if not fpath.exists():
        print("File or directory does not exits. Please check your path.")
        return

    # Convert a single file to a list or a generate a list of all HDF5 files in a directory.
    fpath_list = [fpath] if fpath.is_file() else list(fpath.glob(f"**/*{HDF5_EXTENSION}"))

    n = len(fpath_list)
    for i, hdf5_path in enumerate(fpath_list):
        print(f"({i} / {n}) Plotting for:\n\t{hdf5_path}")
        plot_file(hdf5_path, parsed_args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Plot Confusion',
        description='Reads a ForgeScan HDF5 file and plots the confusion matrix data, if such'
                    'Data is present.'
    )
    parser.add_argument(
        "-f", "--file",
        type=pathlib.Path,
        required=True,
        help="Path to the HDF5 file to load or directory with HDF5 files within it."
    )
    parser.add_argument(
        "-s", "--display-only",
        action="store_true",
        default=False,
        help="If true, will only display the plot but not save it."
    )
    parser.add_argument(
        "--plot-raw",
        action="store_true",
        default=False,
        help="Display a figure of the raw true/false positive/negative values."
    )
    parser.add_argument(
        "--raw-unknown",
        action="store_true",
        default=False,
        help="If plotting raw data will plot the count of unknown data too."
    )
    args = parser.parse_args()

    main(args)
