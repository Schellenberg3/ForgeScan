import argparse
import pathlib

import h5py
import matplotlib.pyplot as plt


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


def get_metric_occupancy_confusion_group(fpath: pathlib.Path) -> h5py.Group:
    """
    Opens an HDF5 file and access the location of the Confusion Matrix data. 
    """
    h5_file  = h5py.File(fpath, "r")
    h5_group = h5_file["Metric"]["OccupancyConfusion"]
    return h5_group


def plot_raw_confusion(data: h5py.Dataset, labels: list[str], parsed_args: argparse.Namespace = None):
    """
    Creates a plot of the true/false positive/negative values.
    May plot the count of unknown values if the flag `--raw-unknown` was provided.
    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

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

    if parsed_args and parsed_args.save:
        fpath: pathlib.Path = parsed_args.save
        if fpath.exists() is False:
            fpath.mkdir(parents=True)
        fpath /= "raw_confusion.png"
        plt.savefig(fpath)
    else:
        plt.show()


def plot_acc_pre(data: h5py.Dataset, parsed_args: argparse.Namespace = None):
    """
    Creates a plot of the accuracy and precision at each step.
    """
    acc = [ accuracy(data[i, 1], data[i, 2], data[i, 3], data[i, 4]) for i in range(data.shape[0])]
    pre = [precision(data[i, 1], data[i, 3]) for i in range(data.shape[0])]
    
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    ax.set_title("Reconstruction Accuracy and Precision")
    ax.set_xlabel("Views Added")

    ax.plot(data[:, 0], acc, "r:", linewidth=2, label='Accuracy')
    ax.plot(data[:, 0], pre, "b",  linewidth=2, label='Precision') 

    ax.legend()

    if parsed_args and parsed_args.save:
        fpath: pathlib.Path = parsed_args.save
        if fpath.exists() is False:
            fpath.mkdir(parents=True)
        fpath /= "acc_pre.png"
        plt.savefig(fpath)
    else:
        plt.show()


def main(parsed_args: argparse.Namespace):
    """
    Program entry point.
    """
    confusion_group = get_metric_occupancy_confusion_group(parsed_args.file)
    confusion_data: h5py.Dataset = confusion_group.get("data")
    confusion_labels: list[str]  = confusion_data.attrs["header"]
    if parsed_args.plot_raw:
        plot_raw_confusion(confusion_data, confusion_labels, parsed_args)

    plot_acc_pre(confusion_data, parsed_args)


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
        help="Path to the HDF5 file to load."
    )
    parser.add_argument(
        "-s", "--save",
        type=pathlib.Path,
        required=False,
        help="Path to a directory to save the plots within."
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
