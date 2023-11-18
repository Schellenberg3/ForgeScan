import argparse
import pathlib

import h5py
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, MultipleLocator, AutoMinorLocator
import numpy as np


PROJECT_ROOT_PATH = pathlib.Path(__file__).parent.parent.resolve().parent


## --------------------------------------- DEFINE PLOTS ---------------------------------------- ##


def get_probability_figure(part: str, policy: str, plot_type: str,
                           reconstructions: list[str]) -> list[tuple[plt.Figure, plt.Axes]]:
    """
    Returns multiple matplotlib figures with different titles.
    """
    plots = []
    for label in reconstructions:
        fig, ax = plt.subplots(figsize=[5.5, 3.75], dpi=200)

        # fig_title = f"{label} Reconstruction of {part} using {policy}".replace("_", " ")
        # fig.suptitle(fig_title)

        ax.set_title(f"Reconstruction {plot_type}")
        # ax.set_xlabel("Views Added")
        ax.set_ylim(0, 1.1)
        ax.grid(axis='y', which='major', color='0.80')
        ax.grid(axis='y', which='minor', color='0.95')

        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.yaxis.set_ticks(list(np.linspace(0.2, 1, 5)))
        ax.yaxis.set_minor_locator(AutoMinorLocator())

        plots.append((fig, ax))
    return plots


def get_quantity_figure(part: str, policy: str, plot_type: str,
                        reconstructions: list[str]) -> list[tuple[plt.Figure, plt.Axes]]:
    """
    """
    plots = []
    for label in reconstructions:
        fig, ax = plt.subplots(figsize=[5.5, 3.75], dpi=200)

        # fig_title = f"{label} Reconstruction of {part} using {policy}".replace("_", " ")
        # fig.suptitle(fig_title)

        ax.set_title(f"Reconstruction {plot_type}")
        ax.grid(axis='y', which='major', color='0.80')
        ax.grid(axis='y', which='minor', color='0.95')
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))

        plots.append((fig, ax))
    return plots


## ------------------------------- CONFUSION MATRIX CALCULATIONS ------------------------------- ##

# Index location in the extracted Nx4 matrix
TP = 0
TN = 1
FP = 2
FN = 3


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


def sensitivity(true_positive: int, false_negative: int) -> float:
    """
    Returns the sensitivity or true positive rate (TPR) value for the given confusion matrix.
    """
    return true_positive / float(true_positive + false_negative)


def specificity(true_negative: int, false_positive: int) -> float:
    """
    Returns the specificity or true negative rate (TPR) value for the given confusion matrix.
    """
    return true_negative / float(true_negative + false_positive)


def fall_out(true_negative: int, false_positive: int) -> float:
    """
    Returns the fall-out or false positive rate (FPR) value for the given confusion matrix.
    """
    return false_positive / float(false_positive + true_negative)


def miss_rate(true_positive: int, false_negative: int) -> float:
    """
    Returns the miss rate or False negative rate (FNR) value for the given confusion matrix.
    """
    return false_negative / float(false_negative + true_positive)


def arr_accuracy(arr: np.ndarray):
    """
    Calls the `accuracy` function on the input array.
    """
    return accuracy(arr[TP], arr[TN], arr[FP], arr[FN])


def arr_precision(arr: np.ndarray):
    """
    Calls the `precision` function on the input array.
    """
    return precision(arr[TP], arr[FP])


def arr_sensitivity(arr: np.ndarray):
    """
    Calls the `sensitivity` function on the input array.
    """
    return sensitivity(arr[TP], arr[FN])


def arr_specificity(arr: np.ndarray):
    """
    Calls the `specificity` function on the input array.
    """
    return specificity(arr[TN], arr[FP])


def arr_balanced_accuracy(arr: np.ndarray):
    """
    Calls the `sensitivity` and `specificity` function on the input array to get the balanced accuracy.
    """
    return 0.5*( sensitivity(arr[TP], arr[FN]) + specificity(arr[TN], arr[FP]) )


def arr_fall_out(arr: np.ndarray):
    """
    Calls the `fall_out` function on the input array.
    """
    return fall_out(arr[TN], arr[FP])


def arr_miss_rate(arr: np.ndarray):
    """
    Calls the `miss_rate` function on the input array.
    """
    return miss_rate(arr[TP], arr[FN])


def arr_true_positive(arr: np.ndarray):
    """
    Returns the count of voxels labeled as true positive.
    """
    return arr[TP]

def arr_true_negative(arr: np.ndarray):
    """
    Returns the count of voxels labeled as true negative.
    """
    return arr[TN]

def arr_false_positive(arr: np.ndarray):
    """
    Returns the count of voxels labeled as false positive.
    """
    return arr[FP]

def arr_false_negative(arr: np.ndarray):
    """
    Returns the count of voxels labeled as false negative.
    """
    return arr[FN]



PLOT_OPTIONS = {
    "accuracy"          : (get_probability_figure, arr_accuracy),
    "precision"         : (get_probability_figure, arr_precision),
    "sensitivity"       : (get_probability_figure, arr_sensitivity),
    "specificity"       : (get_probability_figure, arr_specificity),
    "balanced-accuracy" : (get_probability_figure, arr_balanced_accuracy),
    "fall-out"          : (get_probability_figure, arr_fall_out),
    "miss-rate"         : (get_probability_figure, arr_miss_rate),

    "true-positive" : (get_quantity_figure, arr_true_positive),
    "true-negative" : (get_quantity_figure, arr_true_negative),
    "false-positive" : (get_quantity_figure, arr_false_positive),
    "false-negative" : (get_quantity_figure, arr_false_negative),
}



## ------------------------------ SCRIPT METHODS AND ENTRY POINT ------------------------------- ##


def get_metric_group(hdf5_path: pathlib.Path) -> h5py.Group:
    """
    Opens an HDF5 file and access the location of the Metic group.
    """
    h5_file  = h5py.File(hdf5_path, "r")
    return h5_file["Metric"]


def select_experiment(dir : pathlib.Path) -> pathlib.Path:
    """
    Selects one from the possible experiment directories.
    """
    experiments = [x for x in dir.iterdir() if x.is_dir()]
    if len(experiments) == 0:
        print("No experiment directories available")
    elif len(experiments) == 1:
        return experiments[0]

    print("Please select which experiment directory to use:")
    for i, experiment in enumerate(experiments):
        print(f"[{i}] {experiment.name}")
    idx = int(input("Enter experiment number: "))
    return experiments[idx]


def plot_policy_sweep(policy_path: pathlib.Path, figure_root: pathlib.Path, plot_key: str):
    """
    Generates the accuracy and precision plots. Combines the multiple policy view into one plot and
    if the policy was repeated then it plots the min/max and average with standard deviation bars.
    The generated figure is then saved.
    """

    confusion_groups = ["OccupancyConfusion_TSDF", "OccupancyConfusion_binary", "OccupancyConfusion_probability"]
    confusion_grid_labels = ["TSDF", "Space Carving", "Occupation Probability"]
    n_group = len(confusion_groups)

    part_name   = policy_path.parent.name.capitalize()
    policy_name = policy_path.name
    plots = PLOT_OPTIONS[plot_key][0](part_name, policy_name, plot_key, confusion_grid_labels)

    try:
        # Sort in increasing order on integer number.
        views_dirs = sorted([x for x in policy_path.iterdir() if x.is_dir()],
                            key=lambda x: int(str(x.name)), reverse=True)
    except ValueError:
        print("Could not turn directory name into integer for the number of views generated by the policy.")
        raise

    for views in views_dirs:
        line_label = f"{views.name} Views"

        views_plus_one = 1 + int(views.name)
        xdata = list(range(views_plus_one))

        reps_dirs = [x for x in views.iterdir() if x.is_dir()]
        data   = [ np.zeros((int(views.name), 4, len(reps_dirs))) ] * n_group
        result = [ np.zeros((views_plus_one, len(reps_dirs)))     ] * n_group

        for i, reps in enumerate(reps_dirs):
            hdf5_dir = reps / "results.h5"
            metric_group = get_metric_group(hdf5_dir)
            for g, group in enumerate(confusion_groups):
                confusion_group = metric_group[group]
                data[g][:, :, i] = confusion_group.get("data")[:, 1:5]
                result[g][1:, i] = np.apply_along_axis(PLOT_OPTIONS[plot_key][1], 1, data[g][:, :, i])

        if len(reps_dirs) > 1:
                line_label += " (Average)"
        for g in range(n_group):
            if len(reps_dirs) > 1:
                result_avg = result[g].mean(axis=1)
                result_std = result[g].std(axis=1)
                result_min = result[g].min(axis=1)
                result_max = result[g].max(axis=1)
                plots[g][1].errorbar(xdata, result_avg, result_std, linewidth=2, label=line_label, elinewidth=1, alpha=0.75, capsize=4.5)
                plots[g][1].fill_between(xdata, result_min, result_max, alpha=0.2)
                save_data = result_avg
            else:
                plots[g][1].plot(xdata, result[g][:, 0], linewidth=2, label=line_label)
                save_data = result

    for g in range(n_group):
        # plots[g][1].legend(loc='right')

        # plots[g][1].tick_params(axis='y', which='minor', bottom=False)
        # plots[g][1].yaxis.set_minor_locator(MultipleLocator(3))


        image_fpath = figure_root / policy_path.parent.name / confusion_grid_labels[g]
        # Split the old path to get the location of the figure.
        if image_fpath.exists() is False:
            image_fpath.mkdir(parents=True)

        image_fpath /= "_".join([policy_path.name, plot_key.capitalize(), "Results.jpeg"])

        plots[g][0].savefig(image_fpath)
        plt.close(plots[g][0])



def main(parsed_args: argparse.Namespace) -> None:
    """
    Program entry point.
    """
    assert parsed_args.plot in PLOT_OPTIONS.keys(), \
           f"The option {parsed_args.plot} is not a valid plotting option."

    results_dir = select_experiment(parsed_args.dir)
    figures_root = parsed_args.dir / 'Figures' / results_dir.name


    shape_dirs = [x for x in results_dir.iterdir() if x.is_dir()]
    for shape in shape_dirs:
        policy_dirs = [x for x in shape.iterdir() if x.is_dir()]
        for policy in policy_dirs:
            plot_policy_sweep(policy, figures_root, parsed_args.plot)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Plot Experiment Result Confusion',
        description='This script iterates through a set of experiment results (generated by the '
                    'sweep_run_experiment.py script) and creates condensed accuracy and precision '
                    'results for a policy reconstructing a specific shape.'
    )
    parser.add_argument(
        "-d", "--dir",
        type=pathlib.Path,
        required=False,
        default=PROJECT_ROOT_PATH / "share" / "Experiments" / 'Results',
        help="Where to save the generated data."
    )

    parser.add_argument(
        "-p", "--plot",
        type=str,
        required=True,
        help="What metric to plot. E.g., accuracy, precision."
    )

    args = parser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt as e:
        print("\nExiting early on KeyboardInterrupt.")
