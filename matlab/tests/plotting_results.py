'''This script analyzes and plots the results from the experiments.'''
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

plot = False
save_figs = True

# ordered_strats = ['safety', 'bvc', 'tube', 'on_dem', 'perf']
ordered_strats = ['safety', 'bvc', 'on_dem', 'perf']
# strat_colors = ['cornflowerblue', 'goldenrod', 'pink', 'tomato', 'limegreen']
strat_colors = ['cornflowerblue', 'goldenrod', 'tomato', 'limegreen']


def load_all_algos():
    '''Loads all the results.

    Returns:
        all_results (dict): A dictionary containing all the results.
    '''

    all_results = {}

    for strat in ordered_strats:
        all_results[strat] = {}
        for c_comm in [2, 4, 6, 8]:
            all_results[strat][c_comm] = {}
            for fail in ['sys', 'stoc']:
                data = loadmat(f'./results/data_{strat}_{fail}_c0{c_comm}.mat')
                all_results[strat][c_comm][fail] = data

    data = loadmat('./results/data_c00.mat')
    all_results['no_failure'] = data

    data = loadmat('./results/data_c10.mat')
    all_results['no_comms'] = data

    return all_results


def create_plot(metric, fail_type):
    '''Plots the results.'''
    all_results = load_all_algos()

    fig = plt.figure(figsize=(8.0, 5.0))
    ax = fig.add_subplot(111)

    data = defaultdict(list)

    # labels = ['Safety', 'BVC', 'Field', 'On-Demand', 'Performance']
    labels = ['Safety', 'BVC', 'On-Demand', 'Performance']

    for strat in ordered_strats:
        for c_comm in [2,4,6,8]:
            if metric == 'tEnd':
                raw_data = all_results[strat][c_comm][fail_type][metric][0,0]/np.sum(all_results[strat][c_comm][fail_type]['all_lengths'])
            else:
                raw_data = np.mean(all_results[strat][c_comm][fail_type][metric])
            data[strat].append(raw_data)

    if metric == 'tEnd':
        raw_data = all_results['no_failure'][metric][0,0]/np.sum(all_results['no_failure']['all_lengths'])
    else:
        raw_data = np.mean(all_results['no_failure'][metric])
    data['no_failure'].append(raw_data)

    if metric == 'tEnd':
        raw_data = all_results['no_comms'][metric][0,0]/np.sum(all_results['no_comms']['all_lengths'])
    else:
        raw_data = np.mean(all_results['no_comms'][metric])
    data['no_comms'].append(raw_data)

    num_bars = 4
    width = 1/(num_bars+2)
    x = np.arange(1, num_bars+1)

    for idx, strat_name in enumerate(ordered_strats):
        position = ((num_bars-1)/2.0 - idx)*width
        ax.bar(x=x - position, height=data[strat_name], width=width, color=strat_colors[idx], label=labels[idx])

    ax.bar(x=1 - ((num_bars-1)/2.0)*width - width*3, height=data['no_failure'], width=width, color='limegreen')
    ax.bar(x=num_bars - ((num_bars-1)/2.0 - len(ordered_strats)-1)*width + width, height=data['no_comms'], width=width, color='limegreen')

    if metric == 'all_lengths':
        metric_name = 'Mission Iterations'
    elif metric == 'all_violations':
        metric_name = 'Collisions'
    else:
        metric_name = 'Iteration Runtime (s)'

    ax.set_ylabel(f'Mean {metric_name}', fontsize=20, labelpad=10)

    new_x = [1 - ((num_bars-1)/2.0)*width - width*3] + list(x) + [num_bars - ((num_bars-1)/2.0 - len(ordered_strats)-1)*width + width]
    ax.set_xticks(new_x, ['c=0', 'c=0.2', 'c=0.4', 'c=0.6', 'c=0.8', 'c=1'],  fontsize=20)

    fig.tight_layout()

    ax.set_ylim(ymin=0)
    ax.yaxis.grid(True)
    ax.legend(loc='upper left', fontsize=15)
    if plot is True:
        plt.show()

    if save_figs:
        fig.savefig(f'./plots/{metric}_{fail_type}.png', dpi=300)


if __name__ == '__main__':
    create_plot('all_lengths', 'sys')
    create_plot('all_lengths', 'stoc')
    create_plot('all_violations', 'sys')
    create_plot('all_violations', 'stoc')
    create_plot('tEnd', 'sys')
    create_plot('tEnd', 'stoc')
