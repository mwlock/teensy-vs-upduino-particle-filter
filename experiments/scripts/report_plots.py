

#!/usr/bin/env python3

import os
import re
from pathlib import Path
from argparse import ArgumentParser
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader

# Set style of plots to latex
plt.style.reload_library()
plt.style.use(['science', 'no-latex','high-vis'])
# plt.style.use(['science', 'ieee'])

text_size = 23
plt.rcParams.update({'xtick.labelsize': text_size,
                    'ytick.labelsize': text_size,
                    'axes.titlesize': text_size,
                    'axes.labelsize': text_size,
                    'legend.fontsize': text_size,
                    'legend.frameon' : True
                    })

# Save all graphs in "plots" folder
def save_plot(plot,fig_name):

    if not args.pdf:
        return

    if not os.path.exists("plots"):
        os.makedirs("plots")
    plot.savefig("plots/{}.pdf".format(fig_name), format="pdf")


# Added runtime arguments
def parse_args():
    parser = ArgumentParser()
    parser.add_argument("path1", type=str, help="Path to the ros bag containing the recording of the experiment 1.")
    parser.add_argument("path2", type=str, help="Path to the ros bag containing the recording of the experiment 2.")
    parser.add_argument("path3", type=str, help="Path to the ros bag containing the recording of the experiment 3.")
    parser.add_argument("--pdf", action="store_true", help="Save the plots as pdfs.")
    return parser.parse_args()

# Parse in arguments
args = parse_args()

names = [
    'Experiment_1',
    'Experiment_2',
    'Experiment_3'
]

files = [
    args.path1,
    args.path2,
    args.path3
]

num_particles_list = []
memory_usage_list = []

# Create empty dataframe
dataframe = pd.DataFrame()

for i in range(1, 3+1):

    with AnyReader([Path(files[i-1])]) as reader:

        df = get_dataframe(reader, "/particle_filter/config_string", ["data"])
        num_particles = int(re.search(r'num_particles: (\d+)', df['data'].values[0]).group(1))
        data_usage = re.search(r'memory used \(kb\): (\w+)', df['data'].values[0]).group(1)
        num_particles_list.append(num_particles)
        memory_usage_list.append(float(data_usage))

        # Get odom dataframe
        odom_dataframe = get_dataframe(reader, '/odom', ['pose.pose.position.x', 'pose.pose.position.y'])
        odom_dataframe = odom_dataframe.rename(columns={'pose.pose.position.x': f'odom_x_{i}', 'pose.pose.position.y': f'odom_y_{i}'})
        odom_dataframe['timedeltas'] = odom_dataframe.index - odom_dataframe.index[0]
        odom_dataframe.set_index('timedeltas', inplace=True)

        # Get ground truth dataframe
        ground_truth_dataframe = get_dataframe(reader, '/gps_imu_odom', ['pose.pose.position.x', 'pose.pose.position.y'])
        ground_truth_dataframe = ground_truth_dataframe.rename(columns={'pose.pose.position.x': f'gt_x_{i}', 'pose.pose.position.y': f'gt_y_{i}'})
        ground_truth_dataframe['timedeltas'] = ground_truth_dataframe.index - ground_truth_dataframe.index[0]
        ground_truth_dataframe.set_index('timedeltas', inplace=True)

        # Get estimated position
        estimated_position_dataframe = get_dataframe(reader, '/particle_filter/esimated_pose', ['position.x', 'position.y'])
        estimated_position_dataframe = estimated_position_dataframe.rename(columns={'position.x': f'estimated_x_{i}', 'position.y': f'estimated_y_{i}'})
        estimated_position_dataframe['timedeltas'] = estimated_position_dataframe.index - estimated_position_dataframe.index[0]
        estimated_position_dataframe.set_index('timedeltas', inplace=True)

        # Get update time dataframe
        update_time_dataframe = get_dataframe(reader, '/particle_filter/update_time', ['data'])
        update_time_dataframe = update_time_dataframe.rename(columns={'data': f'update_time_{i}'})
        update_time_dataframe['timedeltas'] = update_time_dataframe.index - update_time_dataframe.index[0]
        update_time_dataframe.set_index('timedeltas', inplace=True)

        # Combine dataframes using pandas align
        dataframe_concat_1, dataframe_concat_2 = dataframe.align(estimated_position_dataframe, join='outer', axis=0, method='bfill')
        dataframe_1, dataframe_2 = dataframe_concat_1.align(ground_truth_dataframe, join='outer', axis=0, method='bfill')
        dataframe_3, dataframe_4 = dataframe_1.align(odom_dataframe, join='outer', axis=0, method='bfill')
        dataframe_5, dataframe_6 = dataframe_3.align(update_time_dataframe, join='outer', axis=0, method='bfill')
        # Merge the dataframes
        dataframe = pd.concat([dataframe_concat_1, dataframe_concat_2, dataframe_1, dataframe_2, dataframe_3, dataframe_4, dataframe_5, dataframe_6], axis=1)
        # Drop columns with NaN values
        dataframe = dataframe.dropna()
        # drop duplicated columns
        dataframe = dataframe.T.drop_duplicates().T
        # print(dataframe)
        # dataframe = pd.concat([odom_dataframe, ground_truth_dataframe,estimated_position_dataframe]).fillna(method='bfill').fillna(method='ffill').drop_duplicates()
        # dataframe = dataframe[5:]

# Concert to seconds
time = dataframe.index.values
time = time / np.timedelta64(1, 's')

# for i,a in enumerate(dataframe.index.values):
#     time[i] = a.total_seconds()

if time[-1] > 120:
    time_120_index = np.where(time > 120)[0][0]
    dataframe = dataframe[:time_120_index]
    time = time[:time_120_index]

# Plot update time for each experiment on the same plot
fig, ax = plt.subplots(figsize=(12, 6))

update_time_1 = dataframe['update_time_1'].values
update_time_2 = dataframe['update_time_2'].values
update_time_3 = dataframe['update_time_3'].values

ax.plot(time, update_time_1, label='Experiment 1')
ax.plot(time, update_time_2, label='Experiment 2')
ax.plot(time, update_time_3, label='Experiment 3')

ax.set_xlabel('Time [s]')
ax.set_ylabel('Update time [ms]')
ax.set_title('Update time for each experiment')
ax.grid(True)

# Calculate standard deviation for each experiment
std_list = [np.std(update_time_1), np.std(update_time_2), np.std(update_time_3)]

# Calculate mean for each experiment
mean_list = [np.mean(update_time_1), np.mean(update_time_2), np.mean(update_time_3)]

# Add legend for number of particles and standard deviation for each experiment
legend_list = []
for i in range(len(num_particles_list)):
    legend_list.append(f'{num_particles_list[i]} particles : mean:{mean_list[i]:.2f}, std: {std_list[i]:.2f}')

ax.legend(legend_list, loc='upper right')
save_plot(fig, 'update_time')

# Point wise division of update time

mean_update_time_1 = np.mean(update_time_1)
mean_update_time_2 = np.mean(update_time_2)
mean_update_time_3 = np.mean(update_time_3)

slowdown_1 = mean_update_time_1 / mean_update_time_1
slowdown_2 = mean_update_time_2 / mean_update_time_1
slowdown_3 = mean_update_time_3 / mean_update_time_1

# Print mean update time for each experiment
print(f'Mean slowdown time for experiment 1: {slowdown_1:.2f}')
print(f'Mean slowdown time for experiment 2: {slowdown_2:.2f}')
print(f'Mean slowdown time for experiment 3: {slowdown_3:.2f}')

# Iterate and plot the distance error between the estimated position and the ground truth, and the odom position and the ground truth
for i in range(1, len(num_particles_list) + 1):

    # Get estimated position
    estimated_position_x = dataframe[f'estimated_x_{i}'].values
    estimated_position_y = dataframe[f'estimated_y_{i}'].values
    # Get ground truth position
    ground_truth_x = dataframe[f'gt_x_{i}'].values
    ground_truth_y = dataframe[f'gt_y_{i}'].values
    # Get odom position
    odom_x = dataframe[f'odom_x_{i}'].values
    odom_y = dataframe[f'odom_y_{i}'].values
    # Calculate the distance error
    estimated_distance_error = np.sqrt((estimated_position_x - ground_truth_x)**2 + (estimated_position_y - ground_truth_y)**2)
    odom_distance_error = np.sqrt((odom_x - ground_truth_x)**2 + (odom_y - ground_truth_y)**2)

    # Plot the distance error
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(time, estimated_distance_error, label='Estimated distance error')
    ax.plot(time, odom_distance_error, label='Odom distance error')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [m]')
    ax.set_title(f'Error for {num_particles_list[i-1]} particles')
    ax.grid(True)

    # Get RMSE for each experiment
    rmse_estimated = np.sqrt(np.mean(estimated_distance_error**2))
    rmse_odom = np.sqrt(np.mean(odom_distance_error**2))

    # Place root mean square error in the legend to three decimals
    ax.legend([f'Particle Filter, RMSE: {rmse_estimated:.3f} m', f'Odometry, RMSE: {rmse_odom:.3f} m'],loc='upper left')


    save_plot(fig, f'distance_error_{num_particles_list[i-1]}')

# Iterate through the experiments and plot the estimated position, ground truth position, and odom position
for i in range(1, len(num_particles_list) + 1):

    # Get estimated position
    estimated_position_x = dataframe[f'estimated_x_{i}'].values
    estimated_position_y = dataframe[f'estimated_y_{i}'].values
    # Get ground truth position
    ground_truth_x = dataframe[f'gt_x_{i}'].values
    ground_truth_y = dataframe[f'gt_y_{i}'].values
    # Get odom position
    odom_x = dataframe[f'odom_x_{i}'].values
    odom_y = dataframe[f'odom_y_{i}'].values

    # Plot the estimated position, ground truth position, and odom position
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.plot(estimated_position_x, estimated_position_y, label='Estimated position')
    ax.plot(ground_truth_x, ground_truth_y, label='Ground truth position')
    ax.plot(odom_x, odom_y, label='Odom position')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'Estimated, ground truth, and odom position for {num_particles_list[i-1]} particles')
    ax.grid(True)

    # Set equal aspect ratio
    ax.set_aspect('equal', 'box')

    # Set x and y limits to [-0.75, 0.75]
    ax.set_xlim(-0.75, 0.75)
    ax.set_ylim(-0.75, 0.75)

    # Place root mean square error in the legend to three decimals
    ax.legend()
    
    save_plot(fig, f'position_{num_particles_list[i-1]}')


# Plot memory usage for each experiment on bar chart from the memory_usage_list
fig, ax = plt.subplots(figsize=(12, 6))

BASE_MEMORY_USAGE = 686

# Minus BASE_MEMORY_USAGE from each memory usage
memory_usage_list = [BASE_MEMORY_USAGE - memory_usage for memory_usage in memory_usage_list]

# Print the memory usage for each experiment
for i, v in enumerate(memory_usage_list):
    print(f'Experiment {i+1} memory usage: {v} MB')

ax.set_xlabel('Number of particles')

ax.set_ylabel('Memory usage [kB]')
ax.set_title('Memory usage for each experiment')
ax.grid(True)

# Make the bars thicker
# list num_particles_list to list of strings
ax.bar([str(x) for x in num_particles_list], memory_usage_list, width=0.5)

# Place value label on top of each bar
for i, v in enumerate(memory_usage_list):
    ax.text(i, v + 0.5, str(int(v)), horizontalalignment='center')

save_plot(fig, 'memory_usage')

plt.show() 