

#!/usr/bin/env python3

import os
import re
from pathlib import Path
from argparse import ArgumentParser
from this import d
# from this import d
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader

# Set style of plots to latex
plt.style.reload_library()
plt.style.use(['science', 'no-latex','high-vis'])
# plt.style.use(['science', 'ieee'])
plt.rcParams.update({'xtick.labelsize': 20,
                    'ytick.labelsize': 20,
                    'axes.titlesize': 20,
                    'axes.labelsize': 20,
                    'legend.fontsize': 20,
                    'legend.frameon' : True
                    })


# Added runtime arguments
def parse_args():
    parser = ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the ros bag containing the recording of the mission.")
    parser.add_argument("--rmse", action="store_true", help="Plot the root mean square error of the mission.")
    parser.add_argument("--update_time", action="store_true", help="Plot the update time during the mission.")
    return parser.parse_args()


# Parse in arguments
args = parse_args()


# Convert the sqlite databse to rosbag
abs_path = os.path.abspath(args.path)

MAP_WIDTH = 1.5
MAP_HEIGHT = 1.5

# Show trajectory plot
fig, ax = plt.subplots(figsize=(12, 12))
with AnyReader([Path(abs_path)]) as reader:
    # Get debug message
    df = get_dataframe(reader, "/particle_filter/config_string", ["data"])
    # Extract the NUMBER_OF_PARTICLES from the debug message
    num_particles = int(re.search(r'num_particles: (\d+)', df['data'].values[0]).group(1))
    # Get the data usage
    data_usage = re.search(r'memory used \(kb\): (\w+)', df['data'].values[0]).group(1)
    print("Number of particles: {}".format(num_particles))
    print("Data usage: {} kb".format(data_usage))

    # Get odom dataframe
    odom_dataframe = get_dataframe(reader, '/odom', ['pose.pose.position.x', 'pose.pose.position.y'])
    odom_dataframe = odom_dataframe.rename(columns={'pose.pose.position.x': 'odom_x', 'pose.pose.position.y': 'odom_y'})

    # Get ground truth dataframe
    ground_truth_dataframe = get_dataframe(reader, '/gps_imu_odom', ['pose.pose.position.x', 'pose.pose.position.y'])
    ground_truth_dataframe = ground_truth_dataframe.rename(columns={'pose.pose.position.x': 'gt_x', 'pose.pose.position.y': 'gt_y'})

    # Get estimated position
    estimated_position_dataframe = get_dataframe(reader, '/particle_filter/esimated_pose', ['position.x', 'position.y'])
    estimated_position_dataframe = estimated_position_dataframe.rename(columns={'position.x': 'estimated_x', 'position.y': 'estimated_y'})

    # Get update time dataframe
    update_time_dataframe = get_dataframe(reader, '/particle_filter/update_time', ['data'])
    update_time_dataframe = update_time_dataframe.rename(columns={'data': 'update_time'})

    # ALIGN ALL THE DATAFRAMES

    # Combine dataframes using pandas align
    dataframe_1, dataframe_2 = estimated_position_dataframe.align(ground_truth_dataframe, join='outer', axis=0, method='bfill')
    dataframe_3, dataframe_4 = dataframe_1.align(odom_dataframe, join='outer', axis=0, method='bfill')
    dataframe_5, dataframe_6 = dataframe_3.align(update_time_dataframe, join='outer', axis=0, method='bfill')
    # Merge the dataframes
    dataframe = pd.concat([dataframe_1, dataframe_2, dataframe_3, dataframe_4, dataframe_5, dataframe_6], axis=1)
    # Drop columns with NaN values
    dataframe = dataframe.dropna()
    # drop duplicated columns
    dataframe = dataframe.T.drop_duplicates().T
    # print(dataframe)
    # dataframe = pd.concat([odom_dataframe, ground_truth_dataframe,estimated_position_dataframe]).fillna(method='bfill').fillna(method='ffill').drop_duplicates()
    dataframe = dataframe[5:]

    # Create time axis
    time = dataframe.index.values
    time = time - time[0]
    time = time / np.timedelta64(1,'s')

    if time[-1] > 120:
        time_120_index = np.where(time > 120)[0]
        dataframe = dataframe[:time_120_index]
        time = time[:time_120_index]
    
    # Plot trajectory
    dataframe.plot(x='gt_x', y='gt_y', figure=fig, ax=ax, color='red', label='Ground Truth')
    dataframe.plot(x='estimated_x', y='estimated_y', figure=fig, ax=ax, color='blue', label='Estimated Pose')
    dataframe.plot(x='odom_x', y='odom_y', figure=fig, ax=ax, color='green', label='Odometry')

    ax.set_aspect('equal')
    plt.xlim([-MAP_WIDTH/2, MAP_WIDTH/2])
    plt.ylim([-MAP_HEIGHT/2, MAP_HEIGHT/2])
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # plt.show()

# Plot the mean squared error
if args.rmse:
    
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot distance error between ground truth and estimated position
    # dataframe['distance_error'] = np.sqrt((dataframe['gt_x'] - dataframe['estimated_x'])**2 + (dataframe['gt_y'] - dataframe['estimated_y'])**2)
    # dataframe.plot(use_index=True, y='distance_error', figure=fig, ax=ax, color='blue', label='Distance Error')

    odom_x = dataframe['odom_x'].values
    odom_y = dataframe['odom_y'].values
    gt_x = dataframe['gt_x'].values
    gt_y = dataframe['gt_y'].values
    estimated_x = dataframe['estimated_x'].values
    estimated_y = dataframe['estimated_y'].values

    # Calculate the distance error
    distance_error = np.sqrt((gt_x - estimated_x)**2 + (gt_y - estimated_y)**2)
    # Calculate the distance error
    odom_distance_error = np.sqrt((gt_x - odom_x)**2 + (gt_y - odom_y)**2)

    # Calculate the root mean square error
    rmse = np.sqrt(np.mean(distance_error**2))
    odom_rmse = np.sqrt(np.mean(odom_distance_error**2))

    # Print the rmse
    print('RMSE: ', rmse)
    print('Odometry RMSE: ', odom_rmse)

    # Calculate the 
    plt.plot(time,distance_error, color='blue', label='Esimated Pose')
    plt.plot(time,odom_distance_error, color='green', label='Odometry Distance Error')
    plt.xlabel('Time [s]')
    plt.ylabel('Error [m]')
    plt.grid(True)
    plt.legend(['Estimated Pose', 'Odometry'])

# Plot the update time
if args.update_time:

    fig, ax = plt.subplots(figsize=(12, 6))

    # Extract the update time per particle
    update_time = dataframe['update_time'].values / num_particles

    # Plot the update time
    plt.plot(time,update_time, color='blue', label='Update Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Update Time [ms]')
    plt.grid(True)
    # plt.legend(['Update Time'])

    fig, ax = plt.subplots(figsize=(12, 6))

    # Extract the update time per particle
    update_time = dataframe['update_time'].values

    # Plot the update time
    plt.plot(time,update_time, color='blue', label='Update Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Update Time [ms]')
    plt.grid(True)

plt.show()
