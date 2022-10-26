#!/usr/bin/env python3

import os
import re
from pathlib import Path
from argparse import ArgumentParser
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.io

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
    parser.add_argument("acc_path1", type=str, help="Path to the ros bag containing the recording of accelerator experiment 1.")
    parser.add_argument("acc_path2", type=str, help="Path to the ros bag containing the recording of accelerator experiment 2.")
    parser.add_argument("acc_path3", type=str, help="Path to the ros bag containing the recording of accelerator experiment 3.")
    parser.add_argument("--pdf", action="store_true", help="Save the plots as pdfs.")
    return parser.parse_args()

# Parse in arguments
args = parse_args()

names = [
    'Experiment_1',
    'Experiment_2',
    'Experiment_3'
]

acc_names = [
    'Accelerator_1',
    'Accelerator_2',
    'Accelerator_3'
]

files = [
    args.path1,
    args.path2,
    args.path3
]

acc_files = [
    args.acc_path1,
    args.acc_path2,
    args.acc_path3
]

mean_teensy_power = []
mean_accel_power = []

# Loop through files and load matlab mat files
print('Teensy')
for name, file in zip(names, files):

    data = scipy.io.loadmat(file)
    power_supply = data['x_1']
    voltage_drop = data['x_3'] * 1000
    current = voltage_drop/ 9 # I = V / R
    power = np.mean(power_supply) * current
    time = data['t']

    voltage_drop = voltage_drop.flatten()
    time = time.flatten()

    power = abs(np.mean(power))
    mean_teensy_power.append(power*1000)

    # print
    print('\n')
    print("Mean voltage drop of {} is {} mV".format(name, np.mean(voltage_drop)))
    print("Mean current of {} is {} A".format(name, np.mean(current)))
    print("Mean power of {} is {} W".format(name,power))

print('Teensy with accel')
for name, file in zip(acc_names, acc_files):

    data = scipy.io.loadmat(file)
    power_supply = data['x_1']
    voltage_drop = data['x_3'] * 1000
    current = voltage_drop/ 9 # I = V / R
    power = np.mean(power_supply) * current
    time = data['t']

    voltage_drop = voltage_drop.flatten()
    time = time.flatten()

    power = abs(np.mean(power))
    mean_accel_power.append(power*1000)

    # print
    print('\n')
    print("Mean voltage drop of {} is {} mV".format(name, np.mean(voltage_drop)))
    print("Mean current of {} is {} A".format(name, np.mean(current)))
    print("Mean power of {} is {} W".format(name,power))


  
n=len(mean_teensy_power)
r = np.arange(n)
width = 0.25
  
fig, ax = plt.subplots(figsize=(12, 6))
plt.grid(linestyle='--')
plt.bar(r, mean_teensy_power, color = 'b',
        width = width, edgecolor = 'black',
        label='Teensy')
plt.bar(r + width, mean_accel_power, color = 'g',
        width = width, edgecolor = 'black',
        label='Teensy + UPduino v3.1')
  
plt.xlabel("Number of particles")
plt.ylabel("Power [mW]")
plt.title("Power draw of the Teensy and Teensy + UPduino v3.1")

# Place value labels on top of bars
for i in range(len(mean_teensy_power)):
    plt.text(x = r[i]-0.05 , y = mean_teensy_power[i]+1, s = str(int(mean_teensy_power[i])), size = 15)
    plt.text(x = r[i]+0.2 , y = mean_accel_power[i]+1, s = str(int(mean_accel_power[i])), size = 15)

  
# plt.grid(linestyle='--')
plt.xticks(r + width/2,['128','256','512'])
plt.legend()
ax.set_axisbelow(True)

save_plot(fig, 'power_usage')


plt.show()