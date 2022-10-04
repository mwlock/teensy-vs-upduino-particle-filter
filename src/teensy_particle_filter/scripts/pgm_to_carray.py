import numpy as np

import pick
from glob import glob

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

OCCUPIED_THRESHOLD = 0.65

# Get all maps
files = list(glob('../resources/*.pgm'))
choice = pick.pick(files)[0]

with open(choice, 'rb') as pgmf:

    # Read file
    im = plt.imread(pgmf)
    im_numpy = np.array(im)
    im_numpy = 1- (im_numpy/255) >= OCCUPIED_THRESHOLD

    # Create subplot
    fig, (ax1, ax2) = plt.subplots(1, 2)
    fig.suptitle('Occupancy grid')

    # Plot original map
    im_1 = ax1.imshow(im)
    ax1.set_title('Original map')

    # Plot binary occupancy map
    im_2 = ax2.imshow(im_numpy)
    ax2.set_title('Binary occupancy grid')

    # make color bar(im1)
    divider = make_axes_locatable(ax1)
    cax = divider.append_axes('right', size='5%', pad=0.05)
    fig.colorbar(im_1, cax=cax, orientation='vertical')

    # make color bar(im2)
    divider = make_axes_locatable(ax2)
    cax = divider.append_axes('right', size='5%', pad=0.05)
    fig.colorbar(im_2, cax=cax, orientation='vertical')

    plt.show()

    # Write string to file to store array for header
    # array_string = "{{ {} }}".format([ (str(row)+'\n') for row in im_numpy])
    array_string = "{"
    for col in im_numpy:
        temp_string = "{"
        for data in col:
            if data:
                temp_string += "true"
            else:
                temp_string += "false"
            temp_string += ","
        temp_string = temp_string[:-1] +'},\n'
        array_string +=temp_string
    array_string = array_string[:-2] + '};'

    array_string = "bool map[{}][{}] = {}".format(im_numpy.shape[0],im_numpy.shape[1],array_string)
    map_string = '#ifndef MAP_H\n#define MAP_H\n{}\n#endif\n'.format(array_string)    

    text_file = open('../include/map_array.h', "w")
    text_file.write(map_string)
    text_file.close()