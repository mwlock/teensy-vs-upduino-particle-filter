import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
from PIL import Image as im

class WoodenBox:
    BOX_WIDTH = 0.1
    BOX_HEIGHT = 0.1
    def __init__(self, x, y) -> None:
        self.x = x - WoodenBox.BOX_WIDTH/2
        self.y = y - WoodenBox.BOX_HEIGHT/2
        self.width = WoodenBox.BOX_WIDTH
        self.height = WoodenBox.BOX_HEIGHT

class Map():

    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.boxes = []
        self.grid = []

    def add_box(self, box):

        self.boxes.append(box)

    def generate_map_grid(self):

        # Populate the map grid with 0s
        grid = [[0 for x in range(int(-int(self.width/self.resolution)/2), int(int(self.width/self.resolution)/2))] for y in range(int(-int(self.height/self.resolution)/2), int(int(self.height/self.resolution)/2))]

        # Populate the map grid with 1s for each and ensure all boxes are the same size    

        # TODO : FIX THE FUCKING BOXES (no squooshing)

        for box in self.boxes:
            # print("Box: x: {}, y: {}".format(box.x, box.y))
            for x in range(int(round(box.x + self.width/2,5)/self.resolution) , int(round(box.x + box.width + self.width/2,5)/self.resolution)):
                for y in range(int(round(box.y + self.height/2,5)/self.resolution) ,int(round(box.y + box.height + self.height/2,5)/self.resolution)):
                    grid[y][x] = 1

        for box in self.boxes:
            # print("Box: x: {}, y: {}".format(box.x, box.y))
            for x in range(int(round(box.x + self.width/2,5)/self.resolution) +1, int(round(box.x + box.width + self.width/2,5)/self.resolution)-1):
                for y in range(int(round(box.y + self.height/2,5)/self.resolution) + 1 ,int(round(box.y + box.height + self.height/2,5)/self.resolution) - 1):
                    grid[y][x] = 0

        # # Remove filling boxes
        # for box in self.boxes:
        #     for x in range(int((box.x + self.width/2)/self.resolution) +1, int((box.x + box.width + self.width/2)/self.resolution)-1):
        #         for y in range(int((box.y + self.height/2)/self.resolution)+1, int((box.y + box.height + self.height/2)/self.resolution)-1):
        #             grid[y][x] = 0
                    
        
        # # Outline
        # for box in self.boxes:
        #     for x in range(int((box.x + self.width/2)/self.resolution) , int((box.x + box.width + self.width/2)/self.resolution)):
        #         grid[int((box.y + self.height/2)/self.resolution)][x] = 1
        #         grid[int((box.y + box.height + self.height/2)/self.resolution) - 1][x] = 1

        #     for y in range(int((box.y + self.height/2)/self.resolution), int((box.y + box.height + self.height/2)/self.resolution)):
        #         grid[y][int((box.x + self.width/2)/self.resolution)] = 1
        #         grid[y][int((box.x + box.width + self.width/2)/self.resolution) - 1] = 1
        # grid[int((box.y + box.height + self.height/2)/self.resolution)][int((box.x + box.width + self.width/2)/self.resolution)] = 1
            

        # Draw walls
        for x in range(0, int(self.width/self.resolution)):
            grid[0][x] = 1
            grid[int(self.height/self.resolution)-1][x] = 1
        for y in range(0, int(self.height/self.resolution)):
            grid[y][0] = 1
            grid[y][int(self.width/self.resolution)-1] = 1

        self.grid = np.array(grid)
        self.grid = np.rot90(grid, 3)

        # Flip the map vertically
        self.grid = np.flip(self.grid, 0)

        # Flip the map horizontally
        self.grid = np.flip(self.grid, 1)

        # return the map grid clockwise
        return self.grid

    def save_map_to_file(self):

        array_string = "{"
        for col in self.grid:
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

        array_string = "const bool map_array[{}][{}] = {}".format(self.grid.shape[0],self.grid.shape[1],array_string)
        map_string = '#ifndef MAP_H\n#define MAP_H\n{}\n#endif\n'.format(array_string)    

        text_file = open('../include/map_array.h', "w")
        text_file.write(map_string)
        text_file.close()

    def write_pgm(self):
        """ Write grayscale image in PGM format to file.
        """
        # open file for writing 
        filename = '../resources/epuck_world_map.pgm'
        fout=open(filename, 'wb')

        width = self.grid.shape[0]
        height = self.grid.shape[1]

        # define PGM Header
        pgmHeader = 'P5\n' + str(width) + ' ' + str(height) + '\n' + str(255) +  '\n'

        pgmHeader_byte = bytearray(pgmHeader,'utf-8')

        # write the header to the file
        fout.write(pgmHeader_byte)

        # write the data to the file 
        img = np.reshape((1-self.grid)*255,(height,width))

        for j in range(height):
            bnd = list(img[j,:])
            fout.write(bytearray(bnd)) # for 8-bit data only

        fout.close()

    
    def save_obstacle_array(self):

        # Save x and y indexes of of grid cells that are obstacles to two different c arrays

        obstacle_x = []
        obstacle_y = []

        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                if self.grid[y][x]:
                    obstacle_x.append(x)
                    obstacle_y.append(y)

        array_string = "{"
        for data in obstacle_x:
            array_string += str(data) + ','
        array_string = array_string[:-1] + '};'

        array_string = "const int obstacle_x[{}] = {}".format(len(obstacle_x),array_string)
        map_string = '#ifndef OBSTACLE_X_H\n#define OBSTACLE_X_H\n{}\n#endif\n'.format(array_string)

        text_file = open('../include/obstacle_x.h', "w")
        text_file.write(map_string)
        text_file.close()

        array_string = "{"
        for data in obstacle_y:
            array_string += str(data) + ','
        array_string = array_string[:-1] + '};'

        array_string = "const int obstacle_y[{}] = {}".format(len(obstacle_y),array_string)
        map_string = '#ifndef OBSTACLE_Y_H\n#define OBSTACLE_Y_H\n{}\n#endif\n'.format(array_string)

        text_file = open('../include/obstacle_y.h', "w")
        text_file.write(map_string)
        text_file.close()



WIDTH = 1.5
HEIGHT = 1.5
RESOLUTION = 0.01

map = Map(WIDTH, HEIGHT, RESOLUTION)

print("Generating map...")
map.add_box(WoodenBox(-0.265062,0.13))
map.add_box(WoodenBox(-0.158467,0.26))
map.add_box(WoodenBox(-0.115895,-0.36))
map.add_box(WoodenBox(0.29726,-0.29))
map.add_box(WoodenBox(0.44,0.12))
grid = map.generate_map_grid()

# Print number of columns and rows
print("Number of columns: {}".format(len(grid[0])))
print("Number of rows: {}".format(len(grid)))

# Save map to file
map.save_map_to_file()
map.write_pgm()
map.save_obstacle_array()

plt.figure()
plt.imshow(np.array(grid))
plt.show()
