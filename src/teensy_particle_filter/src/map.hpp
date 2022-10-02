#ifndef MAP_H
#define MAP_H

#include <iostream> // cout, cerr
// #include <fstream> // ifstream
// #include <sstream> // stringstream

#include "../include/map.h"

class Map
{

private:
    int row, col, numrows, numcols;
    float ** map;

public:
    Map();

    int getColumns();
    int getRows();
    float** getMap();

};

#endif