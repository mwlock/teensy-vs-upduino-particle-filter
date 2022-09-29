#ifndef MAP_H
#define MAP_H

#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream

class Map
{

private:
    int row, col, numrows, numcols;
    float ** map;

public:
    Map(std::string file_name);

    int getColumns();
    int getRows();
    float** getMap();

};

#endif