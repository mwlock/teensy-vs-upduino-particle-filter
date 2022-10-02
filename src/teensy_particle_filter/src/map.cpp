#include "map.hpp"

using namespace std;

Map::Map(){
    /**
     * Read map from pgm file
     * https://stackoverflow.com/questions/8126815/how-to-read-in-data-from-a-pgm-file-in-c
     * https://stackoverflow.com/questions/8617683/return-a-2d-array-from-a-function
     *
     * @param file_name file name read map from
     * @return 2D array with grid cell map
     */

    // int row = 0, col = 0, numrows = 0, numcols = 0;
    // ifstream infile(file_name);
    // stringstream ss;
    // string inputLine = "";

    // // First line : version
    // getline(infile,inputLine);
    // if(inputLine.compare("P2") != 0) cerr << "Version error" << endl;
    // else cout << "Version : " << inputLine << endl;

    // // Second line : comment
    // getline(infile,inputLine);
    // cout << "Comment : " << inputLine << endl;

    // // Continue with a stringstream
    // ss << infile.rdbuf();
    // // Third line : size
    // ss >> numcols >> numrows;
    // cout << numcols << " columns and " << numrows << " rows" << endl;

    // // float array[numrows][numcols];
    // float** array = 0;
    // array = new float*[numrows];

    // // Following lines : data
    // for(row = 0; row < numrows; ++row){
    //     array[row] = new float[numcols];
    //     // for (col = 0; col < numcols; ++col) ss >> array[row][col];
    // }        
    
    // map = array;
}

int Map::getColumns(){
    return numcols;
}

int Map::getRows(){
    return numrows;
}

float** Map::getMap(){
    return map;
}