#include "MapLoader.h"
#include <bits/stdc++.h>

using namespace std;


void MapLoader::loadMap(const string& filename) {
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
    }
    std::string line;
    while(infile>>line)
    {
        if(line=="height")    infile>>height;
        else if(line=="width")    infile>>width;
        else if(line=="map")   break;
    }
    mapData = std::vector<std::vector<int > >(height, std::vector<int>(width, 0));
    for(int i=0;i<height;i++)
    {
        infile>>line;
        for(int j=0;j<width;j++)
        {
            if(line[j]!='.') mapData[i][j] = -1;
        }
    }
}
int MapLoader::getHeight() const {
    return height;
}
int MapLoader::getWidth() const {
    return width;
}
int MapLoader::getMapData(int x, int y) const {
    return mapData[x][y];
}

void MapLoader::printMap() const {
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)    cout<<mapData[i][j]<<" ";
        cout<<endl;
    }
}

void MapLoader::setMapData(int x, int y, int value) {
    mapData[x][y] = value;
}

MapLoader ml;