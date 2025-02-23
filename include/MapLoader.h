#ifndef MapLoader_h
#define MapLoader_h

#include <bits/stdc++.h>

class MapLoader {
    public:
        void loadMap(const std::string& filename);
        void printMap() const;
        void setMapData(int x, int y, int value);
        int getHeight() const;
        int getWidth() const;
        int getMapData(int x, int y) const;
    private:
        int height, width;
        std::string filename;
        std::vector<std::vector<int > > mapData;
    };

extern MapLoader ml;

#endif