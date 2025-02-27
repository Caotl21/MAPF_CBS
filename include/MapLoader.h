#ifndef MapLoader_h
#define MapLoader_h

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <fstream> 
#include <iomanip>

class MapLoader {
    public:
        void loadMap(const std::string& filename);
        void printMap() const;
        void setMapData(int x, int y, int value);
        int getHeight() const;
        int getWidth() const;
        int getMapData(int x, int y) const;
        const cv::Mat& getDensityMap() const;
        void computeDensity();
        double getDensity(int x, int y) const;
        void saveDensity2File(const std::string& filename) const;
    private:
        int height, width;
        std::string filename;
        std::vector<std::vector<int > > mapData;
        cv::Mat density_map; // 障碍物密度图 直接使用OpenCV矩阵
    };

extern MapLoader ml;

#endif