#include "MapLoader.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <fstream>  
#include <iomanip>

using namespace std;


void MapLoader::loadMap(const string& filename) {
    std::ifstream infile(filename);
    if(!infile.is_open()) { // [!code ++]
        throw std::runtime_error("无法打开地图文件: " + filename); // [!code ++]
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
    computeDensity();
    saveDensity2File("density.txt");

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

double MapLoader::getDensity(int x, int y) const {
    return density_map.at<double>(y,x);            //OpenCV是(row,col)即(y,x)
}

void MapLoader::computeDensity() {
    cv::Mat obstacle_map(mapData.size(), mapData[0].size(), CV_64FC1);
    
    // 并行初始化障碍物图
    cv::parallel_for_(cv::Range(0, obstacle_map.rows), [&](const cv::Range& range){
        for (int i = range.start; i < range.end; ++i) {
            for (int j = 0; j < obstacle_map.cols; ++j) {
                obstacle_map.at<double>(i, j) = 
                    (mapData[i][j] == -1) ? 1.0 : 0.0;
            }
        }
    });

    // 预分配输出矩阵
    density_map.create(obstacle_map.size(), obstacle_map.type());
    // 使用OpenCV滤波代替手动卷积
    cv::Mat kernel = (cv::Mat_<double>(3,3) << 
        0.075, 0.124, 0.075,
        0.124, 0.204, 0.124,
        0.075, 0.124, 0.075);
    cv::filter2D(obstacle_map, density_map, -1, kernel,
                 cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
}

//DensityMap interface
const cv::Mat& MapLoader::getDensityMap() const {  // [!code focus]
    assert(!density_map.empty() && "Density map not initialized!");
    return density_map;
}

void MapLoader::saveDensity2File(const std::string& filename) const{
    if (density_map.empty()) {
        throw std::runtime_error("Density map is empty. Call computeDensity() first.");
    }
    
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    
    outFile << std::fixed << std::setprecision(3); // 设置小数点后四位
    
    for (int i = 0; i < density_map.rows; ++i) {
        for (int j = 0; j < density_map.cols; ++j) {
            outFile << density_map.at<double>(i, j);
            if (j < density_map.cols - 1) {
                outFile << " "; // 用空格分隔数值
            }
        }
        outFile << "\n"; // 每行末尾换行
    }
    
    outFile.close();
}

MapLoader ml;

