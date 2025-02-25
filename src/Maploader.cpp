#include "MapLoader.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

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

MapLoader ml;

// void MapLoader::computeDensity() {
//     const double kernel[3][3] = {{0.075, 0.124, 0.075},
//                                 {0.124, 0.204, 0.124},
//                                 {0.075, 0.124, 0.075}};

//     if (mapData.empty() || mapData[0].empty()) {
//         density_map.release();
//         return;
//     }
    
//     try {
//     // 维度验证
//         if (mapData.empty()) throw std::runtime_error("地图数据为空");
//         const int rows = mapData.size();
//         const int cols = mapData[0].size();
    
//     // 验证所有行长度一致
//         for (const auto& row : mapData) {
//             if (row.size() != cols) {
//                 throw std::runtime_error("地图数据行长度不一致");
//             }
//         }

//         if (rows < 3 || cols < 3) {
//             std::cerr << "警告：地图尺寸 " << rows << "x" << cols 
//                      << " 不足以计算密度" << std::endl;
//             return;
//         }
    
//     // 初始化矩阵并填充默认值
//         density_map.create(rows, cols, CV_64FC1);
//         density_map.setTo(0.0);  // 初始化所有元素为0

//     // 卷积计算
//         for(int i=1; i<rows-1; ++i) {
//         // 验证列边界
//             if (mapData[i].size() != cols) {
//                 throw std::runtime_error("第" + std::to_string(i) 
//                                    + "行列数不一致");
//             }
        
//             for(int j=1; j<cols-1; ++j) {
//                 double sum = 0;
//                 for(int dx=-1; dx<=1; ++dx) {
//                     for(int dy=-1; dy<=1; ++dy) {
//                     // 验证卷积核边界
//                         if (i+dx < 0 || i+dx >= rows || 
//                             j+dy < 0 || j+dy >= cols) {
//                             continue;  // 边界外设为0
//                         }
                    
//                         sum += kernel[dx+1][dy+1] * 
//                             (mapData[i+dx][j+dy] == -1 ? 1.0 : 0.0);
//                     }
//                 }
//                 density_map.at<double>(i, j) = sum;
//             }
//         }

//         // 边界初始化（应放在此处）// [!code focus]
//         for(int i=0; i<rows; ++i) {
//             density_map.at<double>(i, 0) = 0.0;
//             density_map.at<double>(i, cols-1) = 0.0;
//         }
//         for(int j=1; j<cols-1; ++j) {
//             density_map.at<double>(0, j) = 0.0;
//             density_map.at<double>(rows-1, j) = 0.0;
//         }
//     }
//     catch (const cv::Exception& e) {
//         std::cerr << "OpenCV异常: " << e.what() << std::endl;
//     }
//     catch (const std::exception& e) {
//         std::cerr << "计算密度时出错: " << e.what() << std::endl;
//     }
// }