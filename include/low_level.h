#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H
#include <bits/stdc++.h>
#include "MapLoader.h"
#include <map>
#include <unordered_map>
#include <functional>
#include <vector>
#include <cstddef> // 包含size_t定义
#include <opencv2/imgproc.hpp>


struct agent
{
    std::vector<int > st;
    std::vector<int > ed;
    void set(int stx,int sty,int edx,int edy);
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

struct Point3D {
    int x, y, t;
    Point3D(int x, int y, int t) : x(x), y(y), t(t) {}
    
    bool operator==(const Point3D& other) const {
        return x == other.x && y == other.y && t == other.t;
    }
};

struct Edge5D {
    int x1, y1, t1;
    int x2, y2;
    
    Edge5D(int x1, int y1, int t1, int x2, int y2)
        : x1(x1), y1(y1), t1(t1), x2(x2), y2(y2) {}
    
    bool operator==(const Edge5D& other) const {
        return x1 == other.x1 && y1 == other.y1 && t1 == other.t1 &&
               x2 == other.x2 && y2 == other.y2;
    }
};

// ================ 哈希特化 ================
namespace std {
    template<> struct hash<Point3D> {
        size_t operator()(const Point3D& p) const {
            const size_t prime1 = 73856093;
            const size_t prime2 = 19349663;
            const size_t prime3 = 83492791;
            return (p.x * prime1) ^ (p.y * prime2) ^ (p.t * prime3);
        }
    };

    template<> struct hash<Edge5D> {
        size_t operator()(const Edge5D& e) const {
            const size_t prime1 = 73856093;
            const size_t prime2 = 19349663;
            const size_t prime3 = 83492791;
            const size_t prime4 = 92756917;
            return (e.x1 * prime1) ^ (e.y1 * prime2) ^ 
                   (e.t1 * prime3) ^ (e.x2 * prime4) ^ 
                   (e.y2 * (prime1 ^ prime2));
        }
    };
}

extern int num_agent;
extern agent* as;

struct stage{
        std::vector<int > post;
        std::vector<int > parent;
        int agent;
        std::vector<int > tvalid_agent;
        int g,h;
        int res;
        int open = 0;
        void geth(std::vector<int > ed, double w = 1.0);
        //void ptf();
        std::pair<int,int> dir;
         // 计算方向变化惩罚
        double turn_penalty(const std::pair<int,int>& prev_dir) const {
            if(prev_dir == std::make_pair(0,0)) return 0; // 初始方向无惩罚
            int dot = dir.first*prev_dir.first + dir.second*prev_dir.second;  // 计算方向向量点积
            double angle = acos(dot / (norm(dir)*norm(prev_dir) + 1e-6));   // 计算角度惩罚（单位：弧度）
            if(angle > M_PI/2) {
                // 惩罚系数（可调整）
                const double PENALTY_WEIGHT = 15.0; 
                return PENALTY_WEIGHT * (cos(angle));
            }
            // else if(angle = M_PI/2) {
            //     // 惩罚系数（可调整）
            //     const double PENALTY_WEIGHT = 6.0; 
            //     return PENALTY_WEIGHT * (cos(angle));
            // }
            else {
                // 惩罚系数（可调整）
                const double PENALTY_WEIGHT = 3.0; 
                return PENALTY_WEIGHT * (cos(angle));
            }
            //return 0;
        }

    private:
        double norm(const std::pair<int,int>& d) const {
            return sqrt(d.first*d.first + d.second*d.second);
        }

};

struct Hashfunc {
    size_t operator() (const std::vector<int>& key) const;
};

struct Equalfunc {
    bool operator() (const std::vector<int>& a, const std::vector<int>& b) const;
};


struct stacmp{
    bool operator()(const std::vector<int>& a,const std::vector<int >& b) const;
};
//bool ifvalid(std::vector<int > stnow ,int dx ,int dy ,std::vector<std::vector<int> > ct_point3s ,std::vector<std::vector<int > > ct_edge6s);
// bool ifvalid(const std::vector<int>& stnow, int dx, int dy, 
//     const std::unordered_set<std::vector<int>, Hashfunc, Equalfunc>& ct_point3s_set, 
//     const std::vector<std::vector<int>>& ct_edge6s);

bool ifvalid(const std::vector<int>& stnow, int dx, int dy,
    const std::unordered_set<Point3D>& ct_point3s_set,
    const std::unordered_set<Edge5D>& ct_edge6s_set,
    const MapLoader& ml) ;

//void explore(std::vector<int > stnow,int dx,int dy,std::priority_queue<std::vector<int >,std::vector<std::vector<int>>, stacmp > &open_list,std::vector<int >& edstage,std::vector<int > ed0);
//void explore(std::vector<int> stnow, int dx, int dy,std::priority_queue<vector<int>, std::vector<vector<int>>, stacmp>& open_list,std::vector<int>& edstage, std::vector<int> ed0,std::unordered_map<std::pair<int, int>, std::map<int, double>, pair_hash>& dominance_map,int& pruned_nodes, int& total_nodes)
//int sta(agent* as,int i,std::vector<std::vector<int> > ct_point3s,std::vector<std::vector<int > > ct_edge6s,std::vector<std::vector<int > >& pths);

void explore(std::vector<int> stnow, int dx, int dy, int density_level,
    std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, stacmp>& open_list,
    std::vector<int>& edstage, std::vector<int> ed0, std::vector<int> st0);

int sta(agent* as, int i,
    std::vector<std::vector<int>> ct_point3s,
    std::vector<std::vector<int>> ct_edge6s,
    std::vector<std::vector<int>>& pths);
    //std::vector<std::vector<int>>& pths_smooth);


extern std::unordered_map<std::vector<int >,stage,Hashfunc,Equalfunc> hs;

std::vector<std::vector<bool>> generate_visited_from_hs(const std::unordered_map<std::vector<int>, stage, Hashfunc, Equalfunc>& hs, 
    int map_height, int map_width);

double calculatePathCost(const std::vector<std::vector<int>>& path);

void visualize_path(const MapLoader& ml, 
    const std::vector<std::vector<int>>& path,
    //const std::vector<std::vector<int>>& path_smooth,
    const agent& agent,
    const std::unordered_map<std::vector<int>, stage, Hashfunc, Equalfunc>& hs,
    int scale=30);

// class PathSmoother {
// public:
//     // Bézier曲线平滑
//     static std::vector<std::vector<int>> bezierSmooth(const std::vector<std::vector<int>>& path, 
//                                                      const MapLoader& ml,
//                                                      int iterations = 3,
//                                                      int sample_points = 20) {
//         if (path.size() < 3) return path; // 过短路径不处理

//         // 转换到OpenCV点格式
//         std::vector<cv::Point> points;
//         for (const auto& p : path) {
//             points.emplace_back(p[1], p[0]); // 转换为(x,y)即(col,row)
//         }

//         // 多次迭代平滑
//         for (int i = 0; i < iterations; ++i) {
//             points = computeControlPoints(points, ml);
//         }

//         // 生成Bézier曲线
//         std::vector<cv::Point> curve = generateBezier(points, sample_points);

//         // 转换回路径格式
//         std::vector<std::vector<int>> smoothed;
//         for (const auto& p : curve) {
//             if (isCollisionFree(p.y, p.x, ml)) { // y是row，x是col
//                 smoothed.push_back({p.y, p.x, 0}); // 时间维度暂置0
//             }
//         }
//         return smoothed;
//     }

// private:
//     // 碰撞检测
//     static bool isCollisionFree(int row, int col, const MapLoader& ml) {
//         if (row < 0 || row >= ml.getHeight() || col < 0 || col >= ml.getWidth()) 
//             return false;
//         return ml.getMapData(row, col) == 0;
//     }

//     // 生成控制点
//     static std::vector<cv::Point> computeControlPoints(const std::vector<cv::Point>& points,
//                                                       const MapLoader& ml) {
//         std::vector<cv::Point> control;
//         control.push_back(points.front());

//         // 自动选择关键转折点
//         for (size_t i = 1; i < points.size()-1; ++i) {
//             cv::Point prev = points[i-1];
//             cv::Point curr = points[i];
//             cv::Point next = points[i+1];
            
//             // 计算方向变化
//             double angle = std::abs(atan2(next.y - curr.y, next.x - curr.x) - 
//                                   atan2(curr.y - prev.y, curr.x - prev.x));
            
//             if (angle > CV_PI/6) { // 方向变化超过30度保留为控制点
//                 control.push_back(curr);
//             }
//         }
//         control.push_back(points.back());
//         return control;
//     }

//     // 生成Bézier曲线
//     static std::vector<cv::Point> generateBezier(const std::vector<cv::Point>& control,
//                                                 int samples) {
//         std::vector<cv::Point> curve;
//         const int n = control.size()-1;
        
//         for (int i = 0; i <= samples; ++i) {
//             double t = (double)i / samples;
//             cv::Point p(0, 0);
            
//             for (int j = 0; j <= n; ++j) {
//                 double blend = bernstein(n, j, t);
//                 p.x += control[j].x * blend;
//                 p.y += control[j].y * blend;
//             }
//             curve.push_back(p);
//         }
//         return curve;
//     }

//     // Bernstein基函数
//     static double bernstein(int n, int k, double t) {
//         return binomial(n, k) * pow(t, k) * pow(1-t, n-k);
//     }

//     // 二项式系数
//     static double binomial(int n, int k) {
//         if (k < 0 || k > n) return 0;
//         if (k == 0 || k == n) return 1;
//         return binomial(n-1, k-1) + binomial(n-1, k);
//     }
// };


#endif 