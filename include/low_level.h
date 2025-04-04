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
        double g,h;
        double res;
        int open = 0;
        void geth(std::vector<int > ed, double w = 1.0);
        //void ptf();
        std::pair<int,int> dir;
         // 计算方向变化惩罚
        double turn_penalty(const std::pair<int,int>& prev_dir) const {
            if(prev_dir == std::make_pair(0,0)) return 0; // 初始方向无惩罚
            int dot = dir.first*prev_dir.first + dir.second*prev_dir.second;  // 计算方向向量点积
            double angle = acos(dot / (norm(dir)*norm(prev_dir) + 1e-6));   // 计算角度惩罚（单位：弧度）
            double PENALTY_WEIGHT;
            if(angle > M_PI/2) {
                // 惩罚系数（可调整）
                PENALTY_WEIGHT = 10.0; 
            }
            else if(angle = M_PI/2) {
                // 惩罚系数（可调整）
                PENALTY_WEIGHT = 0.7; 
            }
            else if(angle = M_PI/4) {
                // 惩罚系数（可调整）
                PENALTY_WEIGHT = 0.0; 
            }
            else {
                // 惩罚系数（可调整）
                PENALTY_WEIGHT = 0.0; 
                
            }
            return PENALTY_WEIGHT;
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

//判断跳步过程中是否有边冲突
// bool check_jump_edges(const std::vector<int>& stnow, int dx, int dy, 
//     const std::unordered_set<Edge5D>& ct_edge6s_set) ;

int get_jump_distance(double density) ;

bool check_jump_path(const std::vector<int>& stnow, std::vector<int> ed, int dx, int dy,
    const std::unordered_set<Point3D>& ct_point3s_set,
    const std::unordered_set<Edge5D>& ct_edge6s_set,
    const MapLoader& ml) ;

void explore_with_jump(std::vector<int> stnow, int dx, int dy, int jump_distance,
    std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, stacmp>& open_list,
    std::vector<int>& edstage, std::vector<int> ed0, std::vector<int> st0,
    const std::unordered_set<Point3D>& ct_point3s_set,
    const std::unordered_set<Edge5D>& ct_edge6s_set,
    const MapLoader& ml) ;

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

double sta(agent* as, int i,
    std::vector<std::vector<int>> ct_point3s,
    std::vector<std::vector<int>> ct_edge6s,
    std::vector<std::vector<int>>& pths);



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

#endif 