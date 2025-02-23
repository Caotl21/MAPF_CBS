#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H
#include <bits/stdc++.h>
#include "MapLoader.h"
#include <map>
#include <unordered_map>
#include <functional>
#include <vector>
#include <cstddef> // 包含size_t定义


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
    int open = 0;
    void geth(std::vector<int > ed, double w = 1.0);
    //void ptf();
    int direction;
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

void explore(std::vector<int> stnow, int dx, int dy,
    std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, stacmp>& open_list,
    std::vector<int>& edstage, std::vector<int> ed0, std::vector<int> st0, 
    std::unordered_map<std::pair<int, int>, std::map<int, double>, pair_hash>& dominance_map,
    int& pruned_nodes, int& total_nodes);

int sta(agent* as, int i,
    std::vector<std::vector<int>> ct_point3s,
    std::vector<std::vector<int>> ct_edge6s,
    std::vector<std::vector<int>>& pths);


extern std::unordered_map<std::vector<int >,stage,Hashfunc,Equalfunc> hs;

void visualize_path(const MapLoader& ml, 
    const std::vector<std::vector<int>>& path,
    const agent& agent,
    int scale=20);

#endif 