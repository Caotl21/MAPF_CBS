#ifndef High_Level_H
#define High_Level_H
#include <bits/stdc++.h>
#include "low_level.h"
#include "MapLoader.h"

extern int node_num;

struct node
{
    std::vector<std::vector<std::vector<int > > > cts_point;// cts_point[i]  [j] = (x,y,t)
    std::vector<std::vector<std::vector<int> > > cts_edge;// cts_edge[i] [j] = (x1,y2,t1,x2,y2,t2)
    std::vector<std::vector<std::vector<int > > > phs;
    double cost;
    void set(agent* as);

    node inherit();

    void ccost();
    //find point_conflict first, then edge_conflict
    void tvalid(std::vector<std::vector<int > >& cts_pointbig,std::vector<std::vector<int > >& cts_edge8);//cts_pointbig[] = (x,y,t,i1,i2,...) cts_edge8[] = (i,j,x1,y1,t1,x2,y2,t2)

    void ptf();

};
struct ndcmp{
    bool operator()(const node& a,const node& b) const;
};

void split(node root,std::vector<int > ct_now,std::priority_queue<node,std::vector<node >, ndcmp>& pno,agent* as,int ct_mode);

std::vector<int > get_ct_point5(std::vector<std::vector<int > > cts_pointbig,node nnow,bool PC=false);

double runCBS(agent* as,int n,bool pc=false);
#endif