#include<bits/stdc++.h>
#include "low_level.h"
#include "MapLoader.h"
#include <map>
#include <unordered_map>
#include <functional>
#include <opencv2/opencv.hpp>

using namespace std;

int num_agent;
// 设置agent的起点终点
void agent::set(int stx,int sty,int edx,int edy)
{
    st.push_back(stx);
    st.push_back(sty);
    ed.push_back(edx);
    ed.push_back(edy);
}

agent* as;

// 启发式代价h  曼哈顿距离
//void stage::geth(vector<int > ed){
//       h = abs(post[0]-ed[0])+abs(post[1]-ed[1]);
//}

// 计算启发式代价 h（切比雪夫距离）
void stage::geth(vector<int> ed) {
    int dx = abs(post[0] - ed[0]);
    int dy = abs(post[1] - ed[1]);
    h = max(dx, dy); // 切比雪夫距离
}
// 打印节点状态
void stage::ptf()
{
    printf("post is %d %d %d, g is %d, h is %d\n",post[0],post[1],post[2],g,h);
}

// 自定义哈希函数，用于将 vector<int> 作为键存入 unordered_map
size_t Hashfunc::operator() (const vector<int>& key) const{
    return hash<int>()(key[0]) ^ hash<int>()(key[1]) ^ hash<int>()(key[2]);
}
// 自定义相等函数，用于比较两个 vector<int> 是否相等
bool Equalfunc::operator() (const vector<int>& a, const vector<int>& b) const{
    return a[0] == b[0] && a[1] == b[1] && a[2]==b[2];
}

// 全局节点表，存储所有已探索的状态
unordered_map<vector<int >,stage,Hashfunc,Equalfunc> hs;

// 自定义比较器，用于优先队列的排序规则（按 f = g + h 从小到大排序）
bool stacmp::operator()(const vector<int>& a,const vector<int >& b) const
{
    return (hs[a].g+hs[a].h)>(hs[b].g+hs[b].h);
}
// 判断是否移动是否合法
// bool ifvalid(vector<int > stnow ,int dx ,int dy ,vector<vector<int> > ct_point3s ,vector<vector<int > > ct_edge6s)
// {
//     int xnew = stnow[0] + dx;
//     int ynew = stnow[1] + dy;
//     int tnew = stnow[2] + 1;
//     // 边冲突
//     for(int i =0;i<ct_edge6s.size();++i)
//     {
//         if((stnow[0]==ct_edge6s[i][0])&&(stnow[1]==ct_edge6s[i][1])&&(stnow[2]==ct_edge6s[i][2])&&(xnew==ct_edge6s[i][3])&&(ynew==ct_edge6s[i][4])) return false;
//     }
//     // 位置冲突
//     for(int i=0;i<ct_point3s.size();++i)
//     {
//         if((xnew==ct_point3s[i][0])&&(ynew==ct_point3s[i][1])&&(tnew==ct_point3s[i][2])) return false;
//     }

// 使用哈希表加速冲突检测
//bool ifvalid(const std::vector<int>& stnow, int dx, int dy, const std::unordered_set<std::vector<int>, Hashfunc, Equalfunc>& ct_point3s_set, const std::vector<std::vector<int>>& ct_edge6s)
bool ifvalid(const std::vector<int>& stnow, int dx, int dy,
    const std::unordered_set<Point3D>& ct_point3s_set,
    const std::unordered_set<Edge5D>& ct_edge6s_set,
    const MapLoader& ml) 
{
    int xnew = stnow[0] + dx;
    int ynew = stnow[1] + dy;
    int tnew = stnow[2] + 1;

    // // 边冲突
    // for (const auto& edge : ct_edge6s) {
    //     if (stnow[0] == edge[0] && stnow[1] == edge[1] && stnow[2] == edge[2] &&
    //         xnew == edge[3] && ynew == edge[4]) {
    //         return false;
    //     }
    // }

    // // 位置冲突
    // if (ct_point3s_set.find({xnew, ynew, tnew}) != ct_point3s_set.end()) {
    //     return false;
    // }

    // 边冲突检测o(1)
    Edge5D current_edge{stnow[0], stnow[1], stnow[2], xnew, ynew};
    if (ct_edge6s_set.find(current_edge) != ct_edge6s_set.end()) {
        return false;
    }

    // 位置冲突o(1)
    Point3D current_point{xnew, ynew, tnew};
    if (ct_point3s_set.find(current_point) != ct_point3s_set.end()) {
        return false;
    }
    // 检查新位置是否在地图范围内
    if (!((xnew<ml.getHeight())&&(ynew<ml.getWidth())&&(xnew>=0)&&(ynew>=0))) return false;
    // 检查新位置是否有障碍物
    int map_data = ml.getMapData(xnew, ynew);
    if (map_data == -1) {
        return false;  // 新位置是静态障碍物
    } else if (map_data > 0) {
        // 动态障碍物检查：如果新位置的时间步大于等于动态障碍物的时间步，则冲突
        if (tnew >= map_data) {
            return false;
        }
    }
    // 检查斜向移动时是否经过障碍物的角落
    if (dx != 0 && dy != 0) {      
        int xside1 = stnow[0] + dx;
        int yside1 = stnow[1];
        int xside2 = stnow[0];
        int yside2 = stnow[1] + dy;

        if (ml.getMapData(xside1, yside1) == -1 || ml.getMapData(xside2, yside2) == -1) {
            return false;
        }
    }
    
    return true;
}

// 节点拓展
//void explore(vector<int > stnow,int dx,int dy,priority_queue<vector<int >,vector<vector<int>>, stacmp > &open_list,vector<int >& edstage,vector<int > ed0)
void explore(std::vector<int> stnow, int dx, int dy,std::priority_queue<vector<int>, std::vector<vector<int>>, stacmp>& open_list,std::vector<int>& edstage, std::vector<int> ed0,std::unordered_map<std::pair<int, int>, std::map<int, double>, pair_hash>& dominance_map,int& pruned_nodes, int& total_nodes)
{
    total_nodes++;
    vector<int > stnew;
    stnew.push_back(stnow[0]+dx);
    stnew.push_back(stnow[1]+dy);
    stnew.push_back(stnow[2]+1);

    // 计算移动代价
    double cost = (dx != 0 && dy != 0) ? std::sqrt(2) : 1;
    //用于剪枝比较
    double new_g = hs[stnow].g + cost;

    //Dominance剪枝
     /* 分层支配剪枝策略 */
    bool dominated = false;
    std::pair<int, int> pos(stnew[0], stnew[1]);
    int t_new = stnew[2];
     
     // 查找空间位置对应的时空记录
    auto& pos_records = dominance_map[pos];
    
     // 检查所有时间步<=当前时间的记录
    auto it_min = pos_records.upper_bound(t_new);
    if (it_min != pos_records.begin()) {
        --it_min; // 定位到最后一个<=t_new的记录
        while (true) {
            if (it_min->second <= new_g) {
                dominated = true;
                break;
            }
            if (it_min == pos_records.begin()) break;
            --it_min;
       }
    }
     
    if (dominated) {
        pruned_nodes++;
        return;
    }
 
     // 插入新记录并清理被支配的后续记录
    auto [inserted_it, success] = pos_records.insert({t_new, new_g});
    if (!success && new_g < inserted_it->second) {
        inserted_it->second = new_g; // 更新更优的g值
    }
     
     // 删除被当前节点支配的后续节点
    auto it_clean = pos_records.upper_bound(t_new);
    while (it_clean != pos_records.end()) {
        if (it_clean->second >= new_g) {
            it_clean = pos_records.erase(it_clean);
        } else {
            ++it_clean;
        }
    }

    // 在hs中查找stnew
    auto it = hs.find(stnew);
    // 如果位于hs中
    if(it!=hs.end())
    {
        if(hs[stnew].open==1)  //且位于openlist中
        {
            if (hs[stnow].g + cost < hs[stnew].g)    // 如果新路径代价更小
            { 
                hs[stnew].g = hs[stnow].g + cost;    // 更新代价
                hs[stnew].parent = stnow;            // 更新父节点
                open_list.push(stnew);               // 重新加入开放列表
            }
        }
    }
    // hs中没有该节点
    else
    {
        stage newst;
        newst.post = stnew;          // 设置新状态的坐标
        newst.geth(ed0);             // 计算启发式代价 h
        newst.g = hs[stnow].g + cost; // 设置实际代价 g
        newst.open = 1;              // 标记为在开放列表中
        newst.parent = stnow;        // 设置父节点
        hs[stnew] = newst;           // 加入状态表
        open_list.push(stnew);       // 加入开放列表

        // 探索到目标节点
        if((stnew[0]==ed0[0])&&(stnew[1]==ed0[1])) edstage = stnew;
    }
}

// 主函数：Space-Time A*
//int sta(agent* as,int i,vector<vector<int> > ct_point3s,vector<vector<int > > ct_edge6s,vector<vector<int > >& pths)
int sta(agent* as,int i,std::vector<vector<int> > ct_point3s,std::vector<vector<int > > ct_edge6s,vector<vector<int > >& pths)
{
    // 初始化支配记录结构
    std::unordered_map<std::pair<int, int>, std::map<int, double>, pair_hash> dominance_map;
    int pruned_nodes = 0;
    int total_nodes = 0;
    ml.setMapData(as[i].ed[0],as[i].ed[1],0); // 设置终点在地图上的数据
    
    vector<int > st0 = as[i].st;   // 得到该agent的起点
    vector<int > ed0 = as[i].ed;   // 得到该agent的终点

    // 检查终点是否合法
    if(ml.getMapData(ed0[0],ed0[1])) 
    {
        cout<<"end point is not valid"<<endl;
        return -1;
    }

    // // 将 ct_point3s 转换为 unordered_set 以加速冲突检测
    // unordered_set<vector<int>, Hashfunc, Equalfunc> ct_point3s_set;
    // for (const auto& point : ct_point3s) 
    // {
    //    ct_point3s_set.insert(point);
    // }

     // 转换边冲突数据为哈希集合
     std::unordered_set<Edge5D> ct_edge6s_set;
     ct_edge6s_set.reserve(ct_edge6s.size());  // 预分配空间
     for (const auto& e : ct_edge6s) {
         ct_edge6s_set.emplace(e[0], e[1], e[2], e[3], e[4]);
     }
    // 将 ct_point3s 转换为 unordered_set 以加速冲突检测
    //unordered_set<vector<int>, Hashfunc, Equalfunc> ct_point3s_set;
    std::unordered_set<Point3D> ct_point3s_set;
    ct_point3s_set.reserve(ct_point3s.size());
    for (const auto& p : ct_point3s) {
        if (p.size() >= 3) {
            ct_point3s_set.emplace(p[0], p[1], p[2]);
        }
    }

    stage st;                     //定义起点
    int res = 0;                  //cost
    vector<int > edstage;         //定义终点

    //对起点进行初始化
    st.post = st0;             
    st.post.push_back(0);
    st.g = 0;
    st.geth(ed0);
    hs[st.post] = st;        //将起点加入状态表
    //定义openlist
    priority_queue<vector<int>,vector<vector<int >>,stacmp> open_list;
    //将起点加入openlist，并设置标志位
    open_list.push(st.post);
    hs[st.post].open = 1;

     // 定义八方向的移动增量
     int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
     int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

    //定义当前已经走过的路径
    vector<vector<int > > pathnow;
    //主循环：如果openlist不为空则始终继续
    while(!open_list.empty())
    {
        //取出openlist中cost最小的节点并从openlist中删掉它
        vector<int > stnow = open_list.top();
        open_list.pop();
        //如果该节点为终点
        if(stnow[0]==ed0[0]&&stnow[1]==ed0[1])
        {
            pathnow.push_back(stnow);
            pths = pathnow;
            break;
        }
        if(hs[stnow].open==1)
        {
            // 向八个方向探索
            for (int i = 0; i < 8; ++i) 
            {
                if (ifvalid(stnow, dx[i], dy[i], ct_point3s_set, ct_edge6s_set, ml))
                    //explore(stnow, dx[i], dy[i], open_list, edstage, ed0);
                    explore(stnow, dx[i], dy[i], open_list, edstage, ed0, dominance_map, pruned_nodes, total_nodes);
            }
        }
        hs[stnow].open = -1;

        //找到终点
        if(edstage.size())
        {
            res = hs[edstage].g;
            vector<int > itter=edstage;
            pathnow.push_back(itter);
            //回溯路径
            while(itter[2])
            {
                itter = hs[itter].parent;
                pathnow.push_back(itter);
                pths = pathnow;
            }
            break;
        }
    }
    //将Agent走到的终点设置为障碍物
    ml.setMapData(as[i].ed[0],as[i].ed[1],res);
    hs.clear();
    // 输出剪枝统计
    std::cout << "Dominance pruning rate: " << (pruned_nodes * 100.0 / total_nodes) << "%" << std::endl;
    return res;
}

//可视化处理
void visualize_path(const MapLoader& ml, 
    const std::vector<std::vector<int>>& path,
    const agent& agent,
    int scale) 
{
// 创建彩色地图图像
int rows = ml.getHeight();
int cols = ml.getWidth();
cv::Mat map_img(rows*scale, cols*scale, CV_8UC3, cv::Scalar(255,255,255));

// 绘制栅格地图
for(int r=0; r<rows; ++r) {
for(int c=0; c<cols; ++c) {
if(ml.getMapData(r,c) == -1) { // 静态障碍物
 cv::rectangle(map_img,
     cv::Point(c*scale, r*scale),
     cv::Point((c+1)*scale-1, (r+1)*scale-1),
     cv::Scalar(0,0,0), // 黑色
     cv::FILLED);
}
else if(ml.getMapData(r,c) > 0) { // 动态障碍物
 cv::rectangle(map_img,
     cv::Point(c*scale, r*scale),
     cv::Point((c+1)*scale-1, (r+1)*scale-1),
     cv::Scalar(150,150,150), // 灰色
     cv::FILLED);
}
}
}

// 绘制路径
if(!path.empty()) {
vector<cv::Point> path_points;
for(const auto& p : path) {
// 注意坐标转换：地图(r,c)对应图像(x,y)=(c,r)
int x = p[1] * scale + scale/2;
int y = p[0] * scale + scale/2;
path_points.emplace_back(x, y);
}

// 绘制连线
for(size_t i=1; i<path_points.size(); ++i) {
cv::line(map_img, 
     path_points[i-1], 
     path_points[i],
     cv::Scalar(0,0,255), // 红色
     2);
}

// 绘制路径点
for(const auto& pt : path_points) {
cv::circle(map_img, pt, 3, cv::Scalar(255,0,0), -1); // 蓝色点
}
}

// 绘制起点和终点
cv::Point start(agent.st[1]*scale + scale/2, agent.st[0]*scale + scale/2);
cv::Point end(agent.ed[1]*scale + scale/2, agent.ed[0]*scale + scale/2);
cv::circle(map_img, start, 6, cv::Scalar(0,255,0), -1);  // 绿色起点
cv::circle(map_img, end, 6, cv::Scalar(0,165,255), -1);  // 橙色终点

// 添加图例
cv::putText(map_img, "Start", start + cv::Point(10,-10), 
cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0));
cv::putText(map_img, "Goal", end + cv::Point(10,-10), 
cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,165,255));
cv::putText(map_img, "Path", cv::Point(10,20), 
cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));

// 显示图像
cv::imshow("Path Visualization", map_img);
cv::waitKey(0);

// 保存图像
cv::imwrite("astar_path.png", map_img);
}