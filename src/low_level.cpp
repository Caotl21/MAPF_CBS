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

// 计算启发式代价 h（切比雪夫距离）
void stage::geth(vector<int> ed, double w) {
    int dx = abs(post[0] - ed[0]);
    int dy = abs(post[1] - ed[1]);
    h = w * max(dx, dy); // 切比雪夫距离
}



// 动态权重计算函数
double dynamic_weight(const vector<int>& current, const vector<int>& start, 
    const vector<int>& goal) {
     // 进度因子：已走路径占总预估路径的比例
     double progress = current[2] / (abs(start[0]-goal[0]) + abs(start[1]-goal[1]) + 1e-6);
    // 动态权重曲线：初始阶段权重高，后期逐渐降低: sigmoid 
    return 1.5 - 0.5 * (1.0 / (1.0 + exp(-10*(progress-0.7))));
}

// 打印节点状态
// void stage::ptf()
// {
//     printf("post is %d %d %d, g is %d, h is %d\n",post[0],post[1],post[2],g,h);
// }

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

// 判断跳部合法性
bool ifsparse(const std::vector<int>& stnow, int dx, int dy,
    const MapLoader& ml)
{
    double predict_denstiy = ml.getDensity(stnow[1]+dx, stnow[0]+dy);
    if(predict_denstiy != 0){
        return false;
    }

    return true;

} 

// 节点拓展
void explore(std::vector<int> stnow, int dx, int dy, int density_level,
    std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, stacmp>& open_list,
    std::vector<int>& edstage, std::vector<int> ed0, std::vector<int> st0)
{
    vector<int > stnew;
    stnew.push_back(stnow[0]+dx);
    stnew.push_back(stnow[1]+dy);
    stnew.push_back(stnow[2]+1);
    // 计算当前移动方向
    pair<int,int> current_dir(dx, dy);
    
    // 获取父节点方向（如果存在）
    pair<int,int> parent_dir(0,0);
    if(!stnow.empty() && hs.find(stnow) != hs.end()) {
        parent_dir = hs[stnow].dir;
    }

    // 计算移动代价
    double cost = (dx != 0 && dy != 0) ? std::sqrt(2) : 1;
    double densitydata = ml.getDensity(stnew[1], stnew[0]);
    double densityalpha = 1 / (1 + exp(20*(densitydata - 0.35)));
    double Density_penalty = 0.5 * densitydata;
            
    
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
        double turn_penalty = newst.turn_penalty(parent_dir);
        newst.post = stnew;          // 设置新状态的坐标
        double w = dynamic_weight(stnew, st0, ed0);
        newst.geth(ed0, w);             // 计算启发式代价 h

        //分层搜索策略对应不同的代价
        switch(density_level){
            case 0 :
                newst.g = newst.g = hs[stnow].g + cost + turn_penalty + Density_penalty;
                break;
            case 1 :
                newst.g = hs[stnow].g + cost + (densityalpha * turn_penalty) + Density_penalty;
                break;
            case 2 :
                newst.g = hs[stnow].g + cost;
                break;
        }

        newst.open = 1;              // 标记为在开放列表中
        newst.parent = stnow;        // 设置父节点
        newst.dir = current_dir;     // 记录当前移动方向
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
    ml.setMapData(as[i].ed[0],as[i].ed[1],0); // 设置终点在地图上的数据
    
    vector<int > st0 = as[i].st;   // 得到该agent的起点
    vector<int > ed0 = as[i].ed;   // 得到该agent的终点

    // 检查终点是否合法
    if(ml.getMapData(ed0[0],ed0[1])) 
    {
        cout<<"end point is not valid"<<endl;
        return -1;
    }

     // 转换边冲突数据为哈希集合
     std::unordered_set<Edge5D> ct_edge6s_set;
     ct_edge6s_set.reserve(ct_edge6s.size());  // 预分配空间
     for (const auto& e : ct_edge6s) {
         ct_edge6s_set.emplace(e[0], e[1], e[2], e[3], e[4]);
     }
    // 将 ct_point3s 转换为 unordered_set 以加速冲突检测
    std::unordered_set<Point3D> ct_point3s_set;
    ct_point3s_set.reserve(ct_point3s.size());
    for (const auto& p : ct_point3s) {
        if (p.size() >= 3) {
            ct_point3s_set.emplace(p[0], p[1], p[2]);
        }
    }

    stage st;                     //定义起点
    vector<int > edstage;         //定义终点
    int res = 0;

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

    int dx_jump2[] = {0, 2, 0, -2, 2, 2, -2, -2};
    int dy_jump2[] = {2, 0, -2, 0, 2, -2, 2, -2};

    int dx_jump3[] = {0, 3, 0, -3, 3, 3, -3, -3};
    int dy_jump3[] = {3, 0, -3, 0, 3, -3, 3, -3};

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
            double current_density = ml.getDensity(stnow[1],stnow[0]);
            //低密度区
            if(current_density == 0){
             // 向八个方向探索
               for (int i = 0; i < 8; ++i) {
                    // 检查跳步中途是否有障碍物
                    if (ifvalid(stnow, dx[i], dy[i], ct_point3s_set, ct_edge6s_set, ml)){
                        if (ifvalid(stnow, dx_jump2[i], dy_jump2[i], ct_point3s_set, ct_edge6s_set, ml)
                            && ifsparse(stnow, dx_jump2[i], dy_jump2[i], ml))
                        {
                            if (ifvalid(stnow, dx_jump3[i], dy_jump3[i], ct_point3s_set, ct_edge6s_set, ml)
                                && ifsparse(stnow, dx_jump3[i], dy_jump3[i], ml))
                            {
                                explore(stnow, dx_jump3[i], dy_jump3[i], 0, open_list, edstage, ed0, st0);
                            }
                            else{
                                explore(stnow, dx_jump2[i], dy_jump2[i], 0, open_list, edstage, ed0, st0);
                            }
                        }
                        else{
                            explore(stnow, dx[i], dy[i], 0, open_list, edstage, ed0, st0);
                        }
                    }
                }
            }
            //高密度区
            else if(current_density > 0.7){
                // 向八个方向探索
               for (int i = 0; i < 8; ++i) {
                    if (ifvalid(stnow, dx[i], dy[i], ct_point3s_set, ct_edge6s_set, ml)){
                        explore(stnow, dx[i], dy[i], 2, open_list, edstage, ed0, st0);
                    }
                }
            }
            //中密度区
            else{
                // 向八个方向探索
               for (int i = 0; i < 8; ++i) {
                if (ifvalid(stnow, dx[i], dy[i], ct_point3s_set, ct_edge6s_set, ml)){
                    explore(stnow, dx[i], dy[i], 1, open_list, edstage, ed0, st0);
                }
            }
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
            }
            pths = pathnow;
            //pths_smooth = PathSmoother::bezierSmooth(pathnow, ml);
            break;
        }
    }
    //将Agent走到的终点设置为障碍物
    ml.setMapData(as[i].ed[0],as[i].ed[1],res);

    return res;
    //为了绘制搜索区域
    //hs.clear();

}

// 路径成本计算函数
double calculatePathCost(const std::vector<std::vector<int>>& path) {
    double total_cost = 0.0;
    
    // 路径至少需要2个点才能计算移动成本
    if (path.size() < 2) return 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        const auto& prev = path[i-1];
        const auto& curr = path[i];
        
        // 确保坐标维度正确
        if (prev.size() < 2 || curr.size() < 2) {
            cerr << "Invalid coordinate dimension at step " << i << endl;
            continue;
        }

        int dx = abs(curr[0] - prev[0]);
        int dy = abs(curr[1] - prev[1]);

        // 计算移动类型
        if (dx == 1 && dy == 1) {
            total_cost += sqrt(2); // 对角线移动
        } else if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1)) {
            total_cost += 1.0; // 直线移动
        } else if (dx == 2 && dy == 2) {
            total_cost += 2*sqrt(2); // 对角线移动jump
        } else if ((dx == 2 && dy == 0) || (dx == 0 && dy == 2)) {
            total_cost += 2.0; // 直线移动jump
        } else if (dx == 3 && dy == 3) {
            total_cost += 3*sqrt(2); // 对角线移动jump
        } else if ((dx == 3 && dy == 0) || (dx == 0 && dy == 3)) {
            total_cost += 3.0; // 直线移动jump 
        } else {
            // 异常移动处理
            cerr << "Warning: Irregular movement from (" 
                 << prev[0] << "," << prev[1] << ") to ("
                 << curr[0] << "," << curr[1] << ")" << endl;
        }
    }
    return total_cost;
}

// 在可视化函数前添加辅助函数
std::vector<std::vector<bool>> generate_visited_from_hs(const std::unordered_map<std::vector<int>, stage, Hashfunc, Equalfunc>& hs, 
    int map_height, int map_width)
{
std::vector<std::vector<bool>> visited(map_height, std::vector<bool>(map_width, false));

// 遍历哈希表所有键
for(const auto& entry : hs) {
const std::vector<int>& state = entry.first;
// 假设状态前两维是行列坐标
int r = state[0]; 
int c = state[1];
if(r >=0 && r < map_height && c >=0 && c < map_width) {
visited[r][c] = true;
}
}
return visited;
}


//可视化处理
void visualize_path(const MapLoader& ml, 
    const std::vector<std::vector<int>>& path,
    //const std::vector<std::vector<int>>& path_smooth,
    const agent& agent,
    const std::unordered_map<std::vector<int>, stage, Hashfunc, Equalfunc>& hs,
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

    auto visited = generate_visited_from_hs(hs, ml.getHeight(), ml.getWidth());
    // 绘制被访问过的节点
    for (int r = 0; r < visited.size(); ++r) {
        for (int c = 0; c < visited[r].size(); ++c) {
            if (visited[r][c]) {
                int center_x = c*scale + scale/2;
                int center_y = r*scale + scale/2;
                int arm_length = scale/4;  // 十字臂长
                
                // 绘制水平线
                cv::line(map_img,
                         cv::Point(center_x - arm_length, center_y),
                         cv::Point(center_x + arm_length, center_y),
                         cv::Scalar(0, 0, 0), 
                         2);  // 线宽
                
                // 绘制垂直线
                cv::line(map_img,
                         cv::Point(center_x, center_y - arm_length),
                         cv::Point(center_x, center_y + arm_length),
                         cv::Scalar(0, 0, 0),
                         2);
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
                    cv::Scalar(255,0,0), 
                    3);
        }

    // 生成并绘制平滑路径（绿色实线）
    //auto smooth_path = PathSmoother::bezierSmooth(path_smooth, ml);
    // if(!path_smooth.empty()) {
    //     vector<cv::Point> smooth_points;
    //     for(const auto& p : path_smooth) {
    //         // 注意平滑路径可能没有时间维度
    //         int x = (p.size() >= 2) ? p[1] * scale + scale/2 : 0;
    //         int y = (p.size() >= 1) ? p[0] * scale + scale/2 : 0;
    //         smooth_points.emplace_back(x, y);
    //     }
        
    //     // 绘制带箭头的路径
    //     for(size_t i = 1; i < smooth_points.size(); ++i) {
    //         cv::arrowedLine(map_img, 
    //                        smooth_points[i-1], 
    //                        smooth_points[i],
    //                        cv::Scalar(0, 255, 0), // 绿色
    //                        2,                     // 线宽
    //                        cv::LINE_AA,
    //                        0,                     // 无偏移
    //                        0.1);                  // 箭头尖端比例
    //     }
    // }

// 绘制路径点
        for(const auto& pt : path_points) {
            cv::circle(map_img, pt, 7, cv::Scalar(0,0,255), -1); // 蓝色点
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
    cv::putText(map_img, "Raw Path", cv::Point(10, 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(225, 0, 0));
    cv::putText(map_img, "Density", cv::Point(10, 60), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

// 绘制障碍物密度图
    cv::Mat density_img;
    cv::Mat scaled_density;
    cv::resize(ml.getDensityMap(), scaled_density, 
          cv::Size(cols*scale, rows*scale), // 注意OpenCV尺寸顺序是(width, height)
          0, 0, cv::INTER_NEAREST); // 保持离散值的清晰度
    cv::normalize(
        scaled_density,  // 直接使用OpenCV矩阵
        density_img,
        0, 255,
        cv::NORM_MINMAX,
        CV_8UC1  // 确保输出为8位无符号单通道 [!code focus]
    );

    // 应用伪彩色时需要转换为3通道
    cv::applyColorMap(density_img, density_img, cv::COLORMAP_JET);

    // 叠加时确保map_img也是彩色图
    if (map_img.channels() == 1) {
        cv::cvtColor(map_img, map_img, cv::COLOR_GRAY2BGR);
    }

    cv::addWeighted(map_img, 0.7, density_img, 0.3, 0, map_img);

// 显示图像
    cv::imshow("Path Visualization", map_img);
    cv::waitKey(0);

// 保存图像
    cv::imwrite("astar_path.png", map_img);
}