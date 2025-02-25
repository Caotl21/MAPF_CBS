#include <bits/stdc++.h>
#include "example.h"
#include "ScenarioLoader.h"
#include "MapLoader.h"
#include "high_level.h"
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::chrono;

//超时处理函数
void handle_alarm(int signal) {
    cout << "Timeout!" << endl;
    exit(0);
}


// to do     bug ,CBSH

int main() {

    //signal(SIGALRM, handle_alarm);

    //alarm(60);

    //std::cout << "Welcome to the CBS project!" << std::endl;
    exampleFunction();

    ml.loadMap("/home/caotl/MAPF_CBS/benchmark/random-32-32-10/random-32-32-10.map");//绝对路径
    ScenarioLoader sl("/home/caotl/MAPF_CBS/benchmark/random-32-32-10/random-32-32-10-random-1.scen");//绝对路径
    
    // cout<<"Num of Experiments "<<sl.GetNumExperiments()<<endl;
    // int n;
    // n = 50; //best solve n = 38 with PC  38 903,
    // int standardcost = 0;
    // as = new agent[n];
    // for(int i=0;i<n;++i)
    // {
    //     Experiment exp = sl.GetNthExperiment(i);
    //     as[i].set(exp.GetStartY(),exp.GetStartX(),exp.GetGoalY(),exp.GetGoalX());
    // }

    // auto start_time = high_resolution_clock::now();
    // int runres = runCBS(as,n,true);
    // auto end_time = high_resolution_clock::now();
    // auto duration = duration_cast<milliseconds>(end_time - start_time).count();
    // cout<<"The num of agent is "<<n<<endl;
    // cout<<"Runres is "<<runres<<endl;
    // cout << "Algorithm execution time: " << duration << " ms" << endl;

    // 测试参数设置
    const int test_agent_index = 15;  // 测试第0个agent
    const bool enable_visualization = true; // 是否显示路径

    // 初始化agent
    Experiment exp = sl.GetNthExperiment(test_agent_index);
    agent test_agent;
    test_agent.set(exp.GetStartY(), exp.GetStartX(), exp.GetGoalY(), exp.GetGoalX());
    
    // 空冲突列表（单独测试A*时没有冲突约束）
    vector<vector<int>> empty_ct_points;
    vector<vector<int>> empty_ct_edges;
    // 路径存储容器
    vector<vector<int>> final_path;

    // 执行A*算法测试
    auto start = high_resolution_clock::now();
    int total_cost = sta(&test_agent, 0, empty_ct_points, empty_ct_edges, final_path);
    auto end = high_resolution_clock::now();
 
    // 结果分析
    if(total_cost == -1) {
        cout << "Path not found!" << endl;
        return -1;
    }

    // 性能指标输出
    cout << "========== A* Algorithm Performance ==========\n";
    cout << "Agent Start: (" << test_agent.st[0] << ", " << test_agent.st[1] << ")\n";
    cout << "Agent Goal:  (" << test_agent.ed[0] << ", " << test_agent.ed[1] << ")\n";
    cout << "Total Cost: " << total_cost << " timesteps\n";
    cout << "Path Length: " << final_path.size() << " steps\n";
    cout << "Computation Time: " 
         << duration_cast<microseconds>(end - start).count() 
         << " 微秒\n";

    if(total_cost != -1) {
        // 注意路径是逆序存储的，需要反转
        reverse(final_path.begin(), final_path.end());
        visualize_path(ml, final_path, test_agent, hs);
    }

    // // 路径可视化输出
    // if(enable_visualization) {
    //     cout << "\nPath Details:\n";
    //     for(auto it = final_path.rbegin(); it != final_path.rend(); ++it) {
    //         cout << "T=" << (*it)[2] << ": (" 
    //              << (*it)[0] << ", " << (*it)[1] << ")\n";
    //     }
    // }
    // alarm(0);

    return 0;
}