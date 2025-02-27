#include "high_level.h"
#include <bits/stdc++.h>
#include "low_level.h"
#include "MapLoader.h"

using namespace std;

int nodecnt = 0;


void node::set(agent* as)
{
    //agent的约束点
    cts_point = vector<vector<vector<int >>> (num_agent);
    //agent的约束边
    cts_edge = vector<vector<vector<int >>> (num_agent);
    //每个agent的路径
    phs = vector<vector<vector<int >>> (num_agent);
    //临时存储路径
    vector<vector<int > > ph;
    //vector<vector<int > > ph_s;
    //为每个agent调用STA*
    for(int i=0;i<num_agent;++i)
    {
        sta(as,i,cts_point[i],cts_edge[i],ph);
        phs[i] = ph;
    }
    for(int i=0;i<num_agent;++i)
    {
        sta(as,i,cts_point[i],cts_edge[i],ph);
        phs[i] = ph;
    }
}
//继承当前节点的状态，生成新的节点
node node::inherit()
{
    node newnode;
    newnode.cts_point = cts_point;
    newnode.cts_edge = cts_edge;
    newnode.phs = phs;
    return newnode;
}
//计算节点的代价：该节点的约束下所有的agent的cost之和
void node::ccost()
{
    int c=0;
    for(int i=0;i<num_agent;++i)   c += phs[i].size() -1;
    cost = c;
}
//判断节点的有效性
void node::tvalid(vector<vector<int > >& cts_pointbig,vector<vector<int > >& cts_edge8)//to be optimzie
{
    int nume = 0;
    int nump = 0;
    //计算所有的节点的边约束和点约束的总数
    for(int i=0;i<num_agent;++i)
    {
        nume += cts_edge[i].size();
        nump += cts_point[i].size();
    }
    //printf("debug data:node %d nume %d  nump %d  cost %d\n",++nodecnt,nume,nump,cost);
    
      // test point

    //测试所有agent的点冲突
    for(int i=0;i<num_agent;++i)
    {
        stage test_p;
        test_p.agent = i;
        test_p.tvalid_agent.push_back(i);
        for(int j=0;j<phs[i].size();++j)
        {
            if(hs.find(phs[i][j])!=hs.end())    hs[phs[i][j]].tvalid_agent.push_back(i);
            else hs[phs[i][j]] = test_p;
        }
    }
    for(const auto& pair : hs)
    {
        //多个agent在同一时间戳占据同一点
        if(pair.second.tvalid_agent.size()>1)
        {
            vector<int > ct_pointbig;
            //ct_pointbig的前几个元素是冲突位置的信息 后面的是冲突agent的ID
            ct_pointbig = pair.first;
            for(int i=0;i<pair.second.tvalid_agent.size();++i) ct_pointbig.push_back(pair.second.tvalid_agent[i]);
            cts_pointbig.push_back(ct_pointbig);
        }
    }
    hs.clear();
    if(cts_pointbig.size()) return;  //如果检测到点冲突则不必再检测边冲突

    //边冲突
    stage test_e;
    vector<int > stnow,stold;
    for(int i=0;i<num_agent;++i)
    {
        test_e.agent = i;
        //对于agent i的每个时间步 j
        for(int j=0;j<phs[i].size();++j)
        {
            hs[phs[i][j]] = test_e;
            if(j)
            {
               stnow = phs[i][j];
               stold = phs[i][j-1];
               stnow[2] = stnow[2] + 1;
               stold[2] = stold[2] - 1;
               if((hs.find(stnow)!=hs.end())&&(hs.find(stold)!=hs.end()))
               {
                    //如果两个agent交换了位置
                    if((hs[stnow].agent == hs[stold].agent)&&(hs[stnow].agent != i))
                    {   
                        vector<int > ct_edge8;
                        ct_edge8.push_back(i);
                        ct_edge8.push_back(hs[stnow].agent);
                        ct_edge8.push_back(phs[i][j][0]);
                        ct_edge8.push_back(phs[i][j][1]);
                        ct_edge8.push_back(phs[i][j][2]);
                        ct_edge8.push_back(phs[i][j-1][0]);
                        ct_edge8.push_back(phs[i][j-1][1]);
                        ct_edge8.push_back(phs[i][j-1][2]);
                        cts_edge8.push_back(ct_edge8);
                    }
               } 
            }
        }
    }
    hs.clear();

    return;
}
    
void node::ptf()
    {
        for(int i=0;i<num_agent;++i)
        {
            for(int j=0;j<phs[i].size();++j)    printf("agent %d, pos %d %d %d\n",i,phs[i][j][0],phs[i][j][1],phs[i][j][2]);
        }
    }


bool ndcmp::operator()(const node& a,const node& b) const
{
    return a.cost>b.cost;
}


void split(node root,vector<int > ct_now,priority_queue<node,vector<node >, ndcmp>& pno,agent* as,int ct_mode)
{
    node newnode1,newnode2;
    newnode1 = root.inherit();
    newnode2 = root.inherit();
    if(ct_mode==1){
        int i = ct_now[0];
        int j = ct_now[1];
        vector<int > ct_point3;
        ct_point3.assign(ct_now.begin()+2,ct_now.end());
        vector<vector<int > > myph;
        //vector<vector<int > > myph_s;

        newnode1.cts_point[i].push_back(ct_point3);
        sta(as,i,newnode1.cts_point[i],newnode1.cts_edge[i],myph);
        newnode1. phs[i] = myph;
        newnode1.ccost();
        pno.push(newnode1);

        newnode2.cts_point[j].push_back(ct_point3);
        sta(as,j,newnode2.cts_point[j],newnode2.cts_edge[j],myph);
        newnode2. phs[j] = myph;
        newnode2.ccost();
        pno.push(newnode2);
        return;
    }
    int i = ct_now[0];
    int j = ct_now[1];
    vector<int > ct_edge6i,ct_edge6j;
    ct_edge6i.assign(ct_now.begin()+2,ct_now.end());
    vector<vector<int > > myph;
    //vector<vector<int > > myph_s;

    newnode1.cts_edge[i].push_back(ct_edge6i);
    sta(as,i,newnode1.cts_point[i],newnode1.cts_edge[i],myph);
    newnode1. phs[i] = myph;
    newnode1.ccost();
    pno.push(newnode1);

    ct_edge6j = ct_edge6i;
    ct_edge6j[0] = ct_edge6i[3];
    ct_edge6j[1] = ct_edge6i[4];
    ct_edge6j[3] = ct_edge6i[0];
    ct_edge6j[4] = ct_edge6i[1];
    newnode2.cts_edge[j].push_back(ct_edge6j);
    sta(as,j,newnode2.cts_point[j],newnode2.cts_edge[j],myph);
    newnode2. phs[j] = myph;
    newnode2.ccost();
    pno.push(newnode2);
    return;
    
}

vector<int > get_ct_point5(vector<vector<int > > cts_pointbig,node nnow,bool pc)
{
    vector<int > ct_point5;
    int cardinal_num = 0;
    ct_point5.push_back(cts_pointbig[0][3]);
    ct_point5.push_back(cts_pointbig[0][4]);
    ct_point5.push_back(cts_pointbig[0][0]);
    ct_point5.push_back(cts_pointbig[0][1]);
    ct_point5.push_back(cts_pointbig[0][2]);
    if(pc)
    {
        for(int i=0;i<cts_pointbig.size();++i)
        {
            vector<int > newct_point5;
            int cardinal_num_new;
            vector<int > constraint;
            vector<int > cardinal_agents;
            vector<int > notcardinal_agents;
            constraint.assign(cts_pointbig[i].begin(),cts_pointbig[i].begin()+3);
            for(int j = 3;j<cts_pointbig[i].size();++j)
            {
                vector<vector<int > > new_constraints = nnow.cts_point[cts_pointbig[i][j]];
                new_constraints.push_back(constraint);
                vector<vector<int > > pths;
                //vector<vector<int > > pths_s;
                int newres = sta(as,cts_pointbig[i][j],new_constraints,nnow.cts_edge[cts_pointbig[i][j]],pths); 
                if(newres>nnow.phs[cts_pointbig[i][j]].size()-1)    cardinal_agents.push_back(cts_pointbig[i][j]);
                else notcardinal_agents.push_back(cts_pointbig[i][j]);
            }
            cardinal_num_new = min(2,(int)cardinal_agents.size());
            for(int i=0;i<cardinal_num_new;++i)    newct_point5.push_back(cardinal_agents[i]);
            for(int i=0;i<2-cardinal_num_new;++i)    newct_point5.push_back(notcardinal_agents[i]);
            newct_point5.push_back(constraint[0]);
            newct_point5.push_back(constraint[1]);
            newct_point5.push_back(constraint[2]);
            if(cardinal_num_new>cardinal_num)
            {
                ct_point5 = newct_point5;
                cardinal_num = cardinal_num_new;
            }
        }
    }
    return ct_point5;
}


int runCBS(agent* as,int n,bool pc)
{
    num_agent = n;
    node root,nnow;
    root.set(as);
    root.ccost();
    priority_queue<node,vector<node >, ndcmp> pno;
    pno.push(root);
    while(!pno.empty())
    {
        vector<vector<int > > newcts_pointbig;
        vector<vector<int > > newcts_edge8;
        nnow = pno.top();
        pno.pop();
        nnow.tvalid(newcts_pointbig,newcts_edge8);
        if(newcts_pointbig.size()) 
        {   
            vector<int > ct_point5 =  get_ct_point5(newcts_pointbig,nnow,pc);
            split(nnow,ct_point5,pno,as,1);//
        }
        else if(newcts_edge8.size()) split(nnow,newcts_edge8[0],pno,as,2);
        else break;
    }
    //cout<<"total cost: "<<nnow.cost<<endl;
    //nnow.ptf();

    return nnow.cost;
}