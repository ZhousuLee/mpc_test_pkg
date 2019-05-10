#include"mpc_test_pkg/traj_generate.h"
#include<iostream>
#include<vector>
#include"mpc_test_pkg/mpc_control.h"
#include"mpc_test_pkg/sim_locate.h"

using namespace std;

int main()
{
    TrajGenerate traj_generator1(500,0.05,"circle");

    vector<traj> traj1;
    vector<double> time1;
    traj_generator1.getTraj(traj1,time1);
    for(int i=0;i<time1.size();i++)
    {
       // cout<<traj1[i].x<<" ";
       // cout<<time1[i]<<" ";
       // cout<<endl;
    }
 


    //此段程序为测试sim_locate文件
    SimLocate  simlocater(2.6,traj1.front());
    
    for(int i=1;i<time1.size();i++)
    {
        simlocater.updateRungeKuttaPosition(traj1[i].vel,traj1[i].delta,0.05);
    }
    vector<traj> real_traj;
    simlocater.getRealTraj(real_traj);
    for(int i=0;i<time1.size();i++)
    {
        cout<<traj1[i].x-real_traj[i].x<<"   ";
        cout<<traj1[i].y-real_traj[i].y<<"   ";
        cout<<traj1[i].phi-real_traj[i].phi<<"   "<<endl;
        
    }    
    cout<<endl;
    cout<<real_traj.size()<<endl;
    cout<<traj1.size()<<endl;
    
    //------------结束sim_locate测试
 



    return 0;
    
}