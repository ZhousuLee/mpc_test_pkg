#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<string>
#include"Eigen/Core"
#include<cmath>
using namespace std;
struct traj{
    double time;
    double x;
    double y;
    double phi;
    double vel;
    double delta;
};

class TrajGenerate{
    public:
        //构造函数
        TrajGenerate();
        TrajGenerate(int n,double sample_time,string type);

        //返回轨迹
        void getTraj(vector<traj> & traj1,vector<double> & time1);
    
    private:
        vector<traj>  ref_traj_;
        vector<double>  ref_time_;
};
