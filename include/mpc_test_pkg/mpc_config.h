#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <string>

class MPCConfig
{
    public:
        MPCConfig();
        virtual ~MPCConfig(); 
        void loadRosParamFromNodeHandle(const ros::NodeHandle & nh);

        //控制时域和预测时域
        int Np_=15;
        int Nc_=10;


        
        //采样时间
        double sample_time_=0.05;
        //状态变量个数和控制变量个数
        int basic_state_size_=3;
        int basic_control_size_=2;
        
        //松弛因子
        int relax_factor_=5.0;
        int relax_factor_max_=10.0;
        //汽车参数
        double car_length_=2.6;

        //约束参数
        //转角单个方向的范围
        double wheel_single_direction_max_degree_ = 30.0/180.0*M_PI;
        //最大转角转速
        double wheel_single_direction_max_vel_ = 40.0/180.0*M_PI;
        //最大速度
        double vx_max_ = 10.0;
        //最大纵向加速度
        double ax_max_ = 1.0;

        // parameters for mpc solver; number of iterations
        int mpc_max_iteration_ = 1000;
        // parameters for mpc solver; threshold for computation
        double mpc_eps_ = 0.05;
        //目标是否到达的范围
        double goal_threshold_= 0.05;

        std::string trajectory_type_="circle";

};
