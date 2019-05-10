#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<string>
#include"Eigen/Core"
#include"mpc_test_pkg/traj_generate.h"
#include<cmath>
#include <qpOASES.hpp>
#include "mpc_test_pkg/mpc_config.h"

using namespace std;


class MPCControl
{
    public:
        //@ brief 默认构造函数与析构函数
        MPCControl(double sample_time,double car_length,int Np,int Nc);
        MPCControl(const MPCConfig & mpc_cfg);
        virtual ~ MPCControl();
        
       //@ brief 设置参考轨迹
       void setRefTraj(vector<traj> & ref_traj);


        
        //@ brief 更新当前机器人的状态变量（即当前位姿）
        void updateState(traj & current_pos);
        
        //@ brief 计算当前位置和对应轨迹参考点的误差
        void computeErrors(traj & traj_ref);



        //更新机器人对应参考位置的状态矩阵
        void updateMatrix();



        //利用qpOASES求解二次规划问题
        bool qpSlover();
        
        bool getFirstControl(double& vel,double& delta);
        bool getRealControl(vector<double>& real_vel,vector<double>& real_delta);

        bool isGoalReached();


    protected:
        //@brief 计算两个点之间的距离
        double calculateDistance(traj pos1,traj pos2); 

        //@ brief 更新轨迹上离机器人实际位置最近的点
        void updateNearestRefState(traj& traj_ref_pos,int & traj_ref_index);  


        
    private:
        //存放参考轨迹
        vector<traj> ref_traj_;

        //当前位置
        traj  current_state_;
        //存放实际控制量
        vector<double> real_control_vel_;
        vector<double> real_control_delta_;
        //存放实际控制量偏差
        vector<double> real_control_vel_offset_;
        vector<double> real_control_delta_offset_;
        

        //存放每次二次规划的最优控制量和最优松弛因子
        vector<double> optim_control_;
        double  optim_relax_factor_;

        //控制时域和预测时域
        int Np_;
        int Nc_;
        
        //采样时间
        double sample_time_;
        //状态变量个数和控制变量个数
        int basic_state_size_;
        int basic_control_size_;
        
        //状态矩阵用到的参考量
        traj nearest_ref_traj_pos_;
        int nearest_ref_traj_index_;
        //松弛因子
        int relax_factor_;
        int relax_factor_max_;
        //汽车参数
        double car_length_=0.0;

        //约束参数
        //转角单个方向的范围
        double wheel_single_direction_max_degree_ = 0.0;
        //最大转角转速
        double wheel_single_direction_max_vel_ = 0.0;
        //最大速度
        double vx_max_ = 0.0;
        //最大纵向加速度
        double ax_max_ = 0.0;

        //目标是否到达的范围
        double goal_threshold_= 0.05;

        // parameters for mpc solver; number of iterations
        int mpc_max_iteration_ = 0;
        // parameters for mpc solver; threshold for computation
        double mpc_eps_ = 0.0;


        //矩阵
          // vehicle state matrix
        Eigen::MatrixXd matrix_a_;
        // vehicle state matrix (discrete-time)
        Eigen::MatrixXd matrix_ad_;

         // control matrix
        Eigen::MatrixXd matrix_b_;
         // control matrix (discrete-time)
        Eigen::MatrixXd matrix_bd_;

        // offset matrix
        Eigen::MatrixXd matrix_c_;
        // offset matrix (discrete-time)
        Eigen::MatrixXd matrix_cd_;

        //考虑控制量作为状态量的状态空间方程
        Eigen::MatrixXd matrix_A_;
        Eigen::MatrixXd matrix_B_;
        Eigen::MatrixXd matrix_C_;

        //预测矩阵
        Eigen::MatrixXd matrix_Ap_;
        Eigen::MatrixXd matrix_Bp_;

        //权重矩阵
        // control authority weighting matrix
        Eigen::MatrixXd matrix_r_;

        // state weighting matrix
        Eigen::MatrixXd matrix_q_;

 

        //二次规划矩阵
        Eigen::MatrixXd matrix_H_;
        Eigen::MatrixXd matrix_G_; //
        //当前位置与对应参考轨迹点的误差（计算matrix_G_用到） matrix_G_=[2vector_error_'Ap'QBp  0];
        Eigen::MatrixXd vector_error_; 

        ////存放当前一个时域内的偏差参考轨迹（以当前对应参考点为起点，与当前参考轨迹的偏差）
        Eigen::MatrixXd matrix_errors_ref_traj_;

        //二次规划约束矩阵
        //控制增量上下界
        Eigen::MatrixXd matrix_du_ub_;
        Eigen::MatrixXd matrix_du_lb_;

        //控制量上下界
        Eigen::MatrixXd matrix_u_ub_;
        Eigen::MatrixXd matrix_u_lb_;  

        //控制增量线性不等式约束
        Eigen::MatrixXd matrix_constraint_A_;
        Eigen::MatrixXd matrix_constraint_ubA_;
        Eigen::MatrixXd matrix_constraint_lbA_;


        




        // heading error of last control cycle
        double previous_heading_error_ = 0.0;
        // lateral distance to reference trajectory of last control cycle
        double previous_lateral_error_ = 0.0;

        // lateral dynamic variables for computing the differential valute to
        // estimate acceleration and jerk
        double previous_lateral_acceleration_ = 0.0;

        double previous_heading_rate_ = 0.0;
        double previous_ref_heading_rate_ = 0.0;

        double previous_heading_acceleration_ = 0.0;
        double previous_ref_heading_acceleration_ = 0.0;

        // longitudinal dynamic variables for computing the differential valute to
        // estimate acceleration and jerk
        double previous_acceleration_ = 0.0;
        double previous_acceleration_reference_ = 0.0;


        





};