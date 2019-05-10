#include "mpc_test_pkg/mpc_config.h"

MPCConfig::MPCConfig()
{
    ;
}

MPCConfig::~MPCConfig()
{
    ;
}

void MPCConfig::loadRosParamFromNodeHandle(const ros::NodeHandle & nh)
{

    nh.param("Np",Np_,Np_);
    nh.param("Nc",Nc_,Nc_);
    nh.param("sample_time",sample_time_,sample_time_);
    nh.param("basic_state_size",basic_state_size_,basic_state_size_);
    nh.param("basic_control_size",basic_control_size_,basic_control_size_);
    nh.param("relax_factor",relax_factor_,relax_factor_);
    nh.param("relax_factor_max",relax_factor_max_,relax_factor_max_);
    nh.param("car_length",car_length_,car_length_);
    nh.param("wheel_single_direction_max_degree",wheel_single_direction_max_degree_,wheel_single_direction_max_degree_);
    nh.param("wheel_single_direction_max_vel",wheel_single_direction_max_vel_,wheel_single_direction_max_vel_);
    nh.param("vx_max",vx_max_,vx_max_);
    nh.param("ax_max",ax_max_,ax_max_);
    nh.param("mpc_max_iteration",mpc_max_iteration_,mpc_max_iteration_);
    nh.param("mpc_eps",mpc_eps_,mpc_eps_);
    nh.param("goal_threshold",goal_threshold_,goal_threshold_);
    nh.param("trajectory_type",trajectory_type_,trajectory_type_);
}