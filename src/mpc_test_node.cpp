#include"mpc_test_pkg/traj_generate.h"
#include<iostream>
#include<vector>
#include"mpc_test_pkg/mpc_control.h"
#include"mpc_test_pkg/sim_locate.h"
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sstream"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h> 
#include <ros/console.h>
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 
#include <geometry_msgs/Quaternion.h> 


using namespace std;
void publishCycle(ros::Publisher & pub,vector<traj>  & real_traj)
{
    if (real_traj.size()==0)
    {
       ROS_INFO("real_traj size is 0");
        return;
    }
    
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="world";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id="world";
    for(int i=0;i<real_traj.size();i++)
    {
        pose_stamped.pose.position.x=real_traj[i].x;
        pose_stamped.pose.position.y=real_traj[i].y;
        
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(real_traj[i].phi);
		pose_stamped.pose.orientation.x = goal_quat.x; 
		pose_stamped.pose.orientation.y = goal_quat.y; 
		pose_stamped.pose.orientation.z = goal_quat.z; 
		pose_stamped.pose.orientation.w = goal_quat.w;
         
        path.poses.push_back(pose_stamped);
        

    }
    pub.publish(path);
    //cout<<path.poses.size();
    //cout<<"I AM IN CYCLE"<<endl;

}


int main(int argc,char ** argv)
{
    ros::init(argc,argv,"mpc_test_node");
    ros::NodeHandle nh("~");
    ros::Publisher local_plan_pub = nh.advertise<nav_msgs::Path>("local_plan",100);
    ros::Publisher real_traj_pub = nh.advertise<nav_msgs::Path>("real_traj",100);
 
    int simulate_N=500;
 
    MPCConfig cfg;

    //加载参数服务器
    cfg.loadRosParamFromNodeHandle(nh);
    traj init_pos{0,0.0,7.0,0.0,0.0,0.0};
    
    //生成轨迹
    string traj_type;
    traj_type=cfg.trajectory_type_;
    TrajGenerate traj_generator(simulate_N,cfg.sample_time_,traj_type);
    vector<traj> traj_ref;
    vector<double> time_ref;
    traj_generator.getTraj(traj_ref,time_ref);

    //初始化模拟定位器
    SimLocate sim_locater(cfg.car_length_,init_pos);

    //初始化MPC控制器

    MPCControl mpc_controller(cfg);

    
    //用于存放当前的位置
    traj current_pos;

    //用于存放最优控制量
    double control_vel;
    double control_delta;


    mpc_controller.setRefTraj(traj_ref);
    //模拟整个轨迹的运动
    //for(int i=0;i<traj_ref.size();i++)
    int max_iteration = 2*simulate_N;
    while(!mpc_controller.isGoalReached() && (--max_iteration>0))
    
    {
        //获得机器当前位置
        sim_locater.getCurrentPosition(current_pos);
        //告诉mpc当前机器人位置
        mpc_controller.updateState(current_pos);
        mpc_controller.updateMatrix();
        mpc_controller.qpSlover();

        mpc_controller.getFirstControl(control_vel,control_delta);

        // //根据控制量 模拟机器人运动
        sim_locater.updateRungeKuttaPosition(control_vel,control_delta,cfg.sample_time_);
     
    }
   

    vector<traj> traj_real;
    sim_locater.getRealTraj(traj_real);
    //cout<<endl<<endl<<endl<<endl; 
    //cout<<"this is difference between traj_real and traj_ref : "<<endl;

    //for(int j=0;j<traj_ref.size();j++)
       // cout<<traj_real[j].x-traj_ref[j].x<<" "<<traj_real[j].y-traj_ref[j].y<<" "<<endl;

    

    vector<double> real_control_vel;
    vector<double> real_control_delta;
    
    mpc_controller.getRealControl(real_control_vel,real_control_delta);
    //cout<<real_control_vel.size();
    //for(int j=0;j<real_control_vel.size();j++)
        //cout<<real_control_vel[j]<<" "<<real_control_delta[j]<<" "<<endl;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        publishCycle(real_traj_pub,traj_real);
        publishCycle(local_plan_pub,traj_ref);
        if(! nh.hasParam("Np"))  
            ROS_INFO("NOT  GET  PARAM!!!!!!!!!!!");
        else
            ROS_INFO("GET  PARAM!!!!!!!!!!!");

        ros::spinOnce();
        loop_rate.sleep();
    }


    //cout<<"end"<<endl;
    return 0;
    
}
