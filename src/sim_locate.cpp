#include"mpc_test_pkg/sim_locate.h"

SimLocate::SimLocate(double car_length,traj init_pos)
{
    car_length_=car_length;
    real_traj_.clear();
    real_traj_.push_back(init_pos);
}
void SimLocate::updateEulerPosition(double vel,double delta,double sample_time)
{
    traj  next_pos;
    next_pos.x=real_traj_.back().x+vel*cos(real_traj_.back().phi)*sample_time;
    next_pos.y=real_traj_.back().y+vel*sin(real_traj_.back().phi)*sample_time;
    next_pos.phi=real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time;
    next_pos.vel=vel;
    next_pos.delta=delta;

    real_traj_.push_back(next_pos);
}

void SimLocate::updateRungeKuttaPosition(double vel,double delta,double sample_time)
{
    traj  next_pos;
    next_pos.x=real_traj_.back().x+vel*cos(real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time/2.0)*sample_time;
    next_pos.y=real_traj_.back().y+vel*sin(real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time/2.0)*sample_time;
    next_pos.phi=real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time;
    next_pos.vel=vel;
    next_pos.delta=delta;

    real_traj_.push_back(next_pos);
}


bool SimLocate::getCurrentPosition(traj & current_state)
{
    if(real_traj_.size()==0)
    return false;
    current_state=real_traj_.back();
    return true;

}

void SimLocate::getRealTraj(vector<traj>& real_traj)
{
    real_traj.resize(real_traj_.size());
    for(int i=0;i<real_traj_.size();i++)
        real_traj[i]=real_traj_[i];
}
