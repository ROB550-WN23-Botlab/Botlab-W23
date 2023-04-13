#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// DONE - TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;            
    random_gen = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// DONE - TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    bool moved = 0;
    if(!initialized_)
    {
        previousPose_ = odometry;
        initialized_ = true;
    }
    float direction = 1.0;
    moved_ = 0;
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;
    ds_ = sqrt(dx_*dx_ + dy_*dy_);
    utime_ = odometry.utime;
    previousPose_ = odometry;
    if(ds_ < min_dist_ && dtheta_ < min_theta_){
        moved_ = 0;
        return moved_;
    }
    else{
        moved_ = 1;
        alpha_ = angle_diff(atan2(dy_,dx_),previousPose_.theta);
        if(abs(alpha_) > M_PI/2.0){
            alpha_ = angle_diff(M_PI, alpha_);
            direction = -1.0f;
        }
        ds_= ds_*direction;

        eps_sig_1_ = sqrt(k1_*abs(alpha_));
        eps_sig_2_ = sqrt(k2_*abs(ds_));
        eps_sig_3_ = sqrt(k1_*abs(dtheta_ - alpha_));
        return moved_;
    }

}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// In Progress - TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
     // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    // Assign predicted pose into pose
        // Sample one of the action result using normal_sample_

    float sampled_eps_1 = normal_sample_(0.0, eps_sig_1_);
    float sampled_eps_2 = normal_sample_(0.0, eps_sig_2_);
    float sampled_eps_3 = normal_sample_(0.0, eps_sig_3_);

        //Update the alpha and distance with sampled error
    float dx_sampled = ((float)ds_ + sampled_eps_2) * (float)cos((double)newSample.pose.theta + alpha_ + (double)sampled_eps_1);
    float dy_sampled = ((float)ds_ + sampled_eps_2) * (float)sin((double)newSample.pose.theta + alpha_ + (double)sampled_eps_1);
    float dtheta_sampeld = wrap_to_pi(dtheta_ + sampled_eps_1 + sampled_eps_3);
   
        //Update the newSample.pose with delta calculated
    newSample.pose.x += dx_sampled;
    newSample.pose.y += dy_sampled;
    newSample.pose.theta += dtheta_sampeld;
    newSample.pose.utime = utime_;
        // Turn parent_pose into current pose and turn pose into the sampled future pose
    newSample.parent_pose = sample.pose;
    return newSample;
}


float ActionModel::normal_sample_(float mean, float stddev)
{
    // Use random seed and normal_distribution to random sample from the given mean and stddev
    std::normal_distribution<float> d(mean, stddev);
    float sample_ = d(random_gen);
    return sample_;
}
