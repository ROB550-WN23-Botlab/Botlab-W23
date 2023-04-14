#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

ActionModel::ActionModel(void)
    : k1_(0.02f), k2_(0.001f), min_dist_(0.0025), min_theta_(0.02), initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd()); 
    dx_ = 0;
    dy_ = 0;
    dtheta_ = 0;

    delta_angle1_ = 0;
    delta_s_ = 0;
    delta_angle2_ = 0;
    
    stdOfAngle1_ = 0;
    stdOfTrans1_ = 0;
    stdOfAngle2_ = 0;
}

void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    previousPose_ = odometry;
}

bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    bool moved = 0;
    if(!initialized_)
    {
        previousPose_ = odometry;
        initialized_ = true;
        std::cout<<"\n<action_model.cpp>:\taction model initialized!\n";
    }

    

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    delta_angle1_ = 0;
    delta_s_ = 0;
    delta_angle2_ = 0;
    
    stdOfAngle1_ = 0;
    stdOfTrans1_ = 0;
    stdOfAngle2_ = 0;


    if(dx_!=0 || dy_!=0 || dtheta_ !=0)
    {
        moved = true;
        delta_angle1_ = angle_diff(atan2(dy_,dx_) , previousPose_.theta); // angle of rotation 1  // alpha
        delta_s_ = sqrt(dx_ * dx_ + dy_ * dy_);   // distance of trans 1
        delta_angle2_ = angle_diff(dtheta_ , delta_angle1_);               // angle of rotation 2  //delta_theta - alpha
        stdOfAngle1_ = k1_ * delta_angle1_;
        stdOfTrans1_ = k2_ * delta_s_;
        stdOfAngle2_ = k1_ * delta_angle2_;
    }


    // printf("\n<action_model.cpp>: updateAction\n");
    // printf("\t prev(%.3f,%.3f,%.3f)",previousPose_.x,previousPose_.y,previousPose_.theta);
    // printf("==> odom(%.3f,%.3f,%.3f)\n",odometry.x,odometry.y,odometry.theta);
    // printf("\t utime:%lld\n",odometry.utime);
    // printf("\t(rot1,trans1,rot2) mean:(%.3f,%.3f,%.3f),std:(%.5f,%.5f,%.5f)\n",delta_angle1_,delta_s_,delta_angle2_,stdOfAngle1_,stdOfTrans1_,stdOfAngle2_);

    
    utime_ = odometry.utime;
    previousPose_ = odometry;

   
    return moved;

}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t &sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    
    
    mbot_lcm_msgs::particle_t newSample;
    mbot_lcm_msgs::pose_xyt_t curPose = sample.pose;
    mbot_lcm_msgs::pose_xyt_t prevPose = sample.parent_pose;



    std::normal_distribution<double> distribution_noise_rot1(0, stdOfAngle1_);
    std::normal_distribution<double> distribution_noise_trans1(0, stdOfTrans1_);
    std::normal_distribution<double> distribution_noise_rot2(0, stdOfAngle2_);

    double epsilon_noise_rot1 = distribution_noise_rot1(numberGenerator_);
    double epsilon_noise_trans1 = distribution_noise_trans1(numberGenerator_);
    double epsilon_noise_rot2 = distribution_noise_rot2(numberGenerator_);


    

    newSample.pose.x = float(curPose.x +
                      (delta_s_ + epsilon_noise_trans1) * cos(curPose.theta + delta_angle1_ + epsilon_noise_rot1));
    newSample.pose.y = float(curPose.y +
                      (delta_s_ + epsilon_noise_trans1) * sin(curPose.theta + delta_angle1_ + epsilon_noise_rot1));

    newSample.pose.theta = curPose.theta + (delta_angle1_+delta_angle2_) + epsilon_noise_rot1 + epsilon_noise_rot1;
    newSample.pose.theta = wrap_to_pi(newSample.pose.theta);


    newSample.pose.utime = utime_;

    newSample.parent_pose = sample.pose;

    newSample.weight = sample.weight;


    // printf("noised Pose:(%f,%f,%f)\n",newPose.x,newPose.y,newPose.theta);

    
    
    
    // std::cout<<"<action_model.cpp>: \t";
    // std::cout<<"applyAction generate 1 particle:\n";
    // printf("\t(rot1,trans1,rot2) mean:(%.3f,%.3f,%.3f),std:(%.3f,%.3f,%.3f)\n",delta_angle1_,delta_s_,delta_angle2_,stdOfAngle1_,stdOfTrans1_,stdOfAngle2_);
    // printf("\t(%.3f,%.3f,%.3f)",newSample.parent_pose.x,newSample.parent_pose.y,newSample.parent_pose.theta);
    // printf("==> (%.3f,%.3f,%.3f)\n",newSample.pose.x,newSample.pose.y,newSample.pose.theta);
    // printf("\tparticle time: %d\n",newSample.pose.utime);

    
    return newSample;
}
