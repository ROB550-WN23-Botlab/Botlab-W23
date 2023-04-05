#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

ActionModel::ActionModel(void)
    : k1_(0.01f), k2_(0.01f), min_dist_(0.0025), min_theta_(0.02), initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}

void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    previousPose_ = odometry;
}

bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    bool moved = 0;

    

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    if(dx_!=0 || dy_!=0 || dtheta_ !=0)
    {
        moved = true;
    }

    delta_angle1_ = angle_diff(atan2(dx_, dy_) , previousPose_.theta); // angle of rotation 1  // alpha
    delta_s_ = sqrt(dx_ * dx_ + dy_ * dy_);   // distance of trans 1
    delta_angle2_ = angle_diff(dtheta_ , delta_angle1_);               // angle of rotation 2  //delta_theta - alpha

    // std::random_device rd;
    // std::mt19937 gen(rd());

    stdOfAngle1_ = k1_ * delta_angle1_;
    stdOfTrans1_ = k2_ * delta_s_;
    stdOfAngle2_ = k1_ * delta_angle2_;

    previousPose_ = odometry;

    // printf("odometry:(%.3f,%.3f,%.3f)\n",odometry.x,odometry.y,odometry.theta);

    // printf("meanPose:(%.3f,%.3f,%.3f),std:(%.3f,%.3f,%.3f)\n",previousPose_.x,previousPose_.y,previousPose_.theta,stdOfAngle1_,stdOfTrans1_,stdOfAngle2_);




    return moved;

}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t &sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample;
    mbot_lcm_msgs::pose_xyt_t curPose = sample.pose;
    mbot_lcm_msgs::pose_xyt_t prevPose = sample.parent_pose;


    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<double> distribution_noise_rot1(0, stdOfAngle1_);
    std::normal_distribution<double> distribution_noise_trans1(0, stdOfTrans1_);
    std::normal_distribution<double> distribution_noise_rot2(0, stdOfAngle2_);

    double epsilon_noise_rot1 = distribution_noise_rot1(numberGenerator_);
    double epsilon_noise_trans1 = distribution_noise_trans1(numberGenerator_);
    double epsilon_noise_rot2 = distribution_noise_rot2(numberGenerator_);

    mbot_lcm_msgs::pose_xyt_t newPose;

    // std::cout<<"<action_model.cpp: >  ";
    // std::cout<< "delta_s="<<delta_s_<<"delta_angle1="<<delta_angle1_<<"delta_angle2="<<delta_angle2_<<"\n";

    newPose.x = float(curPose.x +
                      (delta_s_ + epsilon_noise_trans1) * cos(curPose.theta + delta_angle1_ + epsilon_noise_rot1));
    newPose.y = float(curPose.y +
                      (delta_s_ + epsilon_noise_trans1) * sin(curPose.theta + delta_angle1_ + epsilon_noise_rot1));

    newPose.theta = curPose.theta + (delta_angle1_+delta_angle2_) + epsilon_noise_rot1 + epsilon_noise_rot1;

    newPose.utime = previousPose_.utime;

    newSample.pose = newPose;
    newSample.parent_pose = curPose;
    newSample.weight = sample.weight;

    // printf("noised Pose:(%f,%f,%f)\n",newPose.x,newPose.y,newPose.theta);

    // std::cout<<"previous Pose("<<sample.pose.x<<","<<sample.pose.theta<<","<<sample.pose.x<<")";
    // std::cout<<"    new Pose("<<newSample.pose.x<<","<<newSample.pose.theta<<","<<newSample.pose.x<<")\n";
    std::cout<<"<action_model.cpp: >:\n";
    std::cout<<"particle pose("<<curPose.x<<","<<curPose.y<<","<<curPose.theta<<")\n==>\n";
    std::cout<<"particle pose("<<newPose.x<<","<<newPose.y<<","<<newPose.theta<<")\n\n\n";
    return newSample;
}
