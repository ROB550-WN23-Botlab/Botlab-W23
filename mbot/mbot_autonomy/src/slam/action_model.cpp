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

    //+-3 std will cover 99.7% data
    double resampleIntervalInStd = 6 / (NUM_OF_RESAMPLE - 1);
    double xStart = previousPose_.x - xStd_ * ((NUM_OF_RESAMPLE - 1.0) / 2);
    double yStart = previousPose_.y - yStd_ * ((NUM_OF_RESAMPLE - 1.0) / 2);
    double thetaStart = previousPose_.theta - thetaStd_ * ((NUM_OF_RESAMPLE - 1.0) / 2);

    std::vector<mbot_lcm_msgs::particle_t> particleSet;

    

    for (int xNum = 0; xNum < NUM_OF_RESAMPLE; xNum++)
    {
        for (int yNum = 0; yNum < NUM_OF_RESAMPLE; yNum++)
        {
            for (int thetaNum = 0; thetaNum < NUM_OF_RESAMPLE; thetaNum++)
            {
                double x = xStart + xNum * resampleIntervalInStd * xStd_;
                double y = yStart + yNum * resampleIntervalInStd * yStd_;
                double theta = thetaStart + thetaNum * resampleIntervalInStd * thetaStd_;

                // use gaussian probability distribution function to calculate weight
                double p_x = normal_pdf(x, previousPose_.x, xStd_);
                double p_y = normal_pdf(y, previousPose_.y, yStd_);
                double p_theta = normal_pdf(theta, previousPose_.theta, thetaStd_);
                // assuming x,y,theta are indipendent
                double weight = p_x * p_y * p_theta;

                mbot_lcm_msgs::pose_xyt_t previousPos;
                previousPos.x = x;
                previousPos.y = y;
                previousPos.theta = theta;

                mbot_lcm_msgs::particle_t inputParticle;
                inputParticle.parent_pose = previousPos;
                inputParticle.pose = odometry;
                inputParticle.weight = weight;

                

                mbot_lcm_msgs::particle_t updatedParticle = applyAction(inputParticle);
                
                
                particleSet.push_back(updatedParticle);
            }
        }
    }

    // update mean, std
    int totalResampleNum = NUM_OF_RESAMPLE * NUM_OF_RESAMPLE * NUM_OF_RESAMPLE;

    long double weightedSumOfX =0;
    long double weightedSumOfY =0;
    long double weightedSumOfTheta =0;


    long double weightedSumOfXSquare=0;
    long double weightedSumOfYSquare =0;
    long double weightedSumOfThetaSquare =0;
    
    long double sumOfWeight = 0;

    for (int i=0;i<totalResampleNum;i++)
    {
        double weightedX = particleSet[i].pose.x * particleSet[i].weight;
        double weightedY = particleSet[i].pose.y * particleSet[i].weight;
        double weightedTheta = particleSet[i].pose.theta * particleSet[i].weight;

        double weightedXSquare = particleSet[i].pose.x * particleSet[i].pose.x * particleSet[i].weight;
        double weightedYSquare = particleSet[i].pose.y * particleSet[i].pose.y * particleSet[i].weight;
        double weightedThetaSquare = particleSet[i].pose.theta * particleSet[i].pose.theta * particleSet[i].weight;
        

        weightedSumOfX += weightedX;
        weightedSumOfY += weightedY;
        weightedSumOfTheta += weightedTheta;
        weightedSumOfXSquare += weightedXSquare;
        weightedSumOfYSquare += weightedYSquare;
        weightedSumOfThetaSquare += weightedThetaSquare;
        sumOfWeight += particleSet[i].weight;
    }

    // normalize mean
    double MeanOfX = weightedSumOfX / sumOfWeight;
    double MeanOfY = weightedSumOfY / sumOfWeight;
    double MeanOfTheta = weightedSumOfTheta / sumOfWeight;

   
   
    previousPose_.x=MeanOfX;
    previousPose_.y=MeanOfY;
    previousPose_.theta=MeanOfTheta;


    // Update variance of x,y,theta





    // update time
    previousPose_.utime = odometry.utime;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t &sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    mbot_lcm_msgs::pose_xyt_t curPose = sample.pose;
    mbot_lcm_msgs::pose_xyt_t prevPose = sample.parent_pose;

    double delta_x = curPose.x - prevPose.x;
    double delta_y = curPose.y - prevPose.y;
    double delta_theta = (curPose.theta - prevPose.theta);

    double delta_angle1 = atan2(delta_x, delta_y) - prevPose.theta; // angle of rotation 1  // alpha
    double delta_s = sqrt(delta_x * delta_x + delta_y * delta_y);   // distance of trans 1
    double delta_angle2 = delta_theta - delta_angle1;               // angle of rotation 2  //delta_theta - alpha

    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<double> distribution_noise_rot1(0, k1_ * delta_angle1);
    std::normal_distribution<double> distribution_noise_trans1(0, k2_ * delta_s);
    std::normal_distribution<double> distribution_noise_rot2(0, k1_ * delta_angle2);

    double epsilon_noise_rot1 = distribution_noise_rot1(gen);
    double epsilon_noise_trans1 = distribution_noise_trans1(gen);
    double epsilon_noise_rot2 = distribution_noise_rot2(gen);

    mbot_lcm_msgs::pose_xyt_t newPose;

    newPose.x = float(curPose.x +
                      (delta_s + epsilon_noise_trans1) * cos(curPose.theta + delta_angle1 + epsilon_noise_rot1));
    newPose.y = float(curPose.y +
                      (delta_s + epsilon_noise_trans1) * sin(curPose.theta + delta_angle1 + epsilon_noise_rot1));
    newPose.theta = delta_theta + epsilon_noise_rot1 + epsilon_noise_rot2;

    newPose.utime = curPose.utime;

    newSample.pose = newPose;
    newSample.parent_pose = curPose;
    newSample.weight = sample.weight;

    return newSample;
}
