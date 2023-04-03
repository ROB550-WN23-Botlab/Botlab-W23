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

    double delta_angle1 = angle_diff(atan2(dx_, dy_) , previousPose_.theta); // angle of rotation 1  // alpha
    double delta_s = sqrt(dx_ * dx_ + dy_ * dy_);   // distance of trans 1
    double delta_angle2 = angle_diff(dtheta_ , delta_angle1);               // angle of rotation 2  //delta_theta - alpha

    // std::random_device rd;
    // std::mt19937 gen(rd());

    stdOfAngle1_ = k1_ * delta_angle1;
    stdOfTrans1_ = k2_ * delta_s;
    stdOfAngle2_ = k1_ * delta_angle2;

    previousPose_ = odometry;

    printf("meanPose:(%.3f,%.3f,%.3f),std:(%.3f,%.3f,%.3f)\n",previousPose_.x,previousPose_.y,previousPose_.theta,stdOfAngle1_,stdOfTrans1_,stdOfAngle2_);




    return moved;




    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /*
    // variables for particle recording

    long double weightedSumOfX =0;
    long double weightedSumOfY =0;
    long double weightedSumOfTheta =0;


    long double weightedSumOfXSquare=0;
    long double weightedSumOfYSquare =0;
    long double weightedSumOfThetaSquare =0;
    
    long double sumOfWeight = 0;

    
    // Resampling 
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

                double weightedX = updatedParticle.pose.x * updatedParticle.weight;
                double weightedY = updatedParticle.pose.y * updatedParticle.weight;
                double weightedTheta = updatedParticle.pose.theta * updatedParticle.weight;

                double weightedXSquare = updatedParticle.pose.x * updatedParticle.pose.x * updatedParticle.weight;
                double weightedYSquare = updatedParticle.pose.y * updatedParticle.pose.y * updatedParticle.weight;
                double weightedThetaSquare = updatedParticle.pose.theta * updatedParticle.pose.theta * updatedParticle.weight;
                

                weightedSumOfX += weightedX;
                weightedSumOfY += weightedY;
                weightedSumOfTheta += weightedTheta;
                weightedSumOfXSquare += weightedXSquare;
                weightedSumOfYSquare += weightedYSquare;
                weightedSumOfThetaSquare += weightedThetaSquare;
                sumOfWeight += updatedParticle.weight;
                
                
                // particleSet.push_back(updatedParticle);
            }
        }
    }

    // normalize weight sum to 1
    double MeanOfX = weightedSumOfX / sumOfWeight;
    double MeanOfY = weightedSumOfY / sumOfWeight;
    double MeanOfTheta = weightedSumOfTheta / sumOfWeight;

    double MeanOfXSquare = weightedSumOfXSquare / sumOfWeight;
    double MeanOfYSquare = weightedSumOfYSquare / sumOfWeight;
    double MeanOfThetaSquare = weightedSumOfThetaSquare / sumOfWeight;

    // calculate variance using formula: σ² = Σ (w_i * x_i²) / Σ w_i - (μ_w)² 

    double varX = MeanOfXSquare - (MeanOfX*MeanOfX);
    double varY = MeanOfYSquare - (MeanOfY*MeanOfY);
    double varTheta = MeanOfThetaSquare - (MeanOfTheta*MeanOfTheta);

   
    // update distribution
    dx_ = MeanOfX -   previousPose_.x;
    dy_ = MeanOfY -   previousPose_.y;
    dtheta_ = MeanOfTheta -   previousPose_.theta;

    previousPose_.x=MeanOfX;
    previousPose_.y=MeanOfY;
    previousPose_.theta=MeanOfTheta;

    xStd_ = sqrt(varX);
    yStd_ = sqrt(varY);
    thetaStd_ = sqrt(varTheta);






    // update time
    previousPose_.utime = odometry.utime;

    return moved;
    */
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

    newPose.x = float(curPose.x +
                      (delta_s_ + epsilon_noise_trans1) * cos(curPose.theta + delta_angle1_ + epsilon_noise_rot1));
    newPose.y = float(curPose.y +
                      (delta_s_ + epsilon_noise_trans1) * sin(curPose.theta + delta_angle1_ + epsilon_noise_rot1));

    newPose.theta = curPose.theta + dtheta_ + epsilon_noise_rot1 + epsilon_noise_rot1;

    newPose.utime = previousPose_.utime;

    newSample.pose = newPose;
    newSample.parent_pose = curPose;
    newSample.weight = sample.weight;

    printf("noised Pose:(%f,%f,%f)\n",newPose.x,newPose.y,newPose.theta);

    return newSample;
}
