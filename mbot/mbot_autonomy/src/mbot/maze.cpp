#include <common_utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    int numTimes = 1;
    int numOfPoint = 9;
    double ll = 0.61;
    double pp = M_PI/2;
    double pointXList[9]={ll,ll,2*ll,2*ll,3*ll,3*ll,4*ll,4*ll,5*ll};
    double pointYList[9]={0,ll,ll,-ll,-ll,ll,ll,0,0};
    double pointThetaList[9]={pp,0,-pp,0,pp,0,-pp,0,0};

    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    mbot_lcm_msgs::robot_path_t path;
    path.path.resize(9);
    
    mbot_lcm_msgs::pose_xyt_t nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numOfPoint; n++)
    {
        nextPose.x = pointXList[n];
        nextPose.y = pointYList[n];
        nextPose.theta = pointThetaList[n];
        path.path[n] = nextPose;
    }
    
    
    
    // Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
