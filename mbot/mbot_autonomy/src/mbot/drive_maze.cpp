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
    int numOfPoint = 6;
    double ll = 0.61;
    double pp = M_PI/2;
    // double pointXList[6]={ll,ll,2*ll,2*ll,3*ll,2*ll,2*ll,1*ll,1*ll,0};
    // double pointYList[6]={0,-ll,-ll,0,0,0,-ll+0.1,-ll+0.1,0+0.1,0+0.1};
    // double pointThetaList[6]={-pp,0,pp,0,-2*pp,-pp,-2*pp,pp,-2*pp,0};
    double pointXList[6]={0.3,0.9,1.5, 0.9, 0.3, 0};
    double pointYList[6]={0, -0.61,0,-0.61,-0.05,0};
    double pointThetaList[6]={-0.25*M_PI, 0.25*M_PI, -0.75*M_PI, 0.75*M_PI, -1*M_PI, 0};
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    mbot_lcm_msgs::robot_path_t path;
    path.path.resize(10);
    
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
