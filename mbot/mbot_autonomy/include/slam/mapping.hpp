#ifndef SLAM_MAPPING_HPP
#define SLAM_MAPPING_HPP

#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/lidar_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <cstdint>
#include <vector>
#include <tuple>


// find definition of param at https://youtu.be/1f_m5aJFIj4  lecture 12
// radius difference of ring  (ring center is the ray end point, cell in ring are consider to be occupied)
double REVERSE_MODEL_ALPHA = 0.01;

// angle of approximation
double REVERSE_MODEL_BETA = 2*3.1415926/100;  

int L_FREE = -2;   // ln(p_occupied/p_free)  = log(0.25/0.75)

int L_OCCUPIED = 2;   // ln(p_occupied/p_free)  = log(.75/0.25)

int L_0 = 0;   // log(prior), typically 0

// 
/**
* Mapping implements the occupancy grid mapping algorithm. 
*/
class Mapping
{
public:
    
    /**
    * Constructor for Mapping.
    * 
    * \param    maxLaserDistance    Maximum distance for the rays to be traced
    * \param    hitOdds             Increase in occupied odds for cells hit by a laser ray
    * \param    missOdds            Decrease in occupied odds for cells passed through by a laser ray
    */
    Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds);
    
    /**                                                                                     *
    * updateMap incorporates information from a new laser scan into an existing OccupancyGrid.
    * 
    * \param    scan            Laser scan to use for updating the occupancy grid
    * \param    pose            Pose of the robot at the time when the last ray was measured
    * \param    map             OccupancyGrid instance to be updated
    */
    void updateMap(const mbot_lcm_msgs::lidar_t& scan, const mbot_lcm_msgs::pose_xyt_t& pose, OccupancyGrid& map);


    /**                                                                                     *
    * crude inverse senfor model , see https://youtu.be/1f_m5aJFIj4  lecture 12
    * 
    * \param    ray             adjusted_ray_t, single laser ray
    * \param    map             OccupancyGrid, grid map which we need to update
    * \return   vector, containing inververse sensor model value of each cell
    */
    std::vector<int> crudeInverseSensorModel(const adjusted_ray_t& ray, OccupancyGrid& map);

private:
    
    const float  kMaxLaserDistance_;
    const int8_t kHitOdds_;
    const int8_t kMissOdds_;
    
    bool initialized_;
    mbot_lcm_msgs::pose_xyt_t previousPose_;
    void scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map);
    void scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map);
    std::vector<Point<int>> bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map);
    std::vector<Point<int>> divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map);
    
    //////////////////// TODO: Add any private members needed for your occupancy grid mapping algorithm ///////////////

};

#endif // SLAM_MAPPING_HPP
