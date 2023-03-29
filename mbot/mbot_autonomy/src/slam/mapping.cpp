#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
#include <math.h>
using namespace std::chrono; 



Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    printf("in Mapping::updateMap \n");
    
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_)
    {
        previousPose_ = pose;
        initialized_ = true;
        printf("Mapping::updateMap : first position initialized!\n\n");
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(adjusted_ray_t ray : movingScan)
    {
        std::vector<int> inverseModelVal = Mapping::crudeInverseSensorModel(ray, map);
        for(int i = 0; i<map.widthInCells(); i++)
        {
            for(int j = 0; j<map.heightInCells(); j++)      
            {
                int idx = i * map.widthInCells() + j;
                map.setLogOdds(i, j, map.logOdds(i,j) + inverseModelVal[idx] - L_0);
                // printf("cell (%d,%d): %d\n",i,j,map.logOdds(i,j));
            }
        }
        
    }
    printf("finish one update!\n]n\n");
    previousPose_ = pose;
    return;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}

std::vector<int> Mapping::crudeInverseSensorModel(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    double xt = ray.origin.x;
    double yt = ray.origin.y;
    double zt = ray.range;
    double theta = ray.theta;

    std::vector<int> inverseModelVal;
    for(int i = 0; i<map.widthInCells(); i++)
    {
        for(int j = 0; j<map.heightInCells(); j++)      
        {
            // Point<double> cellCenter = map.cellInGlobalFrameInMeter(i,j);
            Point<double> cellCenter = grid_position_to_global_position(Point<double>(double(i),double(j)), map);
            // printf("cell (%d,%d):(%f,%f)\n",i,j,cellCenter.x,cellCenter.y);    
            double xi = cellCenter.x;
            double yi = cellCenter.y;

            double phi = atan2(xi-xt,yi-yt);
            if(phi<0)
            {
                phi = phi + 2*3.14159;
            }
            double d = sqrt((xi-xt)*(xi-xt)+(yi-yt)*(yi-yt));
            
            // check if point visiable by the ray
            if(abs(phi - theta) > (INVERSE_MODEL_BETA/2) || (d > (zt+INVERSE_MODEL_ALPHA/2)))
            {
                // block is not in visiable area
                // printf("\n\ncell(%d,%d) is no visiable\n ",i,j);
                // if(abs(phi - theta) > (INVERSE_MODEL_BETA/2))
                // {
                //     printf("    angle out of range: phi=%f (theta = %f)\n",phi,theta);
                // }
                // if(d > (zt+INVERSE_MODEL_ALPHA/2))
                // {
                //     printf("   d out of range: d=%f (zt=%f)\n",d,zt);
                // }
                inverseModelVal.push_back(L_0);
            }

            else if(d > (zt - INVERSE_MODEL_ALPHA/2))
            {
                // printf("cell(%d,%d) is occupied, angle_difference: %f, distance_difference:%f\n",i,j,abs(phi - theta), (d -zt));
                inverseModelVal.push_back(kHitOdds_);
            }

            else
            {
                // printf("cell(%d,%d) is free, angle_difference: %f, distance_difference:%f\n",i,j,abs(phi - theta), (d -zt));
                inverseModelVal.push_back(-kMissOdds_);
            }
        }
    }


    return inverseModelVal;
}
