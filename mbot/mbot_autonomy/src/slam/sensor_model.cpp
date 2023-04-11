#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    
    

    double likelihood = 0.0;

    // fraction for cell that is nearest to end cell along ray (just before/ just after)
    double fraction_nearEndCell = 0.325;
    // fraction for end cell
    double fraction_endCell = 0.35;

    // count how many  ray from this particle match the map
    int numOfMatch=0;


    // printf("\nparticle Pose (%.3f,%.3f,%.3f) ==> ",sample.parent_pose.x,sample.parent_pose.y,sample.parent_pose.theta);
    // printf("(%.3f,%.3f,%.3f)\n",sample.pose.x,sample.pose.y,sample.pose.theta);
    // printf("particleTime %lld==>%lld  lidarTime:%lld==>%lld\n",sample.parent_pose.utime,
    //                                                 sample.pose.utime,
    //                                                 scan.times[0],
    //                                                 scan.times[scan.times.size()-1]);

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO

    

    // printf("\n<sensor_model.cpp>: map size:(%d,%d)\n",map.widthInCells(),map.heightInCells());
    // printf("\t time: %d ==> %d\n",sample.parent_pose.utime,sample.pose.utime);
    // printf("\t pose: (%.3f,%.3f,%.3f) ==>",sample.parent_pose.x,sample.parent_pose.y,sample.parent_pose.theta);
    // printf("(%.3f,%.3f,%.3f)\n",sample.pose.x,sample.pose.y,sample.pose.theta);
    // printf("\t lidar time: %d==>%d\n",scan.times[0],scan.times[scan.times.size()-1]);
    for(adjusted_ray_t ray : movingScan)
    {
        float measuredRange = ray.range;
        Point<float> measuredOrigin = ray.origin;
        float measuredThetaInGlobal = ray.theta;







        
        Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
        Point<int> end_cell = global_position_to_grid_cell(
                                                            Point<double>(
                                                                ray.origin.x + ray.range * std::cos(ray.theta),
                                                                ray.origin.y + ray.range * std::sin(ray.theta)),
                                                            map);


        if(DO_PRINT_SENSOR_MODEL_DEBUG_MESSAGE)
        {
            printf("\n<sensor_model.cpp>: ray info\n");
            printf("\t position (%.3f,%.3f) ==>",ray.origin.x,ray.origin.y);
            printf("(%.3f,%.3f)\n",ray.origin.x + ray.range * std::cos(ray.theta),ray.origin.y + ray.range * std::sin(ray.theta));
            printf("\t map range (%.3f,%.3f) ==>(%.3f,%.3f)\n", map.originInGlobalFrame().x,
                                                                map.originInGlobalFrame().y,
                                                                map.originInGlobalFrame().x + map.widthInMeters(),
                                                                map.originInGlobalFrame().y + map.heightInMeters());
            printf("\t cell(%d,%d) ==> cell(%d,%d)\n",start_cell.x,start_cell.y,end_cell.x,end_cell.y);
            printf("\t map cell size:(%d,%d)\n",map.widthInCells(),map.heightInCells());
            printf("\t cellPerMeter:%.3f ,meterPerCell:%.3f\n",map.cellsPerMeter(),map.metersPerCell());
        }

        std::vector<Point<int>> cells_touched;

        bool hitBeforeEnd = false;
        Point<int> hitPoint;
        //////////////// Bresenham's Algorithm ////////////////
        int x0 = start_cell.x;
        int y0 = start_cell.y;
        

        int x1 = end_cell.x;
        int y1 = end_cell.y;

        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0;
        int y = y0;

        while (x != x1 || y != y1)
        {
            cells_touched.push_back(Point<int>(x, y));
            
            // this particle is not giving a trustable position
            // if the map indicate that ray from this particle will hit obstacle before ray ends, 
            if(map.logOdds(x,y)>0)
            {
                hitBeforeEnd = true;
                hitPoint.x = x;
                hitPoint.y = y;
                break;
            }
            int e2 = 2 * err;
            if (e2 >= -dy)
            {
                err -= dy;
                x += sx;
            }
            if(e2<=dx)
            {
                err += dx;
                y += sy;
            }
        }
        cells_touched.push_back(Point<int>(x1, y1)); // the last point is occupied, while prvious are free
        
        if(hitBeforeEnd)
        {
            // std::cout<<"<sensor_model.cpp>:   ray from particle end before reach range,";
            // std::cout<<"should end:at cell("<<x1<<","<<y1<<") but at ("<<x<<","<<y<<")\n";

            // hit occurs at the cell just before end cell (at n-1 th cell)
            if(abs(x1-x)==1 || abs(y1-y)==1 && map.logOdds(x,y)>0)
            {
                likelihood += fraction_nearEndCell * map.logOdds(x,y);
                numOfMatch++;
                // printf("hitbefore  ");
            }
            
            else
            {
                likelihood += 0;
                // printf("tooEarly  ");
            }
        }

        else
        {
            if(map.logOdds(x1,y1)>0)
            {
                likelihood += fraction_endCell * map.logOdds(x1,y1);
                numOfMatch++;
                // printf("hitexact  ");
            }

            else
            {                
                // find the point just after end cell along ray
                // Bresenham's Algorithm from (x1,y1) to (2*x1-x0,2*y1-y0)
                Point<int> cellAfterEnd;
                if (2*(dx - dy) >= -dy)
                {
                    cellAfterEnd.x = x1 + sx;
                }
                if(2*(dx - dy))
                {
                    cellAfterEnd.y= y1 + sy;
                } 
                
                if(map.logOdds(cellAfterEnd.x,cellAfterEnd.y)>0)
                {
                    likelihood += fraction_nearEndCell * map.logOdds(cellAfterEnd.x,cellAfterEnd.y);
                    numOfMatch++;
                    // printf("hitafter  ");
                }

                else
                {
                    // printf("neverHit  ");
                    continue;
                }
                
            }
        }

    }

    if(numOfMatch >= movingScan.size()*0.98)
    {
        likelihood *= 15.0;
    }

    likelihood /= 128;
    likelihood /=movingScan.size();

    // if(likelihood!=0)
    // {
    //     printf("\n<sensor_model.cpp>:\n");
    //     printf("\tparticle pose:(%.6f,%.6f,%.6f)\n",sample.pose.x,sample.pose.y,sample.pose.theta);
    //     printf("\tmatch rate:%d/%d\n",numOfMatch,movingScan.size());
    //     printf("\tlikelihood:%f\n",float(likelihood));
    // }
    





    
    return likelihood;
}