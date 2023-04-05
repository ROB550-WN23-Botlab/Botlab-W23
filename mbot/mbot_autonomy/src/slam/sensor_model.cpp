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

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO

    // count how many  ray from this particle match the map
    int numOfMatch=0;


    for(adjusted_ray_t ray : movingScan)
    {
        float measuredRange = ray.range;
        Point<float> measuredOrigin = ray.origin;
        float measuredThetaInGlobal = ray.theta;

        // Get global positions
        Point<float> f_end = global_position_to_grid_position(
            Point<float>(
                ray.origin.x + ray.range * std::cos(ray.theta),
                ray.origin.y + ray.range * std::sin(ray.theta)),
            map);

        // Cells
        Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
        Point<int> end_cell;
        end_cell.x = static_cast<int>(f_end.x);
        end_cell.y = static_cast<int>(f_end.y);
        std::vector<Point<int>> cells_touched;

        bool hitBeforeEnd = false;
        Point<int> hitPoint;
        //////////////// Bresenham's Algorithm ////////////////
        int y0 = start_cell.y;
        int x0 = start_cell.x;
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

            // hit occurs at the cell just before end cell (at n-1 th cell)
            if(abs(x1-x)==1 || abs(y1-y)==1 && map.logOdds(x,y)>0)
            {
                likelihood += fraction_nearEndCell * map.logOdds(x,y);
                numOfMatch++;
            }
            
            else
            {
                likelihood += 0;
            }
        }

        else
        {
            if(map.logOdds(x1,y1)>0)
            {
                likelihood += fraction_endCell * map.logOdds(x1,y1);
                numOfMatch++;
            }

            else
            {
                // find the point just after end cell along ray
                // Bresenham's Algorithm from (x1,y1) to (2*x1-x0,2*y1-y0)
                Point<int> cellAfterEnd;
                if (2*(dx - dy) >= -dy)
                {
                    cellAfterEnd.x = x + sx;
                }
                if(2*(dx - dy))
                {
                    cellAfterEnd.y= y1 + sy;
                } 
                
                if(map.logOdds(cellAfterEnd.x,cellAfterEnd.y)>0)
                {
                    likelihood += fraction_nearEndCell * map.logOdds(cellAfterEnd.x,cellAfterEnd.y);
                    numOfMatch++;
                }
            }
        }

    }
    if(numOfMatch >= movingScan.size()*0.98)
    {
        likelihood *= 15.0;
    }


    std::cout<<"<sensor_model.cpp>:   particle match map:"<< (1.0*numOfMatch/movingScan.size())<<"\n";
    std::cout<<"likelihood"<<likelihood<<"\n";



    
    return likelihood;
}