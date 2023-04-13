#ifndef SLAM_PARTICLE_FILTER_HPP
#define SLAM_PARTICLE_FILTER_HPP

#include <vector>
#include <algorithm>

#include <mbot_lcm_msgs/lidar_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <mbot_lcm_msgs/particles_t.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>

#include <slam/occupancy_grid.hpp>
#include <slam/sensor_model.hpp>
#include <slam/action_model.hpp>
// #include <angle_functions.hpp>
#include <common_utils/geometric/angle_functions.hpp>

// Controls the percentage of particles to average for posterior pose estimation.
#define PERCENT_PARTICLES_TO_AVERAGE 0.20

typedef std::vector<mbot_lcm_msgs::particle_t> ParticleList;

/**
* ParticleFilter implements a standard SIR-based particle filter. The set of particles is initialized at some pose. Then
* on subsequent calls to updateFilter, a new pose estimate is computed using the latest odometry and laser measurements
* along with the current map of the environment.
*
* This implementation of the particle filter uses a fixed number of particles for each iteration. Each filter update is
* a simple set of operations:
*
*   1) Draw N particles from current set of weighted particles.
*   2) Sample an action from the ActionModel and apply it to each of these particles.
*   3) Compute a weight for each particle using the SensorModel.
*   4) Normalize the weights.
*   5) Use the max-weight or mean-weight pose as the estimated pose for this update.
*/
class ParticleFilter
{
public:

    /**
    * Constructor for ParticleFilter.
    *
    * \param    numParticles        Number of particles to use
    * \pre  numParticles > 1
    */
    ParticleFilter(int numParticles);

    /**
    * initializeFilterAtPose initializes the particle filter with the samples distributed according
    * to the provided pose estimate.
    *
    * \param    pose            Initial pose of the robot
    */
    void initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose);

    void initializeFilterRandomly(const OccupancyGrid& map);

    /**
    * updateFilter increments the state estimated by the particle filter. The filter update uses the most recent
    * odometry estimate and laser scan along with the occupancy grid map to estimate the new pose of the robot.
    *
    * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
    * \param    laser           Most recent laser scan of the environment
    * \param    map             Map built from the maximum likelihood pose estimate
    * \return   Estimated robot pose.
    */
    mbot_lcm_msgs::pose_xyt_t updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                            const mbot_lcm_msgs::lidar_t& laser,
                                            const OccupancyGrid& map);

    /**
    * updateFilterActionOnly increments the state estimated by the particle filter but only applies the action model
    *
    * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
    * \return   odometry robot pose.
    */
    mbot_lcm_msgs::pose_xyt_t updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry);

    /**
    * poseEstimate retrieves the current pose estimate computed by the filter.
    */
    mbot_lcm_msgs::pose_xyt_t poseEstimate(void) const;

    /**
    * particles retrieves the posterior set of particles being used by the algorithm.
    */
    mbot_lcm_msgs::particles_t particles(void) const;

    void resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry);

private:

    ParticleList posterior_;     // The posterior distribution of particles at the end of the previous update
    mbot_lcm_msgs::pose_xyt_t posteriorPose_;  // Pose estimate associated with the posterior distribution

    ActionModel actionModel_;   // Action model to apply to particles on each update
    SensorModel sensorModel_;   // Sensor model to compute particle weights
    float distribution_quality;
    float quality_reinvigoration_percentage;

    int kNumParticles_;         // Number of particles to use for estimating the pose

    // Used to get a pseudo-random seed for the random bit generator.
    std::random_device rand_device_;
    // The random bit generator used in the Mersenne Twister
    std::default_random_engine generator_;

    ParticleList resamplePosteriorDistribution(const OccupancyGrid* map = nullptr);
    ParticleList computeProposalDistribution(const ParticleList& prior);
    ParticleList computeNormalizedPosterior(const ParticleList& proposal,
                                            const mbot_lcm_msgs::lidar_t& laser,
                                            const OccupancyGrid& map);

    // I believe particle reinvigoration should be done in the resampling of the posterior step.
    // Opting to get rid of this function for now.
    // void reinvigoratePosterior();

    mbot_lcm_msgs::pose_xyt_t estimatePosteriorPose(const ParticleList& posterior);
    mbot_lcm_msgs::pose_xyt_t computeParticlesAverage(const ParticleList& particles_to_average);


    /**
     * @brief Used to track averages for augmenting MCL sampling with a randomness.
     *      Refer to the probabilistic robotics book section table 8.3 for more details.
     */
    class SamplingAugmentation{
      public:
        SamplingAugmentation(float slow_rate, float fast_rate, int num_particles)
        :   slow_rate(slow_rate), fast_rate(fast_rate),
            slow_average_weight(1/num_particles), fast_average_weight(1/num_particles),
            generator(std::random_device()()), uniform_distribution(0.0, 1.0),
            rand_sample_prob(0)
        { }

        bool sample_randomly()
        {
            return uniform_distribution(generator) < rand_sample_prob;
        }

        void insert_average_weight(float avg_w)
        {
            slow_average_weight += slow_rate*(avg_w - slow_average_weight);
            fast_average_weight += fast_rate*(avg_w - fast_average_weight);
            rand_sample_prob = std::max<float>(0.0, 1 - fast_average_weight/slow_average_weight);
        }

      private:
        float slow_average_weight, fast_average_weight;
        float slow_rate, fast_rate;
        float rand_sample_prob;
        std::default_random_engine generator;
        std::uniform_real_distribution<double> uniform_distribution;


    }
    samplingAugmentation;


    /**
     * @brief Randomly generates poses in the free space
     * of an occupancy grid.
     */
    class RandomPoseSampler
    {
    public:
        RandomPoseSampler(const OccupancyGrid* map_in=nullptr, int max_attempts=10000)
        : max_attempts(max_attempts)
        {
            if ( map_in == nullptr ) {  map = nullptr;  }
            else {  update_map(map_in);  }
        }

        void update_map(const OccupancyGrid* map_in)
        {
            map = map_in;
            auto origin = map->originInGlobalFrame();
            double initial_x = origin.x;
            double end_x = origin.x + map->widthInMeters();
            double initial_y = origin.y;
            double end_y = origin.y + map->heightInMeters();

            distr_x = std::uniform_real_distribution<double>(initial_x, end_x);
            distr_y = std::uniform_real_distribution<double>(initial_y, end_y);
            distr_theta = std::uniform_real_distribution<double>(-M_PI, M_PI);
        }

        mbot_lcm_msgs::pose_xyt_t get_pose()
        {
            if (map == nullptr)
            { throw std::runtime_error("Invalid map in particle filter's random sampler...");  }

            double rand_x, rand_y;
            for (int i; i < max_attempts; i++)
            {
                std::random_device random_device;
                std::default_random_engine random_engine(random_device());

                rand_x = distr_x(random_engine);
                rand_y = distr_y(random_engine);

                // Check if robot could be there
                auto cell = global_position_to_grid_cell(Point<double>(rand_x, rand_y), *map);
                // TODO: reduce available space by incorporing robot radius
                if (map->logOdds(cell.x, cell.y) < 0)
                {
                    // Generate random theta
                    double rand_theta = distr_theta(random_engine);
                    mbot_lcm_msgs::pose_xyt_t pose;
                    pose.x = rand_x;
                    pose.y = rand_y;
                    pose.theta = rand_theta;
                    pose.utime = 0;
                    return pose;
                }
            }
            // throw std::runtime_error("Particle filter failed to sample a random valid pose from map!");
        }

        mbot_lcm_msgs::particle_t get_particle()
        {
            mbot_lcm_msgs::particle_t p;
            p.pose = p.parent_pose = get_pose();
            p.weight = 0 ;
            return p;
        }

    private:
        int max_attempts;
        const OccupancyGrid* map;
        std::uniform_real_distribution<double> distr_x, distr_y, distr_theta;
    }
    randomPoseGen;

    // For normal sampling of poses given a pose (that is set to the Gaussian's mean) 
    class GaussianPoseSampler
    {
    public:
        GaussianPoseSampler(double const std_dev_position, double const std_dev_angle)
            : std_dev_position_(std_dev_position),
              std_dev_theta_(std_dev_angle),
              avg_pose_(),
              random_device_(),
              random_engine_(random_device_())
        {}

        // Provides the average pose (the means for the three Gaussians we're sampling from)
        void insert_average_pose(mbot_lcm_msgs::pose_xyt_t avg_pose)
        {
            avg_pose_ = avg_pose;
            distr_x_ = std::normal_distribution<double>(avg_pose_.x, std_dev_position_);
            distr_y_ = std::normal_distribution<double>(avg_pose_.y, std_dev_position_);
            distr_theta_ = std::normal_distribution<double>(avg_pose_.theta, std_dev_theta_);
        }

        // Samples a random pose from the distribution around the inserted average pose.
        mbot_lcm_msgs::pose_xyt_t sample_pose()
        {
            // Sample the new pose variables.
            double const sampled_x = distr_x_(random_engine_);
            double const sampled_y = distr_y_(random_engine_);
            // Need to wrap the sampled angle!
            double const sampled_theta = wrap_to_pi(distr_theta_(random_engine_));

            // Construct the pose
            mbot_lcm_msgs::pose_xyt_t sampled_pose;
            sampled_pose.x = sampled_x;
            sampled_pose.y = sampled_y;
            sampled_pose.theta = sampled_theta;
            sampled_pose.utime = avg_pose_.utime;
            
            return sampled_pose;
        }

        // Used for getting a random particle given the average pose inserted with 
        // insert_average_pose() method.
        mbot_lcm_msgs::particle_t sample_particle()
        {
            mbot_lcm_msgs::particle_t sampled_particle;
            sampled_particle.parent_pose = avg_pose_;
            sampled_particle.pose = sample_pose();
            sampled_particle.weight = 0.0;
            return sampled_particle;
        }

    private:
        double const std_dev_position_;  // m.
        double const std_dev_theta_;  // Radians^2.
        
        // The pose describing the means of the Gaussians
        mbot_lcm_msgs::pose_xyt_t avg_pose_;  

        std::normal_distribution<double> distr_x_;
        std::normal_distribution<double> distr_y_;
        std::normal_distribution<double> distr_theta_;

        std::random_device random_device_;
        std::default_random_engine random_engine_;

    } gaussian_pose_sampler;

};

#endif // SLAM_PARTICLE_FILTER_HPP
