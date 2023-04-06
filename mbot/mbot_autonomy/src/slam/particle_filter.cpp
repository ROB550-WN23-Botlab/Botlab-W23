#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles),
      samplingAugmentation(0.5, 0.9, numParticles),
      distribution_quality(1),
      quality_reinvigoration_percentage(25)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t &pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    std::cout << "initial pose:(" << pose.x << "," << pose.y << "," << pose.theta << ")" << std::endl;
    for(auto &&p : posterior_)
    {
        p.pose.x = pose.x;
        p.pose.y = pose.y;
        p.pose.theta = pose.theta;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid &map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    for (auto &&p : posterior_)
    {
        randomPoseGen.update_map(&map);
        p = randomPoseGen.get_particle();
        p.weight = sampleWeight;
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    actionModel_.resetPrevious(odometry);
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t &odometry,
                                                       const mbot_lcm_msgs::lidar_t &laser,
                                                       const OccupancyGrid &map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        std::cout<<"<particle_filter.cpp>:   updateFilter\n";
        
        auto prior = resamplePosteriorDistribution(&map);
        printf("\tprior generated! size:%d\n",prior.size());
        auto proposal = computeProposalDistribution(prior);
        printf("\tproposal generated! size:%d\n",proposal.size());
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        printf("\tposterior_ generated! size:%d\n",posterior_.size());
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
        printf("\tposteriorPose_ generated!:(%.3f,%.3f,%.3f)\n\n\n",posteriorPose_.x,posteriorPose_.y,posteriorPose_.theta);
    }
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();

        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;

        // for(mbot_lcm_msgs::particle_t p : posterior_)
        // {
        //     std::cout<<"prior particle:("<<p.pose.x<<","<<p.pose.y<<","<<p.pose.y<<","<<p.pose.theta<<")\n";
        // }
    }

    posteriorPose_ = odometry;

    // std::cout << "<particle _filter.cpp>  pose:(" << posteriorPose_.x << "," << posteriorPose_.y << "," << posteriorPose_.theta << ")\n";
    // std::cout<<"robot moved:"<<hasRobotMoved<<std::endl;
    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}

mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid *map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;

    // low variance sampler
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> rSampler(0, 1.0 / kNumParticles_);

    double r = rSampler(gen);

    int particleIdx = 0;
    double acw = posterior_[0].weight; // accumulate weight

    for (int i = 0; i < kNumParticles_; i++)
    {
        double u = r + (1.0 * i) / kNumParticles_; // make sure we are using "double" in fraction
        while (acw < u)
        {
            particleIdx++;
            if(particleIdx>=kNumParticles_)
            {
                particleIdx-=kNumParticles_;
            }
            acw += posterior_[particleIdx].weight;
            // std::cout<<"<particle_filter.cpp>:   resamplePosteriorDistribution\n";
            // printf("\taccumulated weight:%.3f\n",acw);
        }
        prior.push_back(posterior_[particleIdx]);
    }

    // std::cout << "<particle_filter.cpp>, resample not enabled\n";
    // prior = posterior_;

    return prior;
}

ParticleList ParticleFilter::computeProposalDistribution(const ParticleList &prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    for (auto &p : prior)
    {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}

ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList &proposal,
                                                        const mbot_lcm_msgs::lidar_t &laser,
                                                        const OccupancyGrid &map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution


    // std::cout<<"<particle_filter.cpp>\n";
    // for(int i=0;i<map.widthInCells();i++)
    // {
    //     for(int j=0;i<map.heightInCells();j++)
    //     {
    //         printf("cell(%d,%d):%d  ",i,j,map.logOdds(i,j));
    //     }
    // }
    // printf("map cell size:(%d,%d)\n",map.widthInCells(),map.heightInCells());
    
    ParticleList posterior;
    double weightSum = 0;
    for (mbot_lcm_msgs::particle_t p : proposal)
    {
        p.weight = sensorModel_.likelihood(p, laser, map);
        weightSum += p.weight;
        posterior.push_back(p);
    }

    for (mbot_lcm_msgs::particle_t p_ : posterior)
    {
        p_.weight /= weightSum;
    }

    return posterior;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList &posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    pose = computeParticlesAverage(posterior); // posterior must be normalized (weightSum = 1)
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList &particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    avg_pose.x = 0;
    avg_pose.y = 0;
    avg_pose.theta = 0;
    for (mbot_lcm_msgs::particle_t p : particles_to_average)
    {
        avg_pose.x += p.pose.x * p.weight;
        avg_pose.y += p.pose.y * p.weight;
        avg_pose.theta += p.pose.theta * p.weight;
    }
    avg_pose.utime = particles_to_average[0].pose.utime;
    return avg_pose;
}
