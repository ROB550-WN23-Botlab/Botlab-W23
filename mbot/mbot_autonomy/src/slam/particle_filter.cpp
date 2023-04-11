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
      quality_reinvigoration_percentage(25),
      is_random_initialized_(false)
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
    is_random_initialized_ = true;
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
        
        
        //resample posterior at time t-1 to get prior at time t
        auto prior = resamplePosteriorDistribution(&map); 
       




        auto proposal = computeProposalDistribution(prior);
        


        posterior_ = computeNormalizedPosterior(proposal, laser, map);
       
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
        

    
        if(PARTICLE_FILTER_PRINT_DEBUG_MESSAGE_)
        {
            std::cout<<"<particle_filter.cpp>:   updateFilter\n";


            printf("\tresample finished, prior particles generated\n");
            mbot_lcm_msgs::pose_xyt_t priorCP = computeParticlesAverage(prior);
            printf("\t\tprior particle mean:(%.6f,%.6f,%.6f)\n",priorCP.x,priorCP.y,priorCP.theta);


            printf("\tapply action finished, proposal particles generated\n",proposal.size());
            mbot_lcm_msgs::pose_xyt_t proposalCP = computeParticlesAverage(proposal);
            printf("\t\tproposal particle mean:(%.6f,%.6f,%.6f)\n",proposalCP.x,proposalCP.y,proposalCP.theta);

            printf("\tsensor model finished, posterior particles generated!\n",posterior_.size());
            printf("\t\tposteriorPose_ weighted mean:(%.3f,%.3f,%.3f)\n\n\n",posteriorPose_.x,posteriorPose_.y,posteriorPose_.theta);
        }
    
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

    }

    posteriorPose_ = odometry;

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
    // ParticleList prior;

    double Neff;
    double temp = 0;
    double sampleWeight = 1.0/kNumParticles_;
    int Nrein;
    if(is_random_initialized_)
        Nrein = (int)kNumParticles_*quality_reinvigoration_percentage;
    else
        Nrein = 0;
    ParticleList prior = posterior_;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0 / kNumParticles_);
    double r = dist(generator);
    int count = 0;
    double c = posterior_[0].weight;
    
    // do low variance resample
    for(int i=0; i<kNumParticles_;i++)
    {
        double u = r + (double)i*1.0/kNumParticles_;
        while (u>c)
        {
            count++;
            c += posterior_[count].weight;
        }
        prior[i] = posterior_[count];
        prior[i].weight = sampleWeight;
    }
        
    
    if(samplingAugmentation.sample_randomly()){
        randomPoseGen.update_map(map);
        std::random_shuffle(prior.begin(), prior.end());
        for(int i=0; i<Nrein; i++){
            prior[i] = randomPoseGen.get_particle();
            prior[i].weight = sampleWeight;
        }
        std::cout << "reinvigorate.\n";
    }
    // std::cout << "Particles resampled\n";    


    return prior;
}



ParticleList ParticleFilter::computeProposalDistribution(const ParticleList &prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    for (mbot_lcm_msgs::particle_t p : prior)
    {
        mbot_lcm_msgs::particle_t particleAfterAction = actionModel_.applyAction(p);
        proposal.push_back(particleAfterAction);
    }
    // printf("action model finish: %ld prior particle ==> %ld proposal particle\n\n\n",prior.size(),proposal.size());
    return proposal;
}

ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList &proposal,
                                                        const mbot_lcm_msgs::lidar_t &laser,
                                                        const OccupancyGrid &map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution


    
    ParticleList posterior;
    double weightSum = 0;
    for (mbot_lcm_msgs::particle_t p : proposal)
    {
        p.weight = sensorModel_.likelihood(p, laser, map);
        weightSum += p.weight;
        posterior.push_back(p);
    }

    assert(weightSum>0.01);

    printf("weight sum: %.3f\n\n\n",weightSum);
    

    for (mbot_lcm_msgs::particle_t p_ : posterior)
    {
        p_.weight /= weightSum;
    }

    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList &posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // posterior must be normalized (weightSum = 1)

    struct particle_t_comparator
    {
        bool operator() (mbot_lcm_msgs::particle_t i,mbot_lcm_msgs::particle_t j) { return (i.weight > j.weight);}
    };
    
    mbot_lcm_msgs::pose_xyt_t pose;

    double percentage = 0.05; // 0.2
    particle_t_comparator comparator;
    ParticleList posterior_sorted = posterior;
    std::sort(posterior_sorted.begin(), posterior_sorted.end(), comparator);

    // std::cout<<" some simple test" << (posterior_sorted[0].weight >= posterior_sorted[1].weight) << std::endl;

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    double totalWeight = 0.0;
    int idx = 0;
    // using best percentage particles
    for(auto& p : posterior_sorted){
        if (idx > static_cast<int>(posterior_sorted.size() * percentage))
            break;
        xMean +=p.weight * p.pose.x;
        yMean +=p.weight * p.pose.y;
        cosThetaMean += p.weight*std::cos(p.pose.theta);
        sinThetaMean += p.weight*std::sin(p.pose.theta);
        totalWeight += p.weight;
        idx++;
    }

    pose.x = xMean / totalWeight;
    pose.y = yMean / totalWeight;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    
    std::cout << "Mean pose calculated.\n"<<" "<<pose.x<<" " << pose.y <<" "<< pose.theta << std::endl;
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList &particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    long double xSum = 0;
    long double ySum = 0;
    long double thetaSum = 0;

    long double weightSum = 0;
    for (mbot_lcm_msgs::particle_t p : particles_to_average)
    {
        assert(p.weight>0);
        assert(p.weight<1);
        xSum += (p.pose.x * p.weight);
        ySum += (p.pose.y * p.weight);
        thetaSum += (p.pose.theta * p.weight);
        weightSum += p.weight;
    }
    assert(weightSum<1.05);
    avg_pose.utime = particles_to_average[0].pose.utime;
    avg_pose.x = float(xSum);
    avg_pose.y = float(ySum);
    avg_pose.theta = float(thetaSum);
    return avg_pose;
}
