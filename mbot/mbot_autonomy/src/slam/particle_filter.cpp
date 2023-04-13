#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>




ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1),
  rand_device_(),
  generator_(rand_device_()),
  gaussian_pose_sampler(0.1, M_PI / 8.0)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{

    ///////////// DONE : TODO: Implement your method for initializing the particles in the particle filter /////////////////
    // According to page 163 of probabilistic robotics, we should set the distribution to be a
    // narrow Gaussian. Under "Position Tracking" heading.
    // However, Michael did say that initializing all particles at (0, 0, 0) is fine since by
    // definition the bot is known to be exactly at the world origin (since the world origin is
    // initialized based on the bot's first pose).

    // See notes in initializeFilterRandomly to potentially improve computational efficiency of this
    // step.
    posterior_.clear();
    double particle_weight_uniform = 1.0 / static_cast<double>(kNumParticles_);
    for (int i = 0; i < kNumParticles_; ++i)
    {
        // Construct the new particle.
        mbot_lcm_msgs::particle_t particle;
        particle.pose = pose;
        particle.parent_pose = pose;
        particle.weight = particle_weight_uniform;

        posterior_.push_back(particle);
        std::cout << "Initial position x " << pose.x << "Initial position x " << pose.y;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    // TODO(Dylan): Verify this method of random filter initialization is correct.

    // Update the random pose generator map to reflect the given map.
    randomPoseGen.update_map(&map);

    // Clear the particle list and sample new random particles.
    // NOTE: We can do this as the SLAM routines are not run asynchronously and thus we don't have
    //       to worry about another thread accessing the posterior_ ParticleList mid-sampling.
    posterior_.clear();
    double particle_weight_uniform = 1.0 / static_cast<double>(kNumParticles_);
    for (int i = 0; i < kNumParticles_; ++i)
    {
        // This pushing back of the particle and then getting the back of the vector may look
        // inefficient but I believe it avoids an unnecessary copy constructor versus defining a
        // particle variable, changing the weight, then pushing it back
        // (not considering compiler optimization).
        // We could probably squeeze more performance out of this by initializing the vector to the
        // correct size and then assigning the particles, but that may be too much pre-matrure
        // optimization.
        posterior_.push_back(randomPoseGen.get_particle());

        // We also have to be certain to give uniform weights here since the random sampler
        // initializes weight to 0.
        posterior_.back().weight = particle_weight_uniform;
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    // std::cout << "in updateFilter()" << std::endl;
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    // std::cout << "Odometry x: " << odometry.x << std::endl;
    // std::cout << "Odometry y: " << odometry.y << std::endl;
    // std::cout << "Odometry theta: " << odometry.theta << std::endl;

    if(hasRobotMoved)
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


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    // std::cout << "Resampling posterior distribution" << std::endl;

    // Temporarily turning this off!
    bool do_random_sampling = true;
    if (map == nullptr)
    {
        do_random_sampling = false;
    }

    // This method uses the low variance resampling algorithm mentioned in lecture as well as the
    // Probabilistic Robotics textbook.
    ParticleList prior;

    double const M_inverse = 1.0 / static_cast<double>(kNumParticles_);

    // * Setup the augmented sampling for Augmented_MCL algorithm from Table 8.3 in Probabilistic
    //   Robotics.
    // We can say this as the posterior is normalized.
    float const weight_avg = M_inverse;
    // std::cout << "1" << std::endl;
    samplingAugmentation.insert_average_weight(weight_avg);

    // * Setup the random pose generator using the given map.
    // std::cout << "2" << std::endl;
    if (do_random_sampling)
    {
        // Now using the GaussianPoseSampler instead of the RandomPoseGen
        // randomPoseGen.update_map(map);
        // Use the previous posteriorPose estimate to sample from.
        gaussian_pose_sampler.insert_average_pose(posteriorPose_);
    }

    // RNG usage inspired by this reference documentation:
    //   https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
    // std::cout << "3" << std::endl;
    std::uniform_real_distribution<double> distribution(0.0, M_inverse);
    double const r = distribution(generator_);

    // std::cout << "grabbing c" << std::endl;
    double c = posterior_[0].weight;
    int i = 0;
    // std::cout << "doing resampling now" << std::endl;
    for (int m = 0; m < kNumParticles_; ++m)
    {
        double U = r + (m - 1) * M_inverse;
        while (U > c)
        {
            ++i;
            if (i == kNumParticles_)
            {
                std::cout << "WARNING: index exceeded the posterior array size." << std::endl;
                break;
            }
            c += posterior_[i].weight;
        }

        // Decide on whether to sample the posterior randomly or to take the particle that is given
        // by low variance sampling method.
        if (samplingAugmentation.sample_randomly() && do_random_sampling)
        {
            // mbot_lcm_msgs::particle_t rand_particle = randomPoseGen.get_particle();
            // Opt to use the GaussianPoseSampler to get a more realistic sampling of the pose.
            mbot_lcm_msgs::particle_t rand_particle = gaussian_pose_sampler.sample_particle();

            // I'm not really sure what the right weight to use is here but I guess I'll just go
            // with inserting average weight?
            // Not actually sure if it matters as the weight is overridden when using the sensor
            // model.
            // rand_particle.weight = weight_avg;

            prior.push_back(rand_particle);

        }
        else
        {
            prior.push_back(posterior_[i]);
        }

    }
    // TODO(Dylan): Do we need to re-normalize the weights?

    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    // std::cout << "computing proposal distribution" << std::endl;
    // 4.28 from Probabilistic Robotics gives the proposal distribution is:
    //     p(x_t | u_t, x_{t-1}) * bel(x_{t-1})
    // where bel(x_{t-1}) is just the prior and thus p(x_t | u_t, x_{t-1}) is the sampling from the
    // action model.

    // Is this method really this easy?
    ParticleList proposal;
    for (auto const& particle_belief_prior : prior)
    {
        // NOTE: Assumes that the proper weight is set in the ActionModel::applyAction() function!!
        mbot_lcm_msgs::particle_t particle_sampled =
            actionModel_.applyAction(particle_belief_prior);

        proposal.push_back(particle_sampled);
    }
    return proposal;
}

// Compute the normalized posterior distribution using the particles in the proposal distribution.
ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    // Look at 4.34 in Probabilistic Robotics for this.

    // Get the initial unnormalized weights by using the sensor model to calculate the likelihood
    // of each particle's pose given the map and the most recent laser scan.
    // It stands to reason that a how much a particle contributes to the pose estimation should be
    // proportional to how well that particle matches up with the observations we make!
    ParticleList posterior;
    double weight_sum = 0.0;
    // std::cout << "Particle likelihoods taken from sensor model: ";
    for (auto const& particle_proposed : proposal)
    {
        // Weight is given by p( <measurement> | particle_proposed ) which is the sensor model
        // likelihood function.
        double const weight_posterior_unnormalized =
            sensorModel_.likelihood(particle_proposed, laser, map);

        // std::cout << weight_posterior_unnormalized << ", ";

        weight_sum += weight_posterior_unnormalized;

        mbot_lcm_msgs::particle_t particle_posterior = particle_proposed;
        particle_posterior.weight = weight_posterior_unnormalized;

        posterior.push_back(particle_posterior);
    }
    // std::cout << std::endl;

    // Normalize the weights.
    for (auto& particle_posterior : posterior)
    {
        particle_posterior.weight /= weight_sum;
    }

    return posterior;
}

// I believe particle reinvigoration should be done in the resampling of the posterior step.
// void ParticleFilter::reinvigoratePosterior()
// {
//     // NOTE: Assuming called after the normalize function!
//     // Compute the average weight
//     float weight_avg = 1.0 / static_cast<float>(kNumParticles_);

//     // Setup the augmented sampling.
//     samplingAugmentation.insert_average_weight(weight_avg);

//     double weight_sum = 0.0;
//     for (auto it = posterior_.begin(); it != posterior_.end(); ++it)
//     {
//         if (samplingAugmentation.sample_randomly())
//         {
//             // Replace the particle with a random pose.
//         }
//     }

//     // Renormalize the distribution.

// }

mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    // Construct a weights list for sorting and populate it with the particle weights.
    std::vector<double> weights(kNumParticles_, 0.0);
    int i = 0;
    // std::cout << "Normalized posterior weights: ";
    double weight_sum = 0.0;
    for (auto it = posterior.begin(); it != posterior.end(); ++it)
    {
        // std::cout << it->weight << ", ";
        weights[i] = it->weight;
        weight_sum += it->weight;
        ++i;
    }
    // std::cout << std::endl;
    // std::cout << "Posterior weight total post-normalization: " << weight_sum << std::endl;

    // Sort the weights. NOTE: This is in ascending order!
    std::sort(weights.begin(), weights.end());

    // Find the weight corresponding to the top X%. Since the weights vector is sorted in ascending
    // order we need to grab particles from the end of the vector (the particles with the highest 
    // weights).
    int num_particles_to_take =
        std::round(PERCENT_PARTICLES_TO_AVERAGE * static_cast<double>(kNumParticles_));
    double weight_threshold = weights[weights.size() - 1 - num_particles_to_take];
    // std::cout << "Weight Threshold:\n\t" << weight_threshold << std::endl;

    // Grab all particles greater than or equal to this weight threshold.
    ParticleList particles_to_average;
    weight_sum = 0.0;
    for (auto it = posterior.begin(); it != posterior.end(); ++it)
    {
        if (it->weight >= weight_threshold)
        {
            particles_to_average.push_back(*it);
            weight_sum += it->weight;
        }
    }

    // Normalize the weights of the particles that we're averaging.
    for (auto it = particles_to_average.begin(); it != particles_to_average.end(); ++it)
    {
        it->weight /= weight_sum;
    }

    // std::cout << "Averaging " << particles_to_average.size() << " particles for pose estimation" 
    //     << std::endl;

    // Compute the particle average with computeParticlesAverage()
    mbot_lcm_msgs::pose_xyt_t pose = computeParticlesAverage(particles_to_average);

    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    // NOTE: Don't have to set the utime of the average particle as it's immediately set upon return
    //       in the updateFilter method.
    mbot_lcm_msgs::pose_xyt_t avg_pose;

    // Compute the average angle.
    double weighted_y = 0.0;
    double weighted_x = 0.0;
    for (auto it = particles_to_average.begin(); it != particles_to_average.end(); ++it)
    {
        mbot_lcm_msgs::particle_t const& particle = *it;
        weighted_y += particle.weight * std::sin(particle.pose.theta);
        weighted_x += particle.weight * std::cos(particle.pose.theta);
    }
    avg_pose.theta = std::atan2(weighted_y, weighted_x);

    // Compute the weighted average of the x and y coordinates.
    // If we want, we can wrap the two loops together to make it slightly more efficient.
    avg_pose.x = 0;
    avg_pose.y = 0;
    for (auto it = particles_to_average.begin(); it != particles_to_average.end(); ++it)
    {
        mbot_lcm_msgs::particle_t const& particle = *it;
        avg_pose.x += particle.weight * particle.pose.x;
        avg_pose.y += particle.weight * particle.pose.y;
    }

    return avg_pose;
}
