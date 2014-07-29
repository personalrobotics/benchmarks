#include <benchmarks/DataUtils.h>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>

using namespace benchmarks;

std::vector<OpenRAVE::Transform> DataUtils::GenerateRandomTransforms(const unsigned int &num_samples, const BoundingBox &bounds) {

	std::vector<OpenRAVE::Transform> data;

	// Setup random number generators
	typedef boost::uniform_real<> NumberDistribution;
	typedef boost::mt19937 RandomNumberGenerator;
	typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

	RandomNumberGenerator generator;
	generator.seed(std::time(0));

	// Generate random number generators for each parameter
	NumberDistribution xdistribution(bounds.xmin, bounds.xmax);
	Generator xgenerator(generator, xdistribution);

	NumberDistribution ydistribution(bounds.ymin, bounds.ymax);
	Generator ygenerator(generator, ydistribution);

	NumberDistribution zdistribution(bounds.zmin, bounds.zmax);
	Generator zgenerator(generator, xdistribution);

	NumberDistribution ang_distribution(0., 2.*boost::math::constants::pi<double>());
	Generator ang_generator(generator, ang_distribution);

	RAVELOG_INFO("[CollisionCheckingBenchmark] Generating %d transforms\n", num_samples);
	for(unsigned int idx=0; idx < num_samples; idx++){
		OpenRAVE::Transform trans;
		trans.trans.x = xgenerator();
		trans.trans.y = ygenerator();
		trans.trans.z = zgenerator();
			
		OpenRAVE::RaveVector<OpenRAVE::dReal> rand_axis_angle(ang_generator(), ang_generator(), ang_generator());
		trans.rot = OpenRAVE::geometry::quatFromAxisAngle(rand_axis_angle);

		data.push_back(trans);
	}

	return data;

}

std::vector<std::vector<double> > DataUtils::GenerateRandomConfigurations(const unsigned int &num_samples, 
																				 const std::vector<double> &lower,
																				 const std::vector<double> &upper){

	std::vector<std::vector<double> > data;

	typedef boost::uniform_real<> NumberDistribution;
	typedef boost::mt19937 RandomNumberGenerator;
	typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;
		
	RandomNumberGenerator generator;
	generator.seed(std::time(0));

	RAVELOG_INFO("[CollisionCheckingBenchmark] Generating %d poses\n", num_samples);
	for(unsigned int idx=0; idx < num_samples; idx++){
		// Sample a random pose
		std::vector<double> pose;
		for(unsigned int idx=0; idx < lower.size(); idx++){
			NumberDistribution distribution(lower[idx], upper[idx]);
			Generator number_generator(generator, distribution);
			pose.push_back(number_generator());
		}
			
		data.push_back(pose);
	}

	return data;
}

double DataUtils::UpdateVariance(const double &data_pt,
								 double &mean,
								 double &mean_sq_dist,
								 const unsigned int &n) {

	if(n < 1){
		RAVELOG_ERROR("[DataUtils] UpdateVariance - n must be greater or equal to 1 - returning invalid value\n");
		return 0.0;
	}

	if(n < 2){
		// No variance yet...
		return 0.0;
	}
	
	double delta = data_pt - mean;
	mean += delta/n;

	mean_sq_dist += delta*(data_pt - mean);
	
	return (mean_sq_dist / (n - 1.0));
}

double DataUtils::ComputeElapsedMilliseconds(const struct timeval &start, const struct timeval &end) {

	float elapsed_s = end.tv_sec - start.tv_sec;
	float elapsed_us = end.tv_usec - start.tv_usec;
	float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
	
	return elapsed_ms;
}
