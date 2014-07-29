#ifndef DATAUTILS_H_
#define DATAUTILS_H_

#include <benchmarks/DataTypes.h>
#include <openrave/openrave.h>

namespace benchmarks {

	class DataUtils {

	public:
		/**
		 * Generates a set of random transforms
		 * 
		 * @param num_samples The number of transforms to generate
		 * @param bounds Draw uniformly from within these bounds
		 * @return A list of randomly generated transforms
		 */
		static std::vector<OpenRAVE::Transform> GenerateRandomTransforms(const unsigned int &num_samples, const BoundingBox &bounds);

		/**
		 * Generates a set of random configurations
		 * 
		 * @param num_samples The number of random configurations to generate
		 * @param lower The lower bounds for each DOF
		 * @param upper The upper bounds for each DOF
		 * @return A list of randomly generated configurations
		 */
		static std::vector<std::vector<double> > GenerateRandomConfigurations(const unsigned int &num_samples, 
																			  const std::vector<double> &lower,
																			  const std::vector<double> &upper);

		/**
		 * Update the variance using the new data point
		 *
		 * @param data_pt The new data point to include in the mean
		 * @param mean The mean for the first n-1 datapoints - this value will be updated by the function to contain
		 *   the mean for the first n datapoints
		 * @param mean_sq_dist The sum squared dist from the mean for the first n-1 datapoints - this value will be
		 *  updated by the function to contain the sum squared dist from the mean for the first n datapoints
		 * @param n The number of points included in the mean calculation, including this one (must be >=1)
		 * @return An update variance
		 */
		static double UpdateVariance(const double &data_pt,
									 double &mean,
									 double &mean_sq_dist,
									 const unsigned int &n);
	};
}

#endif
