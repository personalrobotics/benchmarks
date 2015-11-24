#ifndef CC_BENCHMARK_H_
#define CC_BENCHMARK_H_

#include <benchmarks/YamlUtils.h>
#include <benchmarks/DataTypes.h>

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

/**
 * Namespace for all OpenRAVE benchmark tests
 */
namespace benchmarks {

	/**
	 * This module exposes the collision checking benchmark code as a plugin to OpenRAVE
	 */
	class CollisionCheckingBenchmark : public OpenRAVE::ModuleBase {

	public:
		/**
		 * Constructor
		 * @param env The OpenRAVE environment to run the benchmark against
		 */
		CollisionCheckingBenchmark(OpenRAVE::EnvironmentBasePtr env);

		/**
		 * Destructor
		 */
		virtual ~CollisionCheckingBenchmark();

	private:

		/**
		 * Generates a data set for environment collision checking
		 */
		std::vector<OpenRAVE::Transform> GenerateCollisionData() const;
		
		/**
		 * Generates a data set for self collision checking
		 */
		std::vector<std::vector<double> > GenerateSelfCollisionData() const;

		/**
		 * Parse parameters and execute the appropriate test
		 */
		bool RunBenchmark(std::ostream &out, std::istream &in);
		/**
		 * Execute the collision checking benchmark
		 */
		bool RunCollisionBenchmark();

		/**
		 * Execute the self collision checking benchmark
		 */
		bool RunSelfCollisionBenchmark();

		std::string _body_name;
		OpenRAVE::KinBodyPtr _body;

		BoundingBox _bounds;

		std::string _datafile;
		unsigned int _num_samples;

		bool _record;
		std::string _outfile;
	};
}

#endif
