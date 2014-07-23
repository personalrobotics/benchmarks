#ifndef CC_BENCHMARK_H_
#define CC_BENCHMARK_H_

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

namespace collision_checking {

	/**
	 * Write a RaveTransform to yaml
	 *
	 * @param emitter The YAML emitter to write to
	 * @param transform The transform to write
	 */
	inline YAML::Emitter& operator << (YAML::Emitter &emitter, const OpenRAVE::Transform &transform){
		std::vector<double> vals;
		vals.push_back(transform.trans.x);
		vals.push_back(transform.trans.y);
		vals.push_back(transform.trans.z);
		vals.push_back(transform.rot.x);
		vals.push_back(transform.rot.y);
		vals.push_back(transform.rot.z);
		vals.push_back(transform.rot.w);

		emitter << YAML::Flow << vals;
	}

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

		double _duration;
		double _extent;

		bool _record;
		std::string _outfile;
	};
}

#endif
