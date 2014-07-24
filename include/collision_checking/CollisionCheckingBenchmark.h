#ifndef CC_BENCHMARK_H_
#define CC_BENCHMARK_H_

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

namespace collision_checking {

	/**
	 * Type of data for environment collision checking
	 */
	class CollisionData {
	public:
	CollisionData() : elapsed(0.0), collision(true) {}
	CollisionData(const OpenRAVE::Transform &pt, const double &elapsed, const bool &collision)
		: pt(pt), elapsed(elapsed), collision(collision) {}
		
		OpenRAVE::Transform pt;
		double elapsed;
		bool collision;
	};

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
		
		return emitter;
	}

	/**
	 * Read a rave transform from file
	 * 
	 * @param node The YAML node to parse
	 * @param transform The filled transform
	 */
	inline void operator >> (const YAML::Node& node, OpenRAVE::Transform &transform) {
		std::vector<double> vals;
		node >> vals;

		if(vals.size() == 7){
			transform.trans.x = vals[0];
			transform.trans.y = vals[1];
			transform.trans.z = vals[2];
			transform.rot.x = vals[3];
			transform.rot.y = vals[4];
			transform.rot.z = vals[5];
			transform.rot.w = vals[6];
		}else{
			RAVELOG_ERROR("[CollisionCheckingBenchmark] Failed to load transform. Got %d values, expected 7.", vals.size());
		}
	}

	/**
	 * Write a collision data point to yaml
	 *
	 * @param emitter The YAML emitter to write to
	 * @param transform The transform to write
	 */
	inline YAML::Emitter& operator << (YAML::Emitter &emitter, const CollisionData &data){
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "transform" << YAML::Value << data.pt;
		emitter << YAML::Key << "elapsed" << YAML::Value << data.elapsed;
		emitter << YAML::Key << "collision" << YAML::Value << data.collision;
 		emitter << YAML::EndMap;
		
		return emitter;
	}

	/**
	 * Read a CollisionData point from yaml
	 * 
	 * @param node The YAML node to parse
	 * @param pt The filled pt
	 */
	inline void operator >> (const YAML::Node& node, CollisionData &pt) {
		node["transform"] >> pt.pt;
		node["elapsed"] >> pt.elapsed;
		node["collision"] >> pt.collision;
	}

	/**
	 * Type of data for self collision checking
	 */
	class SelfCollisionData {
	public:
	SelfCollisionData() : elapsed(0.0), collision(true) {}
		SelfCollisionData(const std::vector<double> &pt, const double &elapsed, const bool &collision)
		: pt(pt), elapsed(elapsed), collision(collision) {}
		std::vector<double> pt;
		double elapsed;
		bool collision;
	};

	/**
	 * Write a self collision data point to yaml
	 *
	 * @param emitter The YAML emitter to write to
	 * @param transform The transform to write
	 */
	inline YAML::Emitter& operator << (YAML::Emitter &emitter, const SelfCollisionData &data){
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "pose" << YAML::Value << YAML::Flow << data.pt;
		emitter << YAML::Key << "elapsed" << YAML::Value << data.elapsed;
		emitter << YAML::Key << "collision" << YAML::Value << data.collision;
 		emitter << YAML::EndMap;

		return emitter;
	}

	/**
	 * Read a SelfCollisionData point from yaml
	 * 
	 * @param node The YAML node to parse
	 * @param pt The filled pt
	 */
	inline void operator >> (const YAML::Node& node, SelfCollisionData &pt) {
		node["pose"] >> pt.pt;
		node["elapsed"] >> pt.elapsed;
		node["collision"] >> pt.collision;
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

		double _extent;

		std::string _datafile;
		unsigned int _num_samples;

		bool _record;
		std::string _outfile;
	};
}

#endif
