#ifndef YAML_UTILS_H_
#define YAML_UTILS_H_

#include <benchmarks/DataTypes.h>

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

namespace benchmarks {

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
	 * @param data The CollisionData object to write
	 */
	inline YAML::Emitter& operator << (YAML::Emitter &emitter, const CollisionData &data){
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "transform" << YAML::Value << data.pt;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << data.elapsed;
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
		node["elapsed_ms"] >> pt.elapsed;
		node["collision"] >> pt.collision;
	}

	/**
	 * Write a self collision data point to yaml
	 *
	 * @param emitter The YAML emitter to write to
	 * @param data The SelfCollisionData object to write
	 */
	inline YAML::Emitter& operator << (YAML::Emitter &emitter, const SelfCollisionData &data){
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "pose" << YAML::Value << YAML::Flow << data.pt;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << data.elapsed;
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
		node["elapsed_ms"] >> pt.elapsed;
		node["collision"] >> pt.collision;
	}

}

#endif
