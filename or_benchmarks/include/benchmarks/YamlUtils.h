#ifndef YAML_UTILS_H_
#define YAML_UTILS_H_

#include <benchmarks/DataTypes.h>

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

namespace YAML {
    
    
    template<>
        struct convert<OpenRAVE::Transform> {

        /**
         * Write a RaveTransform to a YAML Node
         *
         * @param transform The transform to write
         */
        static Node encode(const OpenRAVE::Transform &transform){
            Node node;
            node.push_back(transform.trans.x);
            node.push_back(transform.trans.y);
            node.push_back(transform.trans.z);
            node.push_back(transform.rot.x);
            node.push_back(transform.rot.y);
            node.push_back(transform.rot.z);
            node.push_back(transform.rot.w);
            return node;
        }
    
        /**
         * Read a rave transform from file
         * 
         * @param node The YAML node to parse
         * @param transform The filled transform
         */
        static bool decode(const Node& node, OpenRAVE::Transform &transform){
            if(node.size() != 7){
                RAVELOG_ERROR("[YamlUtils] Failed to load transform. Got %d values, expected 7.", node.size());
                return false;
            }
            
            transform.trans.x = node[0].as<double>();
            transform.trans.y = node[1].as<double>();
            transform.trans.z = node[2].as<double>();
            transform.rot.x = node[3].as<double>();
            transform.rot.y = node[4].as<double>();
            transform.rot.z = node[5].as<double>();
            transform.rot.w = node[6].as<double>();
            return true;
        }
    };

    template<>
        struct convert<benchmarks::CollisionData> {

        /**
         * Write a collision data point to yaml
         *
         * @param data The CollisionData object to write
         */
        static Node encode(const benchmarks::CollisionData &data) {
            Node node;
            node["transform"] = data.pt;
            node["elapsed_ms"] = data.elapsed;
            node["collision"] = data.collision;
            return node;
        }

        /**
         * Read a CollisionData point from yaml
         * 
         * @param node The YAML node to parse
         * @param pt The filled pt
         */
        static bool decode(const Node& node, benchmarks::CollisionData &pt) {
            pt.pt = node["transform"].as<OpenRAVE::Transform>();
            pt.elapsed = node["elapsed_ms"].as<double>();
            pt.collision = node["collision"].as<bool>();
            return true;
        }
    };

    template<>
        struct convert<benchmarks::SelfCollisionData> {

        /**
         * Write a self collision data point to yaml
         *
         * @param data The SelfCollisionData object to write
         */
        static Node encode(const benchmarks::SelfCollisionData &data) {
            Node node;
            for(unsigned int idx=0; idx < data.pt.size(); idx++){
                node["pose"].push_back(data.pt[idx]);
            }
            node["elapsed_ms"] = data.elapsed;
            node["collision"] = data.collision;
            return node;
        }

        /**
         * Read a SelfCollisionData point from yaml
         * 
         * @param node The YAML node to parse
         * @param pt The filled pt
         */
        static bool decode(const Node& node, benchmarks::SelfCollisionData &pt) {
            pt.pt.resize(node["pose"].size());
            for(unsigned int idx=0; idx < pt.pt.size(); idx++){
                pt.pt[idx] = node["pose"][idx].as<double>();
            }
            pt.elapsed = node["elapsed_ms"].as<double>();
            pt.collision = node["collision"].as<bool>();
            return true;
        }
    };
}

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
    
}

#endif
