#ifndef KINEMATIC_BENCHMARK_H_
#define KINEMATIC_BENCHMARK_H_

#include <benchmarks/YamlUtils.h>
#include <benchmarks/DataTypes.h>

#include <openrave/openrave.h>
#include <yaml-cpp/yaml.h>

/**
 * Namespace for all OpenRAVE benchmark tests
 */
namespace benchmarks {

	/**
	 * This module exposes kinematics related benchmark code as a plugin to OpenRAVE
	 */
	class KinematicBenchmarks : public OpenRAVE::ModuleBase {

	public:
		/**
		 * Constructor
		 * @param env The OpenRAVE environment to run the benchmark against
		 */
		KinematicBenchmarks(OpenRAVE::EnvironmentBasePtr env);

		/**
		 * Destructor
		 */
		virtual ~KinematicBenchmarks();

	private:
		/**
		 * Parser parameters from the input stream
		 */
		bool ParseParameters(std::istream &in);

		/**
		 * Execute the forward kinematics benchmark - sample several arm configurations
		 *  and compute end-effector transform for the arm
		 */
		bool RunForwardKinematicsBenchmark(std::ostream &out, std::istream &in);

		/**
		 * Execute the jacobian benchmark - sample an arm configuration and compute the translational and rotational jacobian
		 */
		bool RunJacobianBenchmark(std::ostream &out, std::istream &in);

		std::string _robot_name;
		OpenRAVE::RobotBasePtr _robot;
		
		std::string _manip_name;
		OpenRAVE::RobotBase::ManipulatorPtr _manip;

		unsigned int _num_samples;

		bool _record;
		std::string _outfile;
	};
}

#endif
