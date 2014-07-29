#include <benchmarks/KinematicBenchmarks.h>
#include <benchmarks/DataUtils.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

#include <sys/time.h>

using namespace benchmarks;

KinematicBenchmarks::KinematicBenchmarks(OpenRAVE::EnvironmentBasePtr env) 
	: ModuleBase(env), _num_samples(0), _record(false) {

	RegisterCommand("RunForwardKinematics", boost::bind(&KinematicBenchmarks::RunForwardKinematicsBenchmark, this, _1, _2),
					"Run the FK benchmark test");
	RegisterCommand("RunTranslationalJacobian", boost::bind(&KinematicBenchmarks::RunTranslationalJacobianBenchmark, this, _1, _2),
					"Run the translational jacobian benchmark test");
	RegisterCommand("RunRotationalJacobian", boost::bind(&KinematicBenchmarks::RunRotationalJacobianBenchmark, this, _1, _2),
					"Run the rotational jacobian benchmark test");
}

KinematicBenchmarks::~KinematicBenchmarks() {

}

bool KinematicBenchmarks::ParseParameters(std::istream &in){
	YAML::Parser parser(in);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	if(const YAML::Node* pbody = doc.FindValue("robot")){
		*pbody >> _robot_name;
		RAVELOG_INFO("[KinematicBenchmarks] Running benchmark for robot %s\n", _robot_name.c_str());
		_robot = GetEnv()->GetRobot(_robot_name);
		if(!_robot){
			RAVELOG_ERROR("[KinematicBenchmarks] Failed to find body of name %s in environment\n", _robot_name.c_str());
			return false;
		}
	}else{
		RAVELOG_ERROR("[KinematicBenchmarks] Must specify a robot to check FK\n");
		return false;
	}

	if(const YAML::Node* pmanip = doc.FindValue("manip")){
		*pmanip >> _manip_name;
		RAVELOG_INFO("[KinematicBenchmarks] Running benchmark for manip %s\n", _manip_name.c_str());
		_manip = _robot->GetManipulator(_manip_name);
		if(!_manip){
			RAVELOG_ERROR("[KinematicBenchmarks] Failed to find manipulator of name %s on robot\n", _manip_name.c_str());
			return false;
		}
		_robot->SetActiveManipulator(_manip_name);
		_robot->SetActiveDOFs(_manip->GetArmIndices());
	}else{
		RAVELOG_ERROR("[KinematicBenchmarks] Must specify a manipulator to check FK\n");
		return false;
	}


	if(const YAML::Node* prandom = doc.FindValue("random")){
		*prandom >> _num_samples;
	}

	if(const YAML::Node* poutfile = doc.FindValue("outfile")){
		*poutfile >> _outfile;
		_record = true;
	}

	return true;
}

bool KinematicBenchmarks::RunForwardKinematicsBenchmark(std::ostream &out, std::istream &in){

	RAVELOG_DEBUG("[KinematicBenchmarks] Running FK benchmark\n");
	bool success = ParseParameters(in);
	if(!success){
		RAVELOG_ERROR("[KinematicsBenchmarks] Error parsing input parameters.");
		return false;
	}

	std::vector<double> lower;
	std::vector<double> upper;
	_robot->GetActiveDOFLimits(lower, upper);
	std::vector<std::vector<double> > iks = DataUtils::GenerateRandomConfigurations(_num_samples, lower, upper);

	double total_time = 0.0;
	BOOST_FOREACH(std::vector<double> ik, iks){
		
		// Check collision
		struct timeval start, end;
		gettimeofday(&start, NULL);
		_robot->SetActiveDOFValues(ik);
	    _manip->GetEndEffectorTransform();
		gettimeofday(&end, NULL);
		
		float elapsed_s = end.tv_sec - start.tv_sec;
		float elapsed_us = end.tv_usec - start.tv_usec;
		float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
		total_time += elapsed_ms;
	}

	if(_record){
		RAVELOG_INFO("[KinematicBenchmarks] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << iks.size();
		emitter << YAML::EndMap;
		
		std::ofstream out_stream(_outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	total_time /= 1000.0; // swith to seconds for reporting
	RAVELOG_INFO("[KinematicBenchmarks] Results for FK benchmark:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",iks.size());
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/iks.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", iks.size()/total_time);

	return true;
}

bool KinematicBenchmarks::RunTranslationalJacobianBenchmark(std::ostream &out, std::istream &in){

	RAVELOG_DEBUG("[KinematicBenchmarks] Running translational jacobian benchmark\n");
	bool success = ParseParameters(in);
	if(!success){
		RAVELOG_ERROR("[KinematicsBenchmarks] Error parsing input parameters.");
		return false;
	}

	std::vector<double> lower;
	std::vector<double> upper;
	_robot->GetActiveDOFLimits(lower, upper);
	std::vector<std::vector<double> > iks = DataUtils::GenerateRandomConfigurations(_num_samples, lower, upper);

	double total_time = 0.0;
	boost::multi_array<OpenRAVE::dReal, 2> Jtrans;
	BOOST_FOREACH(std::vector<double> ik, iks){
		
		// Check collision
		struct timeval start, end;
		_robot->SetActiveDOFValues(ik);
		gettimeofday(&start, NULL);
		_manip->CalculateJacobian(Jtrans);
		gettimeofday(&end, NULL);
		
		float elapsed_s = end.tv_sec - start.tv_sec;
		float elapsed_us = end.tv_usec - start.tv_usec;
		float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
		total_time += elapsed_ms;
	}

	if(_record){
		RAVELOG_INFO("[KinematicBenchmarks] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << iks.size();
		emitter << YAML::EndMap;
		
		std::ofstream out_stream(_outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	total_time /= 1000.0; // swith to seconds for reporting
	RAVELOG_INFO("[KinematicBenchmarks] Results for translational jacobian benchmark:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",iks.size());
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/iks.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", iks.size()/total_time);

	return true;
}

bool KinematicBenchmarks::RunRotationalJacobianBenchmark(std::ostream &out, std::istream &in){

	RAVELOG_DEBUG("[KinematicBenchmarks] Running rotational jacobian benchmark\n");
	bool success = ParseParameters(in);
	if(!success){
		RAVELOG_ERROR("[KinematicsBenchmarks] Error parsing input parameters.");
		return false;
	}

	std::vector<double> lower;
	std::vector<double> upper;
	_robot->GetActiveDOFLimits(lower, upper);
	std::vector<std::vector<double> > iks = DataUtils::GenerateRandomConfigurations(_num_samples, lower, upper);

	double total_time = 0.0;
	boost::multi_array<OpenRAVE::dReal, 2> Jrot;
	BOOST_FOREACH(std::vector<double> ik, iks){
		
		// Check collision
		struct timeval start, end;
		_robot->SetActiveDOFValues(ik);
		gettimeofday(&start, NULL);
		_manip->CalculateAngularVelocityJacobian(Jrot);
		gettimeofday(&end, NULL);
		
		float elapsed_s = end.tv_sec - start.tv_sec;
		float elapsed_us = end.tv_usec - start.tv_usec;
		float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
		total_time += elapsed_ms;
	}

	if(_record){
		RAVELOG_INFO("[KinematicBenchmarks] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << iks.size();
		emitter << YAML::EndMap;
		
		std::ofstream out_stream(_outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	total_time /= 1000.0; // swith to seconds for reporting
	RAVELOG_INFO("[KinematicBenchmarks] Results for rotational jacobian benchmark:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",iks.size());
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/iks.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", iks.size()/total_time);

	return true;
}