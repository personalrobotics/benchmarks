#include <collision_checking/CollisionCheckingBenchmark.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

using namespace collision_checking;

CollisionCheckingBenchmark::CollisionCheckingBenchmark(OpenRAVE::EnvironmentBasePtr env) 
	: ModuleBase(env), _duration(0.0), _extent(0.0), _record(false) {

	RegisterCommand("Run", boost::bind(&CollisionCheckingBenchmark::RunBenchmark, this, _1, _2),
					"Run the benchmark test");
}

CollisionCheckingBenchmark::~CollisionCheckingBenchmark() {

}

bool CollisionCheckingBenchmark::RunBenchmark(std::ostream &out, std::istream &in){


	RAVELOG_DEBUG("[CollisionCheckingBenchmark] Running benchmark\n");

	YAML::Parser parser(in);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	if(const YAML::Node* pbody = doc.FindValue("body")){
		*pbody >> _body_name;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Running collision checking for body %s\n", _body_name.c_str());
		_body = GetEnv()->GetKinBody(_body_name);
		if(!_body){
			RAVELOG_ERROR("[CollisionCheckingBenchmark] Failed to find body of name %s in environment\n", _body_name.c_str());
			return false;
		}
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify a kinbody to collision check\n");
		return false;
	}


	if(const YAML::Node* pduration = doc.FindValue("duration")){
		*pduration >> _duration;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Running collision checking for %0.3f seconds\n", _duration);
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify a duration\n");
		return false;
	}	


	if(const YAML::Node* pextent = doc.FindValue("extent")){
		*pextent >> _extent;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Sampling poses from cube with extent %0.3f\n", _extent);
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify extents of the area for pose sampling.\n");
		return false;
	}	

	if(const YAML::Node* poutfile = doc.FindValue("outfile")){
		*poutfile >> _outfile;
		_record = true;
	}

	if(const YAML::Node* pself = doc.FindValue("self")){
		return RunSelfCollisionBenchmark();
	}else{
		return RunCollisionBenchmark();
	}

}

bool CollisionCheckingBenchmark::RunCollisionBenchmark() {

	// Setup random number generators
	typedef boost::uniform_real<> NumberDistribution;
	typedef boost::mt19937 RandomNumberGenerator;
	typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

	NumberDistribution distribution(0., _extent);
	RandomNumberGenerator generator;
	Generator number_generator(generator, distribution);
	generator.seed(std::time(0));

	NumberDistribution adistribution(0., 2.*boost::math::constants::pi<double>());
	Generator angle_generator(generator, adistribution);

	double total_time = 0.0;
	typedef std::pair<OpenRAVE::Transform, double> BenchmarkData;
	std::vector<BenchmarkData> data;
	OpenRAVE::EnvironmentBasePtr env = GetEnv();
	int collisions = 0;
	while(total_time < _duration){
		// Sample a random pose
		OpenRAVE::Transform btrans = _body->GetTransform();
		btrans.trans.x = number_generator();
		btrans.trans.y = number_generator();
		btrans.trans.z = number_generator();

		OpenRAVE::RaveVector<OpenRAVE::dReal> rand_axis_angle(angle_generator(), angle_generator(), angle_generator());
		btrans.rot = OpenRAVE::geometry::quatFromAxisAngle(rand_axis_angle);

		// Set the transform
		_body->SetTransform(btrans);

		// Check collision
		boost::timer t;
		bool incollision = env->CheckCollision(_body);
		double elapsed = t.elapsed();
		total_time += elapsed;

		if(incollision)
			collisions++;

		// Save the data
		data.push_back(std::make_pair(btrans, elapsed));
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << data.size();
		emitter << YAML::Key << "data" << YAML::Value;
		emitter << YAML::BeginSeq;
		BOOST_FOREACH(BenchmarkData pt, data){
			emitter << YAML::BeginMap;
			emitter << YAML::Key << "pose" << YAML::Value << pt.first;
			emitter << YAML::Key << "duration" << YAML::Value << pt.second;
			emitter << YAML::EndMap;
		}
		emitter << YAML::EndSeq;
		emitter << YAML::EndMap;
		
		std::ofstream out_stream(_outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	RAVELOG_INFO("[CollisionCheckingBenchmark] Results:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",data.size());
	RAVELOG_INFO("\tTotal collisions found: %d\n",collisions);
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/data.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", data.size()/total_time);
	
	return true;
}

bool CollisionCheckingBenchmark::RunSelfCollisionBenchmark() {

	// Get the boundaries on the arm
	std::vector<OpenRAVE::dReal> lower;
	std::vector<OpenRAVE::dReal> upper;
	_body->GetDOFLimits(lower, upper);

	typedef boost::uniform_real<> NumberDistribution;
	typedef boost::mt19937 RandomNumberGenerator;
	typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

	RandomNumberGenerator generator;
	generator.seed(std::time(0));

	double total_time = 0.0;
	typedef std::pair<std::vector<double>, double> BenchmarkData;
	std::vector<BenchmarkData> data;
	int collisions = 0;
	while(total_time < _duration){
		// Sample a random pose
		std::vector<double> pose;
		for(unsigned int idx=0; idx < lower.size(); idx++){
			NumberDistribution distribution(lower[idx], upper[idx]);
			Generator number_generator(generator, distribution);
			pose.push_back(number_generator());
		}

		// Set the transform
		_body->SetDOFValues(pose);

		// Check collision
		boost::timer t;
		bool incollision = _body->CheckSelfCollision();
		double elapsed = t.elapsed();
		total_time += elapsed;

		if(incollision)
			collisions++;

		// Save the data
		data.push_back(std::make_pair(pose, elapsed));
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << data.size();
		emitter << YAML::Key << "data" << YAML::Value;
		emitter << YAML::BeginSeq;
		BOOST_FOREACH(BenchmarkData pt, data){
			emitter << YAML::BeginMap;
			emitter << YAML::Key << "pose" << YAML::Value << YAML::Flow << pt.first;
			emitter << YAML::Key << "duration" << YAML::Value << pt.second;
			emitter << YAML::EndMap;
		}
		emitter << YAML::EndSeq;
		emitter << YAML::EndMap;
		
		std::ofstream out_stream(_outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	RAVELOG_INFO("[CollisionCheckingBenchmark] Results:\n");
	RAVELOG_INFO("\tTotal self collision checks: %d\n",data.size());
	RAVELOG_INFO("\tTotal collisions found: %d\n",collisions);
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/data.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", data.size()/total_time);

	return true;
}
