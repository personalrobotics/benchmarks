#include <collision_checking/CollisionCheckingBenchmark.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

using namespace collision_checking;

CollisionCheckingBenchmark::CollisionCheckingBenchmark(OpenRAVE::EnvironmentBasePtr env) 
	: ModuleBase(env), _extent(0.0), _num_samples(0), _record(false) {

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


	if(const YAML::Node* prandom = doc.FindValue("random")){
		*prandom >> _num_samples;
	}

	if(const YAML::Node* pdatafile = doc.FindValue("datafile")){
		*pdatafile >> _datafile;
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

	if(doc.FindValue("self")){
		return RunSelfCollisionBenchmark();
	}else{
		return RunCollisionBenchmark();
	}

}

std::vector<OpenRAVE::Transform> CollisionCheckingBenchmark::GenerateCollisionData() const {

	std::vector<OpenRAVE::Transform> data;
	if(_datafile.length() == 0){

		// Setup random number generators
		typedef boost::uniform_real<> NumberDistribution;
		typedef boost::mt19937 RandomNumberGenerator;
		typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

		RandomNumberGenerator generator;
		generator.seed(std::time(0));

		NumberDistribution pos_distribution(0., _extent);
		Generator pos_generator(generator, pos_distribution);

		NumberDistribution ang_distribution(0., 2.*boost::math::constants::pi<double>());
		Generator ang_generator(generator, ang_distribution);

		RAVELOG_INFO("[CollisionCheckingBenchmark] Generating %d transforms\n", _num_samples);
		for(unsigned int idx=0; idx < _num_samples; idx++){
			OpenRAVE::Transform trans;
			trans.trans.x = pos_generator();
			trans.trans.y = pos_generator();
			trans.trans.z = pos_generator();
			
			OpenRAVE::RaveVector<OpenRAVE::dReal> rand_axis_angle(ang_generator(), ang_generator(), ang_generator());
			trans.rot = OpenRAVE::geometry::quatFromAxisAngle(rand_axis_angle);

			data.push_back(trans);
		}

	}else{

		RAVELOG_INFO("[CollisionCheckingBenchmark] Loading transforms from file %s\n", _datafile.c_str());
		
		// Open the yaml file for reading
		std::ifstream in_stream(_datafile.c_str(), std::ofstream::binary);

		YAML::Parser parser(in_stream);
		YAML::Node doc;
		parser.GetNextDocument(doc);

		std::vector<CollisionData> cdata;
		doc["data"] >> cdata;

		BOOST_FOREACH(CollisionData pt, cdata){
			data.push_back(pt.pt);
		}

		RAVELOG_INFO("[CollisionCheckingBenchmark] Laoded %d transforms\n", data.size());
	}

	return data;
}

std::vector<std::vector<double> > CollisionCheckingBenchmark::GenerateSelfCollisionData() const {

	std::vector<std::vector<double> > data;
	if(_datafile.length() == 0){
		// Get the boundaries on the arm
		std::vector<OpenRAVE::dReal> lower;
		std::vector<OpenRAVE::dReal> upper;
		_body->GetDOFLimits(lower, upper);
		
		typedef boost::uniform_real<> NumberDistribution;
		typedef boost::mt19937 RandomNumberGenerator;
		typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;
		
		RandomNumberGenerator generator;
		generator.seed(std::time(0));

		RAVELOG_INFO("[CollisionCheckingBenchmark] Generating %d poses\n", _num_samples);
		for(unsigned int idx=0; idx < _num_samples; idx++){
			// Sample a random pose
			std::vector<double> pose;
			for(unsigned int idx=0; idx < lower.size(); idx++){
				NumberDistribution distribution(lower[idx], upper[idx]);
				Generator number_generator(generator, distribution);
				pose.push_back(number_generator());
			}
			
			data.push_back(pose);
		}

	}else{
		RAVELOG_INFO("[CollisionCheckingBenchmark] Loading poses from file %s\n", _datafile.c_str());
		
		// Open the yaml file for reading
		std::ifstream in_stream(_datafile.c_str(), std::ofstream::binary);

		YAML::Parser parser(in_stream);
		YAML::Node doc;
		parser.GetNextDocument(doc);

		std::vector<SelfCollisionData> cdata;
		doc["data"] >> cdata;

		BOOST_FOREACH(SelfCollisionData pt, cdata){
			data.push_back(pt.pt);
		}

		RAVELOG_INFO("[CollisionCheckingBenchmark] Laoded %d poses\n", data.size());

	}

	return data;
}


bool CollisionCheckingBenchmark::RunCollisionBenchmark() {

	// Grab data
	std::vector<OpenRAVE::Transform> data = GenerateCollisionData();

	double total_time = 0.0;
	std::vector<CollisionData> collision_data;
	OpenRAVE::EnvironmentBasePtr env = GetEnv();
	int collisions = 0;
	for(unsigned int idx=0; idx < data.size(); idx++){

		// Grab the next pose
		OpenRAVE::Transform btrans = data[idx];

		// Set the transform
		_body->SetTransform(btrans);

		// Check collision
		boost::timer t;
		bool incollision = env->CheckCollision(_body);
		float elapsed = t.elapsed();
		total_time += elapsed;

		if(incollision)
			collisions++;

		// Save the data
		CollisionData pt(btrans, elapsed*1000.0, incollision);
		collision_data.push_back(pt);
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << data.size();
		emitter << YAML::Key << "data" << YAML::Value;
		emitter << YAML::BeginSeq;
		BOOST_FOREACH(CollisionData pt, collision_data){
			emitter << pt;
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

	std::vector<std::vector<double> > data = GenerateSelfCollisionData();

	double total_time = 0.0;
	std::vector<SelfCollisionData> collision_data;
	int collisions = 0;

	for(unsigned int idx=0; idx < data.size(); idx++){
		std::vector<double> pose = data[idx];

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
		SelfCollisionData pt(pose, elapsed, incollision);
		collision_data.push_back(pt);
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << data.size();
		emitter << YAML::Key << "data" << YAML::Value;
		emitter << YAML::BeginSeq;
		BOOST_FOREACH(SelfCollisionData pt, collision_data){
			emitter << pt;
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
