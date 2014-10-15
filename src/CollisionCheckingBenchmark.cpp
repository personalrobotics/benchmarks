#include <benchmarks/CollisionCheckingBenchmark.h>
#include <benchmarks/DataUtils.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>
//#include <google/profiler.h>

#include <sys/time.h>

using namespace benchmarks;

CollisionCheckingBenchmark::CollisionCheckingBenchmark(OpenRAVE::EnvironmentBasePtr env) 
	: ModuleBase(env), _num_samples(0), _record(false) {

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
		double extent;
		*pextent >> extent;
		_bounds.set(0, extent, 0, extent, 0, extent);
		RAVELOG_INFO("[CollisionCheckingBenchmark] Sampling poses from cube with extent %0.3f\n", extent);
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
		data = DataUtils::GenerateRandomTransforms(_num_samples, _bounds);
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

		RAVELOG_INFO("[CollisionCheckingBenchmark] Loaded %d transforms\n", data.size());
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
		
		data = DataUtils::GenerateRandomConfigurations(_num_samples, lower, upper);

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

		RAVELOG_INFO("[CollisionCheckingBenchmark] Loaded %d poses\n", data.size());

	}

	return data;
}


bool CollisionCheckingBenchmark::RunCollisionBenchmark() {

	// Grab data
	std::vector<OpenRAVE::Transform> data = GenerateCollisionData();

	// Initialize values for calculating variance online
	double mean = 0.0;
	double mean_sq_dist = 0.0;
	double variance = 0.0;

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
		struct timeval start, end;
		gettimeofday(&start, NULL);

        //ProfilerStart("CheckCollision");
		bool incollision = env->CheckCollision(_body);
        //ProfilerStop();

		gettimeofday(&end, NULL);

		float elapsed_s = end.tv_sec - start.tv_sec;
		float elapsed_us = end.tv_usec - start.tv_usec;
		float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
		total_time += elapsed_ms;

		variance = DataUtils::UpdateVariance(elapsed_ms, mean, mean_sq_dist, idx+1);

		if(incollision)
			collisions++;

		// Save the data
		CollisionData pt(btrans, elapsed_ms, incollision);
		collision_data.push_back(pt);
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << total_time;
		emitter << YAML::Key << "checks" << YAML::Value << data.size();
		emitter << YAML::Key << "mean_ms" << YAML::Value << mean;
		emitter << YAML::Key << "variance_ms" << YAML::Value << variance;
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

	total_time /= 1000.0; // swith to seconds for reporting
	RAVELOG_INFO("[CollisionCheckingBenchmark] Results:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",data.size());
	RAVELOG_INFO("\tTotal collisions found: %d\n",collisions);
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/data.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", data.size()/total_time);
	
	return true;
}

bool CollisionCheckingBenchmark::RunSelfCollisionBenchmark() {

    OpenRAVE::EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

	std::vector<std::vector<double> > data = GenerateSelfCollisionData();

	double total_time = 0.0;
	std::vector<SelfCollisionData> collision_data;
	int collisions = 0;

	for(unsigned int idx=0; idx < data.size(); idx++){
		std::vector<double> pose = data[idx];

		// Set the transform
		_body->SetDOFValues(pose);

		// Check collision
		struct timeval start, end;
		gettimeofday(&start, NULL);
        
		bool incollision = _body->CheckSelfCollision();

		gettimeofday(&end, NULL);

		float elapsed_s = end.tv_sec - start.tv_sec;
		float elapsed_us = end.tv_usec - start.tv_usec;
		float elapsed_ms = elapsed_s*1000.0 + elapsed_us/1000.0;
		total_time += elapsed_ms;

		if(incollision)
			collisions++;

		// Save the data
		SelfCollisionData pt(pose, elapsed_ms, incollision);
		collision_data.push_back(pt);
	}

	if(_record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", _outfile.c_str());
		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "elapsed_ms" << YAML::Value << total_time;
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

	total_time /= 1000.0;
	RAVELOG_INFO("[CollisionCheckingBenchmark] Results:\n");
	RAVELOG_INFO("\tTotal self collision checks: %d\n",data.size());
	RAVELOG_INFO("\tTotal collisions found: %d\n",collisions);
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/data.size());
	RAVELOG_INFO("\tAverage checks per second: %0.2f\n", data.size()/total_time);

	return true;
}
