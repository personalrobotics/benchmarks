#include <collision_checking/CollisionCheckingBenchmark.h>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

using namespace collision_checking;

CollisionCheckingBenchmark::CollisionCheckingBenchmark(OpenRAVE::EnvironmentBasePtr env) 
	: ModuleBase(env) {

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

	std::string body_name;
	OpenRAVE::KinBodyPtr body;
	if(const YAML::Node* pbody = doc.FindValue("body")){
		*pbody >> body_name;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Running collision checking for body %s\n", body_name.c_str());
		body = GetEnv()->GetKinBody(body_name);
		if(!body){
			RAVELOG_ERROR("[CollisionCheckingBenchmark] Failed to find body of name %s in environment\n", body_name.c_str());
			return false;
		}
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify a kinbody to collision check\n");
		return false;
	}

	double duration = 0.0;
	if(const YAML::Node* pduration = doc.FindValue("duration")){
		*pduration >> duration;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Running collision checking for %0.3f seconds\n", duration);
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify a duration\n");
		return false;
	}	

	double extent = 0.0;
	if(const YAML::Node* pextent = doc.FindValue("extent")){
		*pextent >> extent;
		RAVELOG_INFO("[CollisionCheckingBenchmark] Sampling poses from cube with extent %0.3f\n", extent);
	}else{
		RAVELOG_ERROR("[CollisionCheckingBenchmark] Must specify extents of the area for pose sampling.\n");
		return false;
	}	

	bool record = false;
	std::string outfile;
	if(const YAML::Node* poutfile = doc.FindValue("outfile")){
		*poutfile >> outfile;
		record = true;
	}

	// Setup random number generators
	typedef boost::uniform_real<> NumberDistribution;
	typedef boost::mt19937 RandomNumberGenerator;
	typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

	NumberDistribution distribution(0., extent);
	RandomNumberGenerator generator;
	Generator number_generator(generator, distribution);
	generator.seed(std::time(0));

	NumberDistribution adistribution(0., 2.*boost::math::constants::pi<double>());
	Generator angle_generator(generator, adistribution);

	double total_time = 0.0;
	typedef std::pair<OpenRAVE::Transform, double> BenchmarkData;
	std::vector<BenchmarkData> data;
	OpenRAVE::EnvironmentBasePtr env = GetEnv();
	while(total_time < duration){
		// Sample a random pose
		OpenRAVE::Transform btrans = body->GetTransform();
		btrans.trans.x = number_generator();
		btrans.trans.y = number_generator();
		btrans.trans.z = number_generator();

		OpenRAVE::RaveVector<OpenRAVE::dReal> rand_axis_angle(angle_generator(), angle_generator(), angle_generator());
		btrans.rot = OpenRAVE::geometry::quatFromAxisAngle(rand_axis_angle);

		// Set the transform
		body->SetTransform(btrans);

		// Check collision
		boost::timer t;
		bool incollision = env->CheckCollision(body);
		double elapsed = t.elapsed();
		total_time += elapsed;

		// Save the data
		data.push_back(std::make_pair(btrans, elapsed));
	}

	if(record){
		RAVELOG_INFO("[CollisionCheckingBenchmark] Recording results to file %s\n", outfile.c_str());
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
		
		std::ofstream out_stream(outfile.c_str(), std::ofstream::binary);
		out_stream << emitter.c_str();
		out_stream.close();
	}

	RAVELOG_INFO("[CollisionCheckingBenchmark] Results:\n");
	RAVELOG_INFO("\tTotal collision checks: %d\n",data.size());
	RAVELOG_INFO("\tTotal elapsed time: %0.5f\n", total_time);
	RAVELOG_INFO("\tAverage time per check: %0.7f\n", total_time/data.size());
}
