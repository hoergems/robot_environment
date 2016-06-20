#include "include/robot_environment.hpp"

namespace shared {

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

struct TerrainStruct {
    std::string name;
    double velocityDamping;
    double traversalCost;    
    bool traversable;
};

struct ObstacleStruct {
	std::string name;
	std::string type;
    double x;
    double y;
    double z;
    std::vector<double> extends;
    
    // Diffuse color
    std::vector<double> d_color;
    
    // Ambient color
    std::vector<double> a_color;
    TerrainStruct terrain;     
};

RobotEnvironment::RobotEnvironment():
	obstacles_(),
    goal_area_(),
    robot_(nullptr),
    process_distribution_(nullptr),
    observation_distribution_(nullptr),
    generator_(nullptr),
	robot_path_(""),
	environment_path_("")
{
	boost::random_device rd;
	generator_ = std::make_shared<boost::mt19937>(rd());	
}

std::shared_ptr<RobotEnvironment> RobotEnvironment::clone() {
	std::shared_ptr<RobotEnvironment> env = std::make_shared<RobotEnvironment>();
	env->createManipulatorRobot(robot_path_);
	env->loadEnvironment(environment_path_);
	env->setControlDuration(control_duration_);
	env->setSimulationStepSize(simulation_step_size_);
	env->setProcessDistribution(process_distribution_);
	env->setObservationDistribution(observation_distribution_);
	return env;
}

std::shared_ptr<shared::EigenMultivariateNormal<double>> RobotEnvironment::createDistribution(Eigen::MatrixXd &mean, Eigen::MatrixXd &covariance_matrix) {
	std::shared_ptr<shared::EigenMultivariateNormal<double>> distribution = 
			std::make_shared<shared::EigenMultivariateNormal<double>>(*(generator_.get()));
	//process_distribution_ = std::make_shared<shared::EigenMultivariateNormal<double>>(generator_);
	distribution->setMean(mean);
	distribution->setCovar(covariance_matrix);
	return distribution;
}

void RobotEnvironment::setProcessDistribution(std::shared_ptr<shared::EigenMultivariateNormal<double>> &process_distribution) {
	process_distribution_ = process_distribution;
}

void RobotEnvironment::setObservationDistribution(std::shared_ptr<shared::EigenMultivariateNormal<double>> &observation_distribution) {
	observation_distribution_ = observation_distribution;
}

std::shared_ptr<shared::EigenMultivariateNormal<double>> RobotEnvironment::getProcessDistribution() {
	return process_distribution_;
}
	
std::shared_ptr<shared::EigenMultivariateNormal<double>> RobotEnvironment::getObservationDistribution() {
	return observation_distribution_;
}

std::shared_ptr<boost::mt19937> RobotEnvironment::getRandomGenerator() {
	return generator_;
}

void RobotEnvironment::addObstacle(std::shared_ptr<Obstacle> &obstacle) {
	obstacles_.push_back(obstacle);
}

void RobotEnvironment::setRobot(std::shared_ptr<shared::Robot> &robot) {
	robot_ = robot;
}

bool RobotEnvironment::createManipulatorRobot(std::string robot_file) {	
	if (file_exists(robot_file)) {
		robot_ = std::make_shared<shared::ManipulatorRobot>(robot_file);
		robot_path_ = robot_file;
		return true;
	}
	
	return false;
}

bool RobotEnvironment::createDubinRobot(std::string robot_file) {	
	if (file_exists(robot_file)) {
		robot_ = std::make_shared<shared::DubinRobot>(robot_file);
		robot_path_ = robot_file;
		return true;
	}
	
	return false;
}

std::shared_ptr<shared::Robot> RobotEnvironment::getRobot() {
	return robot_;
} 

void RobotEnvironment::setControlDuration(double control_duration) {
	control_duration_ = control_duration;
}
	
void RobotEnvironment::setSimulationStepSize(double simulation_step_size) {
	simulation_step_size_ = simulation_step_size;
}
	
double RobotEnvironment::getControlDuration() const {
	return control_duration_;
}
	
double RobotEnvironment::getSimulationStepSize() const {
	return simulation_step_size_;
}

void RobotEnvironment::getObstacles(std::vector<std::shared_ptr<shared::Obstacle> > &obstacles) {
	for (auto &k: obstacles_) {
		obstacles.push_back(k);
	}
	
}

std::vector<std::shared_ptr<shared::ObstacleWrapper>> RobotEnvironment::getObstaclesPy() {
	std::vector<std::shared_ptr<shared::ObstacleWrapper>> obstacles;
	for (size_t i = 0; i < obstacles_.size(); i++) {
		obstacles.push_back(std::static_pointer_cast<shared::ObstacleWrapper>(obstacles_[i]));
	}
	
	return obstacles;	
}

bool RobotEnvironment::file_exists(std::string &filename) {
	return boost::filesystem::exists(filename);
}

bool RobotEnvironment::loadEnvironment(std::string environment_file) {
	if (!loadObstaclesXML(environment_file)) {
		return false;
	}
	
	if (!loadGoalArea(environment_file)) {
		return false;
	}
	
	environment_path_ = environment_file;
	return true;
}

bool RobotEnvironment::loadObstaclesXML(std::string &obstacles_file) {	
	if (!file_exists(obstacles_file)) {
		cout << "RobotEnvironment: ERROR: Environment file '" << obstacles_file << "' doesn't exist" << endl;		
		assert(false);		
	}
	obstacles_.clear();
	std::vector<ObstacleStruct> obstacles;	
	TiXmlDocument xml_doc;	
    xml_doc.LoadFile(obstacles_file);    
	TiXmlElement *env_xml = xml_doc.FirstChildElement("Environment");
	for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody"))
	{	
		std::string name(obst_xml->Attribute("name"));
		TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");
		std::string enable_str(body_xml->Attribute("enable"));		
		if (enable_str == "true") {			
			TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");			
			if (body_xml) {
				TiXmlElement *geom_xml = body_xml->FirstChildElement("Geom");
				TiXmlElement *terrain_xml = body_xml->FirstChildElement("Terrain");
				//Can play with different shapes here in the future
				if (geom_xml) {
					std::string type(geom_xml->Attribute("type"));					
					TiXmlElement *trans_xml = geom_xml->FirstChildElement("Translation");
					if (trans_xml) {
						obstacles.push_back(ObstacleStruct());
						const char* xyz_str = trans_xml->GetText();
						std::vector<std::string> pieces;
						std::vector<double> xyz_vec;
						boost::split( pieces, xyz_str, boost::is_any_of(" "));
				        for (unsigned int i = 0; i < pieces.size(); ++i) {
					       if (pieces[i] != "") { 
					          xyz_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
		                   }
			            }
				        
				        TiXmlElement *dcolor_xml = geom_xml->FirstChildElement("diffuseColor");
				        if (dcolor_xml) {
				        	const char* color_string = dcolor_xml->GetText();				        	
				        	std::vector<std::string> pieces;
				        	std::vector<double> color_vec;
				        	boost::split(pieces, color_string, boost::is_any_of(" "));
				            for (unsigned i = 0; i < pieces.size(); i++) {				            	
				        		color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				        	}
				            
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[0]);
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[1]);
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[2]);
				            
				        }
				        
				        TiXmlElement *acolor_xml = geom_xml->FirstChildElement("ambientColor");
				        if (acolor_xml) {
				            const char* color_string = acolor_xml->GetText();
				        	std::vector<std::string> pieces;
				            std::vector<double> color_vec;
				            boost::split(pieces, color_string, boost::is_any_of(" "));
				        	for (unsigned i = 0; i < pieces.size(); i++) {
				        		color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				        	}
				        				            
				        	obstacles[obstacles.size() - 1].a_color.push_back(color_vec[0]);
				            obstacles[obstacles.size() - 1].a_color.push_back(color_vec[1]);
				        	obstacles[obstacles.size() - 1].a_color.push_back(color_vec[2]);				        				            
				        }
				        
				        obstacles[obstacles.size() - 1].name = name;
				        obstacles[obstacles.size() - 1].type = type;
				        obstacles[obstacles.size() - 1].x = xyz_vec[0];
				        obstacles[obstacles.size() - 1].y = xyz_vec[1];
				        obstacles[obstacles.size() - 1].z = xyz_vec[2];
					    if (type == "box") {					    	
						    TiXmlElement *ext_xml = geom_xml->FirstChildElement("extents");
						    if (ext_xml) {
						        const char* ext_str = ext_xml->GetText();						        
						        std::vector<double> extends_vec;
						        pieces.clear();
						        boost::split( pieces, ext_str, boost::is_any_of(" "));
						        for (unsigned int i = 0; i < pieces.size(); ++i) {
						            if (pieces[i] != "") {
						                extends_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
						            }
						        }
						        
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[0]);
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[1]);
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[2]);						        
						    }
					    }
					    else if (type == "sphere") {
					    	TiXmlElement *rad_xml = geom_xml->FirstChildElement("Radius");
					    	if (rad_xml) {					    		
					    		obstacles[obstacles.size() - 1].extends.push_back(boost::lexical_cast<double>(rad_xml->GetText()));					    		
					    	}
					    }
					}
					
				}
				
				if (terrain_xml) {					
					TerrainStruct terrain;
					std::string terrain_name(terrain_xml->Attribute("name"));					
					TiXmlElement *damping_xml = terrain_xml->FirstChildElement("Damping");
					double damping = boost::lexical_cast<double>(damping_xml->GetText());
					TiXmlElement *cost_xml = terrain_xml->FirstChildElement("Cost");
					double cost = boost::lexical_cast<double>(cost_xml->GetText());
					TiXmlElement *traversable_xml = terrain_xml->FirstChildElement("Traversable");
					bool traversable = false;					
					if (boost::lexical_cast<std::string>(traversable_xml->GetText()) == "true") {						
						traversable = true;
					}
					terrain.name = terrain_name;
					terrain.velocityDamping = damping;
					terrain.traversalCost = cost;
					terrain.traversable = traversable;
					obstacles[obstacles.size() - 1].terrain = terrain;					
				}
			}
		}
	}
	
	for (size_t i = 0; i < obstacles.size(); i++) {
	    shared::Terrain terrain(obstacles[i].terrain.name,
	                            obstacles[i].terrain.traversalCost,
	                            obstacles[i].terrain.velocityDamping,
	                            obstacles[i].terrain.traversable);
	    if (obstacles[i].type == "box") {
	    	obstacles_.push_back(std::make_shared<shared::BoxObstacle>(obstacles[i].name,
	    			                                             obstacles[i].x,
                                                                 obstacles[i].y,
                                                                 obstacles[i].z,
														         obstacles[i].extends[0],
														         obstacles[i].extends[1],
														         obstacles[i].extends[2],
														         terrain));
	    }
	    
	    else if (obstacles[i].type == "sphere") {
	    	obstacles_.push_back(std::make_shared<shared::SphereObstacle>(obstacles[i].name,
	    			                                                obstacles[i].x,
                                                                    obstacles[i].y,
                                                                    obstacles[i].z,
                                                                    obstacles[i].extends[0],
					                                                terrain));
	    }
	    else {
	    	assert(false && "Utils: ERROR: Obstacle has an unknown type!");
	    }
	    
	    obstacles_[obstacles_.size() - 1]->setStandardColor(obstacles[i].d_color, obstacles[i].a_color);
	}
}

std::vector<std::vector<double>> RobotEnvironment::loadGoalStatesFromFile(std::string filename) {
	std::vector<std::vector<double>> gs;
	        std::ifstream file;
	        try {
	            file.open(filename);
	        }
	        catch (std::ios_base::failure& e) {
	            std::cerr << e.what() << '\n';
	            sleep(5);
	        }
	        
	        double dub_val;
	        std::string line;
	               
	        while (std::getline(file, line))
	        {                      
	            std::istringstream sin(line);
	            std::vector<double> angles; 
	            while (sin >> dub_val) {
	                angles.push_back(dub_val);                
	            }
	            
	            gs.push_back(angles);            
	        }
	        file.clear();
	        file.seekg(0, file.beg);
	        file.close();
	        return gs; 
}

void RobotEnvironment::getGoalArea(std::vector<double> &goal_area) {	
	for (auto &k: goal_area_) {		
		goal_area.push_back(k);
	}	
}

bool RobotEnvironment::loadGoalArea(std::string &env_file) {
	if (!file_exists(env_file)) {
		cout << "Utils: ERROR: Environment file '" << env_file << "' doesn't exist" << endl;		
		assert(false);		
	}
	goal_area_.clear();
	TiXmlDocument xml_doc;	
	xml_doc.LoadFile(env_file);    
    TiXmlElement *env_xml = xml_doc.FirstChildElement("Environment");
	for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody"))
	{
		std::string name(obst_xml->Attribute("name"));
		if (name == "GoalArea") { 
			TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");			
			if (body_xml) {
				TiXmlElement *geom_xml = body_xml->FirstChildElement("Geom");
				if (geom_xml) {
					TiXmlElement *trans_xml = geom_xml->FirstChildElement("Translation");
					if (trans_xml) {
						const char* xyz_str = trans_xml->GetText();
						std::vector<std::string> pieces;						
						boost::split( pieces, xyz_str, boost::is_any_of(" "));
						for (unsigned int i = 0; i < pieces.size(); ++i) {
						    if (pieces[i] != "") { 
						        goal_area_.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
							}
						}
					}
					TiXmlElement *radius_xml = geom_xml->FirstChildElement("Radius");
					if (radius_xml) {
						const char* rad_str = radius_xml->GetText();
						goal_area_.push_back(boost::lexical_cast<double>(radius_xml->GetText()));
					}
				}				
			}
		}
	}
}

BOOST_PYTHON_MODULE(librobot_environment) {
    #include "include/Terrain.hpp"
    using namespace boost::python;    

    bool (ObstacleWrapper::*in_collision_d)(boost::python::list&) = &ObstacleWrapper::in_collision_discrete;
    bool (ObstacleWrapper::*in_collision_c)(boost::python::list&) = &ObstacleWrapper::in_collision_continuous;
    bool (ObstacleWrapper::*in_collision_p)(std::vector<double>&) = &ObstacleWrapper::in_collision_point;
    
    boost::python::type_info info = boost::python::type_id<std::vector<int>>();
    const boost::python::converter::registration* reg_int = boost::python::converter::registry::query(info);
    if (reg_int == NULL || (*reg_int).m_to_python == NULL)  {    
        class_<std::vector<int> > ("v_int")
            .def(vector_indexing_suite<std::vector<int> >());
    }
    
    class_<std::vector<std::shared_ptr<shared::ObstacleWrapper>> > ("v_obstacle")
                .def(vector_indexing_suite<std::vector<std::shared_ptr<shared::ObstacleWrapper>> >());
    to_python_converter<std::vector<std::shared_ptr<shared::ObstacleWrapper>, std::allocator<std::shared_ptr<shared::ObstacleWrapper>> >, 
                VecToList<std::shared_ptr<shared::ObstacleWrapper>> >();
    register_ptr_to_python<std::shared_ptr<shared::ObstacleWrapper>>();
    
    class_<ObstacleWrapper, boost::noncopyable>("Obstacle", init<std::string, Terrain>())         
             .def("inCollisionDiscrete", in_collision_d)
             .def("inCollisionContinuous", in_collision_c)
    		 .def("inCollisionPoint", in_collision_p)
             .def("isTraversable", &ObstacleWrapper::isTraversable)
    		 .def("getExternalForce", &ObstacleWrapper::getExternalForce)
    		 .def("createCollisionObject", boost::python::pure_virtual(&ObstacleWrapper::createCollisionObject))
    		 .def("getName", &ObstacleWrapper::getName)
    		 .def("getStandardDiffuseColor", &ObstacleWrapper::getStandardDiffuseColor)
    		 .def("getStandardAmbientColor", &ObstacleWrapper::getStandardAmbientColor)
    		 .def("distance", &ObstacleWrapper::distancePy)
        ;
   
    class_<RobotEnvironment, std::shared_ptr<shared::RobotEnvironment> >("RobotEnvironment", init<>())
							   .def("getRobot", &RobotEnvironment::getRobot)
							   .def("createManipulatorRobot", &RobotEnvironment::createManipulatorRobot)
							   .def("createDubinRobot", &RobotEnvironment::createDubinRobot)
							   .def("getGoalArea", &RobotEnvironment::getGoalArea)
							   .def("loadEnvironment", &RobotEnvironment::loadEnvironment)
							   .def("getObstacles", &RobotEnvironment::getObstaclesPy)
							   
    ;
}



}