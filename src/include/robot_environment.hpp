#ifndef _ROBOT_ENVIRONMENT_HPP_
#define _ROBOT_ENVIRONMENT_HPP_
#include <assert.h>
#include <iostream> 
#include <fstream>
#include <dirent.h>
#include <tinyxml.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include "BoxObstacle.hpp"
#include "SphereObstacle.hpp"

namespace shared {

class RobotEnvironment {
public:
	RobotEnvironment();
	
	void addObstacle(std::shared_ptr<Obstacle> &obstacle);
	
	void loadObstaclesXML(std::string &obstacles_file);
	
	std::vector<std::shared_ptr<Obstacle>> getObstacles();
	
private:
	std::vector<std::shared_ptr<Obstacle>> obstacles_;
	
	bool file_exists(std::string &filename);
	
};


}

#endif