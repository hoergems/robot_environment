#ifndef _ROBOT_ENVIRONMENT_HPP_
#define _ROBOT_ENVIRONMENT_HPP_
#include <assert.h>
#include <iostream> 
#include <fstream>
#include <dirent.h>
#include <tinyxml.h>
#include <Eigen/Dense>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include "BoxObstacle.hpp"
#include "SphereObstacle.hpp"
#include <robots/ManipulatorRobot.hpp>
#include <robots/DubinRobot.hpp>
#include "mult_normal.hpp"

namespace shared {

class RobotEnvironment {
public:
	RobotEnvironment();
	
	void addObstacle(std::shared_ptr<Obstacle> &obstacle);
	
	void setRobot(std::shared_ptr<shared::Robot> &robot);	
	
	std::shared_ptr<shared::Robot> getRobot();
	
	bool loadEnvironment(std::string environment_file);
	
	void getObstacles(std::vector<std::shared_ptr<shared::Obstacle> > &obstacles);
	
	std::vector<std::shared_ptr<shared::ObstacleWrapper>> getObstaclesPy();
	
	void getGoalArea(std::vector<double> &goal_area);
	
	/**
	 * Create the robots
	 */
	bool createManipulatorRobot(std::string robot_file);
	
	bool createDubinRobot(std::string robot_file);
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> createDistribution(Eigen::MatrixXd &mean, 
			                                                                    Eigen::MatrixXd &covariance_matrix);
	
	void setProcessDistribution(std::shared_ptr<shared::EigenMultivariateNormal<double>> &process_distribution);
	
	void setObservationDistribution(std::shared_ptr<shared::EigenMultivariateNormal<double>> &observation_distribution);
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> getProcessDistribution();
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> getObservationDistribution();
	
private:
	std::vector<std::shared_ptr<Obstacle>> obstacles_;
	
	std::shared_ptr<shared::Robot> robot_;
	
	std::vector<double> goal_area_;
	
	bool file_exists(std::string &filename);
	
	bool loadObstaclesXML(std::string &obstacles_file);
		
	bool loadGoalArea(std::string &env_file);
	
	boost::mt19937 generator_;
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> process_distribution_;
	
	std::shared_ptr<shared::EigenMultivariateNormal<double>> observation_distribution_;
	
};


}

#endif