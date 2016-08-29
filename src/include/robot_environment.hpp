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
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include "BoxObstacle.hpp"
#include "SphereObstacle.hpp"
#include <robots/ManipulatorRobot.hpp>
#include <robots/DubinRobot.hpp>
#include <frapu_core/core.hpp>

namespace shared
{

class RobotEnvironment
{
public:
    RobotEnvironment();

    void setObstacles(std::vector<frapu::ObstacleSharedPtr>& obstacles);

    void setRobot(std::shared_ptr<shared::Robot>& robot);

    std::shared_ptr<shared::Robot> getRobot();

    bool loadEnvironment(std::string environment_file);

    std::vector<std::vector<double>> loadGoalStatesFromFile(std::string filename);

    void getObstacles(std::vector<frapu::ObstacleSharedPtr>& obstacles);

    /**
     * Get all observable obstacles
     */
    void getObservableObstacles(std::vector<frapu::ObstacleSharedPtr>& obstacles) const;

    void getGoalArea(std::vector<double>& goal_area);

    void setGoalArea(std::vector<double>& goal_area);

    void makeObservationSpace(const shared::ObservationSpaceInfo& observationSpaceInfo);

    /**
     * Create the robots
     */
    template <class RobotType> bool createRobot(std::string robot_file) {
        if (frapu::fileExists(robot_file)) {
            robot_ = std::make_shared<RobotType>(robot_file);
            robot_path_ = robot_file;
            return true;
        }

        return false;
    }

    bool createManipulatorRobot(std::string robot_file);

    bool createDubinRobot(std::string robot_file);

    std::shared_ptr<boost::mt19937> getRandomGenerator();

    void setControlDuration(double control_duration);

    void setSimulationStepSize(double simulation_step_size);

    void setGravityConstant(double gravity_constant);

    void setNewtonModel();

    double getControlDuration() const;

    double getSimulationStepSize() const;

    std::shared_ptr<Eigen::Distribution<double>> createDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covar,
            unsigned long& seed,
            std::string& type);

    template <class RobotType>
    std::shared_ptr<shared::RobotEnvironment> clone() {
        std::shared_ptr<shared::RobotEnvironment> env = std::make_shared<shared::RobotEnvironment>();
        env->createRobot<RobotType>(robot_path_);
        env->getRobot()->makeObservationSpace(robot_->getObservationSpace()->getObservationSpaceInfo());

        bool normalizedActionSpace = robot_->getActionSpace()->isNormalized();
        env->getRobot()->makeActionSpace(normalizedActionSpace);
        env->setControlDuration(control_duration_);
        env->setSimulationStepSize(simulation_step_size_);

        std::shared_ptr<Eigen::Distribution<double>> processDistribution = robot_->getProcessDistribution();
        std::shared_ptr<Eigen::Distribution<double>> observationDistribution = robot_->getObservationDistribution();
        Eigen::MatrixXd meanProcess = processDistribution->_mean;
        Eigen::MatrixXd covarProcess = processDistribution->_covar;
        uint64_t seedProc = processDistribution->_seed;

        Eigen::MatrixXd meanObs = observationDistribution->_mean;
        Eigen::MatrixXd covarObs = observationDistribution->_covar;
        uint64_t seedObs = observationDistribution->_seed;

        env->getRobot()->makeProcessDistribution(meanProcess, covarProcess, seedProc);
        env->getRobot()->makeObservationDistribution(meanObs, covarObs, seedObs);
        env->setGoalStates(goal_states_);	
	env->setEnvironmentInfo(environmentInfo_);
        env->setObstacles(obstacles_);        
        env->setGoalArea(goal_area_);
        if (dynamic_model_ == "newton") {
            env->getRobot()->setNewtonModel();
        }

        env->setGravityConstant(gravity_constant_);
        std::vector<double> goal_position( {goal_area_[0], goal_area_[1], goal_area_[2]});
        double goal_radius = goal_area_[3];
        env->getRobot()->setGoalArea(goal_position, goal_radius);
        return env;
    }

    void setGoalStates(std::vector<std::vector<double>>& goal_states);

    std::vector<std::vector<double>> getGoalStates() const;

    void generateRandomScene(unsigned int& numObstacles);

    /**** Methods to handle changes in the environment ****/
    frapu::ObstacleSharedPtr makeObstacle(std::string obstacleName,
                                          std::string obstacleType,
                                          std::vector<double>& dims,
                                          bool traversable,
                                          double traversalCost,
                                          bool observable) const;

    bool addObstacle(frapu::ObstacleSharedPtr& obstacle);

    frapu::ObstacleSharedPtr getObstacle(std::string name);

    bool removeObstacle(std::string obstacleName);

    bool removeObstacles(std::vector<std::string>& obstacle_names);

    void makeEnvironmentInfo();
    
    void setEnvironmentInfo(std::shared_ptr<frapu::EnvironmentInfo> &environmentInfo);

    frapu::EnvironmentInfoSharedPtr getEnvironmentInfo() const;

private:
    std::string robot_path_;

    std::string environment_path_;

    std::string dynamic_model_;

    double simulation_step_size_;

    double control_duration_;

    double gravity_constant_;

    std::vector<frapu::ObstacleSharedPtr> obstacles_;

    std::shared_ptr<shared::Robot> robot_;

    std::vector<double> goal_area_;

    std::vector<std::vector<double>> goal_states_;

    bool loadObstaclesXML(std::string& obstacles_file);

    bool loadGoalArea(std::string& env_file);

    std::shared_ptr<boost::mt19937> generator_;

    frapu::EnvironmentInfoSharedPtr environmentInfo_;

};


}

#endif
