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
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#include "BoxObstacle.hpp"
#include "SphereObstacle.hpp"
#include <robot_headers/robot.hpp>
#include <frapu_core/core.hpp>

namespace frapu
{

class RobotEnvironment
{
public:
    RobotEnvironment();

    //void setObstacles(std::vector<frapu::ObstacleSharedPtr>& obstacles);

    void setRobot(std::shared_ptr<frapu::Robot>& robot);

    std::shared_ptr<frapu::Robot> getRobot();

    bool loadEnvironment(std::string environment_file);

    std::vector<frapu::RobotStateSharedPtr> loadGoalStatesFromFile(std::string filename);

    void getGoalArea(std::vector<double>& goal_area);

    void setGoalArea(std::vector<double>& goal_area);
    
    /**
     * Create the robots
     */
    template <class RobotType> bool createRobot(std::string robotFile, std::string configFile) {
        if (frapu::fileExists(robotFile) && frapu::fileExists(configFile)) {
	    cout << "config file " << configFile << endl;
            robot_ = std::make_shared<RobotType>(robotFile, configFile);
            robot_path_ = robotFile;
	    config_path_ = configFile;
	    cout << "Robot created" << endl;
            return true;
        }

        return false;
    }   

    std::shared_ptr<boost::mt19937> getRandomGenerator();    

    void setSimulationStepSize(double simulation_step_size);
    
    double getSimulationStepSize() const;

    void setGravityConstant(double gravity_constant);

    void setNewtonModel();

    std::shared_ptr<Eigen::Distribution<double>> createDistribution(Eigen::MatrixXd& mean,
            Eigen::MatrixXd& covar,
            unsigned long& seed,
            std::string& type);

    template <class RobotType>
    std::shared_ptr<RobotEnvironment> clone() {
        std::shared_ptr<RobotEnvironment> env = std::make_shared<RobotEnvironment>();
	env->setScene(scene_);
        env->createRobot<RobotType>(robot_path_, config_path_);
        env->getRobot()->makeStateSpace();
        env->getRobot()->makeObservationSpace(robot_->getObservationSpace()->getObservationSpaceInfo());

        const frapu::ActionSpaceInfo info = robot_->getActionSpace()->getInfo();
        env->getRobot()->makeActionSpace(info);
	double controlDuration = robot_->getControlDuration();
        env->getRobot()->setControlDuration(controlDuration);
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
	std::vector<frapu::RobotStateSharedPtr> goalStates = robot_->getGoalStates();
        env->getRobot()->setGoalStates(goalStates);	
	env->setEnvironmentInfo(environmentInfo_);              
        env->setGoalArea(goal_area_);
        if (dynamic_model_ == "newton") {
            env->getRobot()->setNewtonModel();
        }

        env->setGravityConstant(gravity_constant_);
        std::vector<double> goal_position( {goal_area_[0], goal_area_[1], goal_area_[2]});
        double goal_radius = goal_area_[3];
        env->getRobot()->setGoalArea(goal_position, goal_radius);
	env->getRobot()->makeGoal();
	env->setRewardModel(rewardModel_);
	env->getRobot()->setupHeuristic(rewardModel_);
        return env;
    }    

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
    
    void updateEnvironment(const frapu::RobotStateSharedPtr &state);
    
    void setRewardModel(frapu::RewardModelSharedPtr &rewardModel);
    
    void setScene(frapu::SceneSharedPtr &scene);
    
    frapu::SceneSharedPtr getScene() const;

private:
    std::string robot_path_;
    
    std::string config_path_;

    std::string environment_path_;

    std::string dynamic_model_;

    double simulation_step_size_;    

    double gravity_constant_;    

    std::shared_ptr<frapu::Robot> robot_;

    std::vector<double> goal_area_;   

    bool loadObstaclesXML(std::string& obstacles_file);

    bool loadGoalArea(std::string& env_file);

    std::shared_ptr<boost::mt19937> generator_;

    frapu::EnvironmentInfoSharedPtr environmentInfo_;
    
    frapu::RewardModelSharedPtr rewardModel_;
    
    frapu::SceneSharedPtr scene_;

};


}

#endif
