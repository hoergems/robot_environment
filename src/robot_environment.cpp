#include "include/robot_environment.hpp"

namespace frapu
{

template<class T>
struct VecToList {
    static PyObject* convert(const std::vector<T>& vec) {
        boost::python::list* l = new boost::python::list();
        for (size_t i = 0; i < vec.size(); i++) {
            (*l).append(vec[i]);
        }

        return l->ptr();
    }
};

struct TerrainStruct {
    std::string name;
    double velocityDamping;
    double traversalCost;
    bool traversable;
    bool observable;
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
    goal_area_(),
    robot_(nullptr),
    generator_(nullptr),
    robot_path_(""),
    config_path_(""),
    environment_path_(""),
    gravity_constant_(0.0),
    dynamic_model_("lagrange"),
    environmentInfo_(nullptr),
    rewardModel_(nullptr),
    scene_(nullptr)
{
    boost::random_device rd;
    generator_ = std::make_shared<boost::mt19937>(rd());
}

std::shared_ptr<Eigen::Distribution<double>> RobotEnvironment::createDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covar,
        unsigned long& seed,
        std::string& type)
{
    std::shared_ptr<Eigen::Distribution<double>> distr;
    if (type == "MultivariateNormal") {
        distr = std::make_shared<Eigen::EigenMultivariateNormal<double>>(mean, covar, false, seed);
    }

    return distr;
}

/**std::shared_ptr<Eigen::Distribution<double>> RobotEnvironment::createDistribution(Eigen::MatrixXd& mean,
        Eigen::MatrixXd& covariance_matrix,
        unsigned long seed)
{
    std::shared_ptr<Eigen::Distribution<double>> distribution =
        std::make_shared<Eigen::Distribution<double>>(mean, covariance_matrix, false, seed);
    //distribution->setMean(mean);
    //distribution->setCovar(covariance_matrix);
    return distribution;
}*/



std::shared_ptr<boost::mt19937> RobotEnvironment::getRandomGenerator()
{
    return generator_;
}

/**void RobotEnvironment::setObstacles(std::vector<frapu::ObstacleSharedPtr>& obstacles)
{
    obstacles_ = obstacles;
    environmentInfo_->obstacles = obstacles;
}*/

void RobotEnvironment::makeEnvironmentInfo()
{
    environmentInfo_ = std::make_shared<frapu::EnvironmentInfo>();
    environmentInfo_->scene = scene_;    
    robot_->setEnvironmentInfo(environmentInfo_);
}

void RobotEnvironment::setEnvironmentInfo(std::shared_ptr<frapu::EnvironmentInfo>& environmentInfo)
{
    environmentInfo_ = environmentInfo;
    robot_->setEnvironmentInfo(environmentInfo_);
}

void RobotEnvironment::updateEnvironment(const frapu::RobotStateSharedPtr& state)
{
    robot_->updateRobot(state);
}

void RobotEnvironment::setRobot(std::shared_ptr<frapu::Robot>& robot)
{
    robot_ = robot;
}

std::shared_ptr<frapu::Robot> RobotEnvironment::getRobot()
{
    return robot_;
}

void RobotEnvironment::setSimulationStepSize(double simulation_step_size)
{
    simulation_step_size_ = simulation_step_size;
}

void RobotEnvironment::setGravityConstant(double gravity_constant)
{
    gravity_constant_ = gravity_constant;
    robot_->setGravityConstant(gravity_constant_);
}

void RobotEnvironment::setNewtonModel()
{
    dynamic_model_ = "newton";
    robot_->setNewtonModel();
}

double RobotEnvironment::getSimulationStepSize() const
{
    return simulation_step_size_;
}

void RobotEnvironment::setScene(frapu::SceneSharedPtr& scene)
{
    scene_ = scene;
}

frapu::SceneSharedPtr RobotEnvironment::getScene() const
{
    return scene_;
}

bool RobotEnvironment::loadEnvironment(std::string environment_file)
{
    scene_ = std::make_shared<frapu::Scene>();
    if (!loadObstaclesXML(environment_file)) {
        cout << "RobotEnvironment: Couldn't load environment" << endl;
        return false;
    }

    cout << "RobotEnvironment: Environment loaded successfully" << endl;
    if (!loadGoalArea(environment_file)) {
        cout << "RobotEnvironment: Couldn't load goal area" << endl;
        return false;
    }

    cout << "RobotEnvironment: Successfully loaded goal area" << endl;
    environment_path_ = environment_file;
    return true;
}

bool RobotEnvironment::loadObstaclesXML(std::string& obstacles_file)
{
    if (!frapu::fileExists(obstacles_file)) {
        cout << "RobotEnvironment: ERROR: Environment file '" << obstacles_file << "' doesn't exist" << endl;
        return false;
    }

    std::vector<frapu::ObstacleSharedPtr> obstacles;
    std::vector<ObstacleStruct> obstaclesStruct;
    TiXmlDocument xml_doc;
    xml_doc.LoadFile(obstacles_file);
    TiXmlElement* env_xml = xml_doc.FirstChildElement("Environment");
    for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody")) {
        std::string name(obst_xml->Attribute("name"));
        TiXmlElement* body_xml = obst_xml->FirstChildElement("Body");
        std::string enable_str(body_xml->Attribute("enable"));
        if (enable_str == "true") {
            TiXmlElement* body_xml = obst_xml->FirstChildElement("Body");
            if (body_xml) {
                TiXmlElement* geom_xml = body_xml->FirstChildElement("Geom");
                TiXmlElement* terrain_xml = body_xml->FirstChildElement("Terrain");
                //Can play with different shapes here in the future
                if (geom_xml) {
                    std::string type(geom_xml->Attribute("type"));
                    TiXmlElement* trans_xml = geom_xml->FirstChildElement("Translation");
                    if (trans_xml) {
                        obstaclesStruct.push_back(ObstacleStruct());
                        const char* xyz_str = trans_xml->GetText();
                        std::vector<std::string> pieces;
                        std::vector<double> xyz_vec;
                        boost::split(pieces, xyz_str, boost::is_any_of(" "));
                        for (unsigned int i = 0; i < pieces.size(); ++i) {
                            if (pieces[i] != "") {
                                xyz_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                            }
                        }

                        TiXmlElement* dcolor_xml = geom_xml->FirstChildElement("diffuseColor");
                        if (dcolor_xml) {
                            const char* color_string = dcolor_xml->GetText();
                            std::vector<std::string> pieces;
                            std::vector<double> color_vec;
                            boost::split(pieces, color_string, boost::is_any_of(" "));
                            for (unsigned i = 0; i < pieces.size(); i++) {
                                color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                            }

                            obstaclesStruct[obstaclesStruct.size() - 1].d_color.push_back(color_vec[0]);
                            obstaclesStruct[obstaclesStruct.size() - 1].d_color.push_back(color_vec[1]);
                            obstaclesStruct[obstaclesStruct.size() - 1].d_color.push_back(color_vec[2]);

                        }

                        TiXmlElement* acolor_xml = geom_xml->FirstChildElement("ambientColor");
                        if (acolor_xml) {
                            const char* color_string = acolor_xml->GetText();
                            std::vector<std::string> pieces;
                            std::vector<double> color_vec;
                            boost::split(pieces, color_string, boost::is_any_of(" "));
                            for (unsigned i = 0; i < pieces.size(); i++) {
                                color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                            }

                            obstaclesStruct[obstaclesStruct.size() - 1].a_color.push_back(color_vec[0]);
                            obstaclesStruct[obstaclesStruct.size() - 1].a_color.push_back(color_vec[1]);
                            obstaclesStruct[obstaclesStruct.size() - 1].a_color.push_back(color_vec[2]);
                        }

                        obstaclesStruct[obstaclesStruct.size() - 1].name = name;
                        obstaclesStruct[obstaclesStruct.size() - 1].type = type;
                        obstaclesStruct[obstaclesStruct.size() - 1].x = xyz_vec[0];
                        obstaclesStruct[obstaclesStruct.size() - 1].y = xyz_vec[1];
                        obstaclesStruct[obstaclesStruct.size() - 1].z = xyz_vec[2];
                        if (type == "box") {
                            TiXmlElement* ext_xml = geom_xml->FirstChildElement("extents");
                            if (ext_xml) {
                                const char* ext_str = ext_xml->GetText();
                                std::vector<double> extends_vec;
                                pieces.clear();
                                boost::split(pieces, ext_str, boost::is_any_of(" "));
                                for (unsigned int i = 0; i < pieces.size(); ++i) {
                                    if (pieces[i] != "") {
                                        extends_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                                    }
                                }

                                obstaclesStruct[obstaclesStruct.size() - 1].extends.push_back(extends_vec[0]);
                                obstaclesStruct[obstaclesStruct.size() - 1].extends.push_back(extends_vec[1]);
                                obstaclesStruct[obstaclesStruct.size() - 1].extends.push_back(extends_vec[2]);
                            }
                        } else if (type == "sphere") {
                            TiXmlElement* rad_xml = geom_xml->FirstChildElement("Radius");
                            if (rad_xml) {
                                obstaclesStruct[obstaclesStruct.size() - 1].extends.push_back(boost::lexical_cast<double>(rad_xml->GetText()));
                            }
                        }
                    }

                }

                if (terrain_xml) {
                    TerrainStruct terrain;
                    std::string terrain_name(terrain_xml->Attribute("name"));
                    TiXmlElement* damping_xml = terrain_xml->FirstChildElement("Damping");
                    double damping = boost::lexical_cast<double>(damping_xml->GetText());
                    TiXmlElement* cost_xml = terrain_xml->FirstChildElement("Cost");
                    double cost = boost::lexical_cast<double>(cost_xml->GetText());
                    TiXmlElement* traversable_xml = terrain_xml->FirstChildElement("Traversable");
                    bool traversable = false;
                    if (traversable_xml) {
                        if (boost::lexical_cast<std::string>(traversable_xml->GetText()) == "true") {
                            traversable = true;
                        }
                    }

                    bool observable = true;
                    TiXmlElement* observable_xml = terrain_xml->FirstChildElement("Observable");
                    if (observable_xml) {
                        if (boost::lexical_cast<std::string>(observable_xml->GetText()) == "false") {
                            observable = false;
                        }
                    }

                    terrain.name = terrain_name;
                    terrain.velocityDamping = damping;
                    terrain.traversalCost = cost;
                    terrain.traversable = traversable;
                    terrain.observable = observable;
                    obstaclesStruct[obstaclesStruct.size() - 1].terrain = terrain;
                }
            }
        }
    }

    for (size_t i = 0; i < obstaclesStruct.size(); i++) {
        frapu::TerrainSharedPtr terrain = std::make_shared<frapu::TerrainImpl>(obstaclesStruct[i].terrain.name,
                                          obstaclesStruct[i].terrain.traversalCost,
                                          obstaclesStruct[i].terrain.velocityDamping,
                                          obstaclesStruct[i].terrain.traversable,
                                          obstaclesStruct[i].terrain.observable);
        if (obstaclesStruct[i].type == "box") {
            frapu::ObstacleSharedPtr obstacle = std::make_shared<frapu::BoxObstacle>(obstaclesStruct[i].name,
                                                obstaclesStruct[i].x,
                                                obstaclesStruct[i].y,
                                                obstaclesStruct[i].z,
                                                obstaclesStruct[i].extends[0],
                                                obstaclesStruct[i].extends[1],
                                                obstaclesStruct[i].extends[2],
                                                terrain);
            obstacles.push_back(obstacle);
        }

        else if (obstaclesStruct[i].type == "sphere") {
            frapu::ObstacleSharedPtr obstacle = std::make_shared<frapu::SphereObstacle>(obstaclesStruct[i].name,
                                                obstaclesStruct[i].x,
                                                obstaclesStruct[i].y,
                                                obstaclesStruct[i].z,
                                                obstaclesStruct[i].extends[0],
                                                terrain);
            obstacles.push_back(obstacle);
        } else {
            assert(false && "Utils: ERROR: Obstacle has an unknown type!");
        }

        static_cast<frapu::ObstacleImpl*>(obstacles[obstacles.size() - 1].get())->setStandardColor(obstaclesStruct[i].d_color,
                obstaclesStruct[i].a_color);
        //obstacles_[obstacles_.size() - 1]->setStandardColor(obstacles[i].d_color, obstacles[i].a_color);
    }

    scene_->setObstacles(obstacles);

    return true;
}

void RobotEnvironment::setRewardModel(frapu::RewardModelSharedPtr& rewardModel)
{
    rewardModel_ = rewardModel;
}

void RobotEnvironment::generateRandomScene(unsigned int& numObstacles)
{
    scene_ = std::make_shared<frapu::Scene>();
    std::vector<frapu::ObstacleSharedPtr> obstacles(numObstacles);
    std::uniform_real_distribution<double> uniform_dist(-1.0, 4.0);
    std::uniform_real_distribution<double> uniform_distZ(3.0, 6.0);
    for (size_t i = 0; i < numObstacles; i++) {
        double rand_x = uniform_dist(*generator_);
        double rand_y = uniform_dist(*generator_);
        double rand_z = uniform_distZ(*generator_);

        frapu::TerrainSharedPtr terrain = std::make_shared<frapu::TerrainImpl>("t" + std::to_string(i),
                                          0.0,
                                          0.0,
                                          false,
                                          true);

        std::string box_name = "b" + std::to_string(i);
        obstacles[i] = std::make_shared<frapu::BoxObstacle>(box_name,
                       rand_x,
                       rand_y,
                       rand_z,
                       0.25,
                       0.25,
                       0.25,
                       terrain);
        std::vector<double> diffuseColor( {0.5, 0.5, 0.5, 0.5});
        static_cast<frapu::ObstacleImpl*>(obstacles[obstacles.size() - 1].get())->setStandardColor(diffuseColor, diffuseColor);
        std::vector<double> dims( {rand_x, rand_y, rand_z, 0.25, 0.25, 0.25});
        robot_->addBox(box_name, dims);

    }

    scene_->setObstacles(obstacles);
    cout << "random scene created " << obstacles.size();

}

std::vector<frapu::RobotStateSharedPtr> RobotEnvironment::loadGoalStatesFromFile(std::string filename)
{
    return robot_->loadGoalStatesFromFile(filename);
    /**goalStates_.clear();

    std::ifstream file;
    try {
        file.open(filename);
    } catch (std::ios_base::failure& e) {
        std::cerr << e.what() << '\n';
        sleep(5);
    }

    double dub_val;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream sin(line);
        std::vector<double> angles;
        while (sin >> dub_val) {
            angles.push_back(dub_val);
        }

        goalStates_.push_back(angles);
    }
    file.clear();
    file.seekg(0, file.beg);
    file.close();
    return goalStates_;*/

}

void RobotEnvironment::getGoalArea(std::vector<double>& goal_area)
{
    return robot_->getGoalArea(goal_area);
}

void RobotEnvironment::setGoalArea(std::vector<double>& goal_area)
{
    goal_area_ = goal_area;    
}

bool RobotEnvironment::loadGoalArea(std::string& env_file)
{
    if (!frapu::fileExists(env_file)) {
        cout << "Utils: ERROR: Environment file '" << env_file << "' doesn't exist" << endl;
        return false;
    }
    goal_area_.clear();
    TiXmlDocument xml_doc;
    xml_doc.LoadFile(env_file);
    TiXmlElement* env_xml = xml_doc.FirstChildElement("Environment");
    for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody")) {
        std::string name(obst_xml->Attribute("name"));
        if (name == "GoalArea") {
            TiXmlElement* body_xml = obst_xml->FirstChildElement("Body");
            if (body_xml) {
                TiXmlElement* geom_xml = body_xml->FirstChildElement("Geom");
                if (geom_xml) {
                    TiXmlElement* trans_xml = geom_xml->FirstChildElement("Translation");
                    if (trans_xml) {
                        const char* xyz_str = trans_xml->GetText();
                        std::vector<std::string> pieces;
                        boost::split(pieces, xyz_str, boost::is_any_of(" "));
                        for (unsigned int i = 0; i < pieces.size(); ++i) {
                            if (pieces[i] != "") {
                                goal_area_.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                            }
                        }
                    }
                    TiXmlElement* radius_xml = geom_xml->FirstChildElement("Radius");
                    if (radius_xml) {
                        const char* rad_str = radius_xml->GetText();
                        goal_area_.push_back(boost::lexical_cast<double>(radius_xml->GetText()));
                    }
                }
            }
        }
    }   
    
    return true;
}

frapu::ObstacleSharedPtr RobotEnvironment::makeObstacle(std::string obstacleName,
        std::string obstacleType,
        std::vector<double>& dims,
        bool traversable,
        double traversalCost,
        bool observable) const
{
    frapu::TerrainSharedPtr terrain = std::make_shared<frapu::TerrainImpl>("terr",
                                      traversalCost,
                                      0.0,
                                      traversable,
                                      observable);

    frapu::ObstacleSharedPtr obstacle;
    if (obstacleType == "box") {
        obstacle = std::make_shared<frapu::BoxObstacle>(obstacleName,
                   dims[0],
                   dims[1],
                   dims[2],
                   dims[3],
                   dims[4],
                   dims[5],
                   terrain);
    }

    else if (obstacleType == "sphere") {
        obstacle = std::make_shared<frapu::SphereObstacle>(obstacleName,
                   dims[0],
                   dims[1],
                   dims[2],
                   dims[3],
                   terrain);

    } else {
        cout << "RobotEnvironment: Error: Can't create obstacle with type " << obstacleType << ". Type not recognized." << endl;
        return nullptr;
    }

    return obstacle;
}

frapu::ObstacleSharedPtr RobotEnvironment::getObstacle(std::string name)
{
    return scene_->getObstacle(name);    
}

bool RobotEnvironment::addObstacle(frapu::ObstacleSharedPtr& obstacle)
{    
    scene_->addObstacle(obstacle);
    std::vector<double> dimensions;
    static_cast<frapu::ObstacleImpl*>(obstacle.get())->getDimensions(dimensions);
    robot_->addBox(obstacle->getName(), dimensions);
}

bool RobotEnvironment::removeObstacles(std::vector<std::string>& obstacle_names)
{
    return scene_->removeObstacles(obstacle_names);
}

bool RobotEnvironment::removeObstacle(std::string obstacleName)
{
    return scene_->removeObstacle(obstacleName);
}

frapu::EnvironmentInfoSharedPtr RobotEnvironment::getEnvironmentInfo() const
{
    return environmentInfo_;
}

/**BOOST_PYTHON_MODULE(librobot_environment)
{
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

    class_<std::vector<std::shared_ptr<frapu::ObstacleWrapper>> > ("v_obstacle")
    .def(vector_indexing_suite<std::vector<std::shared_ptr<frapu::ObstacleWrapper>> >());
    to_python_converter < std::vector<std::shared_ptr<frapu::ObstacleWrapper>, std::allocator<std::shared_ptr<frapu::ObstacleWrapper>> >,
                        VecToList<std::shared_ptr<frapu::ObstacleWrapper>> > ();
    register_ptr_to_python<std::shared_ptr<frapu::ObstacleWrapper>>();

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

    class_<RobotEnvironment, std::shared_ptr<frapu::RobotEnvironment> >("RobotEnvironment", init<>())
    .def("getRobot", &RobotEnvironment::getRobot)
    .def("createManipulatorRobot", &RobotEnvironment::createManipulatorRobot)
    .def("createDubinRobot", &RobotEnvironment::createDubinRobot)
    .def("getGoalArea", &RobotEnvironment::getGoalArea)
    .def("loadEnvironment", &RobotEnvironment::loadEnvironment)
    .def("getObstacles", &RobotEnvironment::getObstaclesPy)

    ;
}*/



}
