#include "include/Obstacle.hpp"
#include <iostream>
#include <boost/python/converter/registry.hpp>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;
using namespace boost::python;

namespace shared
{

template<class T>
struct VecToList {
    static PyObject* convert(const std::vector<T>& vec) {
        boost::python::list* l = new boost::python::list();
        for (size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

Obstacle::Obstacle(std::string& name, frapu::TerrainSharedPtr& terrain):
    frapu::Obstacle(name, terrain),
    diffuse_color_(),
    ambient_color_()
{

}

void Obstacle::setStandardColor(std::vector<double>& diffuseColor,
                                std::vector<double>& ambientColor)
{
    diffuse_color_.clear();
    ambient_color_.clear();

    if (diffuseColor.size() == 0) {
        diffuse_color_.push_back(0.0);
        diffuse_color_.push_back(0.0);
        diffuse_color_.push_back(0.0);
    }

    if (ambientColor.size() == 0) {
        ambient_color_.push_back(0.0);
        ambient_color_.push_back(0.0);
        ambient_color_.push_back(0.0);
    }

    for (size_t i = 0; i < diffuseColor.size(); i++) {
        diffuse_color_.push_back(diffuseColor[i]);
    }

    for (size_t i = 0; i < ambientColor.size(); i++) {
        ambient_color_.push_back(ambientColor[i]);
    }
}

bool Obstacle::isTraversable() const
{
    return terrain_->isTraversable();
}

double Obstacle::getExternalForce()
{
    return static_cast<shared::Terrain*>(terrain_.get())->getVelocityDamping();
}

bool Obstacle::in_collision(std::vector<double>& point)
{
    Vec3f p_vec(point[0], point[1], point[2]);
    return collisionObject_->getAABB().contain(p_vec);
}

bool Obstacle::in_collision_point(std::vector<double>& point)
{
    Vec3f p_vec(point[0], point[1], point[2]);
    return collisionObject_->getAABB().contain(p_vec);
}

double Obstacle::distance(std::vector<frapu::CollisionObjectSharedPtr>& other_collision_objects) const
{
    double min_distance = 1000000.0;
    for (size_t i = 0; i < other_collision_objects.size(); i++) {
        fcl::DistanceRequest request;
        fcl::DistanceResult result;
        fcl::distance(other_collision_objects[i].get(),
                      collisionObject_.get(),
                      request,
                      result);
        if (result.min_distance < min_distance) {
            min_distance = result.min_distance;
        }
    }

    return min_distance;
}

bool Obstacle::inCollision(std::vector<frapu::CollisionObjectSharedPtr>& collisionObjects) const
{
    for (size_t i = 0; i < collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(collisionObjects[i].get(),
                     collisionObject_.get(),
                     request,
                     result);
        if (result.isCollision()) {
            return true;
        }
    }

    return false;
}

bool Obstacle::in_collision(const std::vector<std::shared_ptr<Obstacle> >& other_obstacles) const
{
    for (size_t i = 0; i < other_obstacles.size(); i++) {
        if (collisionObject_->getAABB().overlap(other_obstacles[i]->getCollisionObject()->getAABB())) {
            return true;
        }
    }

    return false;
}

bool Obstacle::inCollisionContinuous(frapu::CollisionObjectSharedPtr& collisionObjectStart,
                                     frapu::CollisionObjectSharedPtr& collisionObjectGoal) const
{
    fcl::ContinuousCollisionRequest request(10,
                                            0.0001,
                                            CCDM_LINEAR,
                                            GST_LIBCCD,
                                            CCDC_NAIVE);
    fcl::ContinuousCollisionResult result;
    fcl::continuousCollide(collisionObjectStart.get(),
                           collisionObjectGoal->getTransform(),
                           collisionObject_.get(),
                           collisionObject_->getTransform(),
                           request,
                           result);
    return result.is_collide;
}

double Obstacle::distancePy(boost::python::list& ns)
{
    std::vector<frapu::CollisionObjectSharedPtr> other_collision_objects;
    for (int i = 0; i < len(ns); ++i) {
        other_collision_objects.push_back(boost::python::extract<frapu::CollisionObjectSharedPtr>(ns[i]));
    }

    return distance(other_collision_objects);
}

bool Obstacle::in_collision_discrete(boost::python::list& ns)
{
    std::vector<frapu::CollisionObjectSharedPtr> other_collision_objects;
    for (int i = 0; i < len(ns); ++i) {
        other_collision_objects.push_back(boost::python::extract<frapu::CollisionObjectSharedPtr>(ns[i]));
    }

    return inCollision(other_collision_objects);
}

bool Obstacle::in_collision_continuous(boost::python::list& ns)
{

    frapu::CollisionObjectSharedPtr collision_object_start =
        boost::python::extract<frapu::CollisionObjectSharedPtr>(ns[0]);
    frapu::CollisionObjectSharedPtr collision_object_goal =
        boost::python::extract<frapu::CollisionObjectSharedPtr>(ns[1]);
    return inCollisionContinuous(collision_object_start, collision_object_goal);
}

std::vector<double> Obstacle::getStandardDiffuseColor()
{
    return diffuse_color_;
}

std::vector<double> Obstacle::getStandardAmbientColor()
{
    return ambient_color_;
}

/**BOOST_PYTHON_MODULE(libobstacle)
{
#include "include/Terrain.hpp"
    bool (ObstacleWrapper::*in_collision_d)(boost::python::list&) = &ObstacleWrapper::in_collision_discrete;
    bool (ObstacleWrapper::*in_collision_c)(boost::python::list&) = &ObstacleWrapper::in_collision_continuous;
    bool (ObstacleWrapper::*in_collision_p)(std::vector<double>&) = &ObstacleWrapper::in_collision_point;
    //double (ObstacleWrapper::*distance_d)(boost::python::list&) = &ObstacleWrapper::distancePy;

    boost::python::type_info info = boost::python::type_id<std::vector<std::shared_ptr<shared::ObstacleWrapper>>>();
    const boost::python::converter::registration* reg_vobst = boost::python::converter::registry::query(info);
    if (reg_vobst == NULL || (*reg_vobst).m_to_python == NULL)  {
        class_<std::vector<std::shared_ptr<shared::ObstacleWrapper>> > ("v_obstacle")
                .def(vector_indexing_suite<std::vector<std::shared_ptr<shared::ObstacleWrapper>> >());
        to_python_converter < std::vector<std::shared_ptr<shared::ObstacleWrapper>, std::allocator<std::shared_ptr<shared::ObstacleWrapper>> >,
                            VecToList<std::shared_ptr<shared::ObstacleWrapper>> > ();
        register_ptr_to_python<std::shared_ptr<shared::ObstacleWrapper>>();
    }

    class_<Terrain>("Terrain", init<const std::string, const double, const double, const bool, bool>())
    .def("getTraversalCost", &Terrain::getTraversalCost)
    .def("getName", &Terrain::getName)
    .def("getVelocityDamping", &Terrain::getVelocityDamping)
    .def("isTraversable", &Terrain::isTraversable)
    .def("isObservable", &Terrain::isObservable)
    ;

    class_<ObstacleWrapper, boost::noncopyable>("Obstacle", init<std::string, Terrain>())
    .def("inCollisionDiscrete", in_collision_d)
    .def("inCollisionContinuous", in_collision_c)
    .def("inCollisionPoint", in_collision_p)
    .def("isTraversable", &ObstacleWrapper::isTraversable)
    .def("getExternalForce", &ObstacleWrapper::getExternalForce)
    .def("createCollisionObject", boost::python::pure_virtual(&ObstacleWrapper::createCollisionObject))
    .def("getName", &ObstacleWrapper::getName)
    .def("getDimensions", &ObstacleWrapper::getDimensions)
    .def("getStandardDiffuseColor", &ObstacleWrapper::getStandardDiffuseColor)
    .def("getStandardAmbientColor", &ObstacleWrapper::getStandardAmbientColor)
    .def("distance", &ObstacleWrapper::distancePy)
    ;
}*/

}
