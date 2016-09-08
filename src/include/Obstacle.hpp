/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_
#include <vector>
#include <unistd.h>
#include <memory>
#include "Terrain.hpp"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/continuous_collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <mutex>
#include <frapu_core/core.hpp>

using std::cout;
using std::endl;

namespace frapu
{

class ObstacleImpl: public frapu::Obstacle
{
public:
    ObstacleImpl(std::string& name, frapu::TerrainSharedPtr& terrain);
    
    ~ObstacleImpl() = default;

    //void set_dimensions(std::vector<double> &position, std::vector<double>)

    /**
     * Checks if this obstacle collides with another obstacle
     * */
    bool in_collision(const std::vector<std::shared_ptr<ObstacleImpl> >& other_obstacles) const;

    //bool in_collision(std::vector<fcl::AABB> &other_collision_structures) const;

    virtual bool inCollision(std::vector<frapu::CollisionObjectSharedPtr> &collisionObjects) const override;

    /**
     * Computes the smallest distance between the obstacle and the given collision objects
     */
    double distance(std::vector<frapu::CollisionObjectSharedPtr>& other_collision_objects) const;

    /**
     * Checks if the obstacle collides with another moving collision object.
     * The motion of of the other collision object is determined by a start
     * and goal transformation
     */
    virtual bool inCollisionContinuous(frapu::CollisionObjectSharedPtr& collisionObjectStart,
                                       frapu::CollisionObjectSharedPtr& collisionObjectGoal) const override;    

    /**
     * Checks if a point lies withing this obstacle
     */
    bool in_collision(std::vector<double>& point);

    virtual bool in_collision_point(std::vector<double>& point);    
    
    /**
     * Gets the external force (proportional to the end effector velocity)
     * the underlying obstacles induces on the end effector.
     */
    virtual double getExternalForce();

    /**
     * Determines if the obstacle is traversable
     */
    bool isTraversable() const;    

    /**
     * Set the obstacle's standard color
     */
    virtual void setStandardColor(std::vector<double>& diffuseColor,
                                  std::vector<double>& ambientColor);

    /**
     * Get the standard diffuse color
     */
    virtual std::vector<double> getStandardDiffuseColor();

    /**
     * Get the standard ambient color
     */
    virtual std::vector<double> getStandardAmbientColor();

protected:
    std::vector<double> diffuse_color_;

    std::vector<double> ambient_color_;
    
};

/**
 * A Python wrapper to handle polymorphism
 */
/**struct ObstacleWrapper: Obstacle, boost::python::wrapper<Obstacle> {
public:
    ObstacleWrapper(std::string name, const Terrain& terrain):
        Obstacle(name, terrain) {
    }

    void createCollisionObject() {
        this->get_override("createCollisionObject")();
    }

    double distancePy(boost::python::list& ns) {
        this->get_override("distancePy")(ns);
    }

    bool in_collision_discrete(boost::python::list& ns) {
        this->get_override("in_collision_discrete")(ns);
    }

    bool in_collision_continuous(boost::python::list& ns) {
        this->get_override("in_collision_continuous")(ns);
    }

    bool in_collision_point(std::vector<double>& point) {
        this->get_override("in_collision_point")(point);
    }

    double getExternalForce() {
        this->get_override("getExternalForce")();
    }

    std::string getName() {
        this->get_override("getName")();
    }

    void getDimensions(std::vector<double>& dimensions) {
        this->get_override("getDimensions")(dimensions);
    }

    std::vector<double> getStandardDiffuseColor() {
        this->get_override("getStandardDiffuseColor")();
    }

    std::vector<double> getStandardAmbientColor() {
        this->get_override("getStandardAmbientColor")();
    }

};*/

}

#endif /* OBSTACLE_HPP_ */
