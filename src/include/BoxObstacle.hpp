/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef BOX_OBSTACLE_HPP_
#define BOX_OBSTACLE_HPP_
#include "Obstacle.hpp"
#include <vector>
#include <unistd.h>
#include <memory>
#include "Terrain.hpp"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/continuous_collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace frapu
{

class BoxObstacle: public ObstacleImpl
{
public:
    BoxObstacle(std::string name,
                double pos_x,
                double pos_y,
                double pos_z,
                double size_x,
                double size_y,
                double size_z,
                frapu::TerrainSharedPtr &terrain);

    virtual void createCollisionObject() override;

    virtual void getDimensions(std::vector<double>& dimensions) const override;

    double pos_x_;
    double pos_y_;
    double pos_z_;
    double size_x_;
    double size_y_;
    double size_z_;

};

}

#endif /* OBSTACLE_HPP_ */
