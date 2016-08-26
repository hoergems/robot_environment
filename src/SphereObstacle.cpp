#include "include/SphereObstacle.hpp"
#include <iostream>

using std::cout;
using std::endl;

using namespace fcl;

namespace shared
{

SphereObstacle::SphereObstacle(std::string name,
                               double pos_x,
                               double pos_y,
                               double pos_z,
                               double radius,
                               frapu::TerrainSharedPtr& terrain):
    Obstacle(name, terrain),
    pos_x_(pos_x),
    pos_y_(pos_y),
    pos_z_(pos_z),
    radius_(radius)
{
    createCollisionObject();
}

void SphereObstacle::getDimensions(std::vector<double>& dimensions)
{
    dimensions.clear();
    dimensions.resize(4);
    dimensions[0] = pos_x_;
    dimensions[1] = pos_y_;
    dimensions[2] = pos_z_;
    dimensions[3] = radius_;
}

void SphereObstacle::createCollisionObject()
{
    Sphere* sphere = new Sphere(radius_);
    Vec3f trans(pos_x_, pos_y_, pos_z_);
    Matrix3f rot(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0);
    Transform3f rotate_translate(rot, trans);
    collisionObject_ = std::make_shared<fcl::CollisionObject>(fcl::CollisionObject(boost::shared_ptr<CollisionGeometry>(sphere),
                       rotate_translate));

}

}
