#include "include/Terrain.hpp"

namespace frapu
{

TerrainImpl::TerrainImpl(const std::string name,
                         const double traversalCost,
                         const double velocityDamping,
                         bool traversable,
                         bool observable):
    Terrain(traversable, observable),
    name_(name),
    traversalCost_(traversalCost),
    velocityDamping_(velocityDamping)
{

}

const std::string TerrainImpl::getName() const
{
    return name_;
}

const double TerrainImpl::getTraversalCost() const
{
    return traversalCost_;
}

const double TerrainImpl::getVelocityDamping() const
{
    return velocityDamping_;
}

}
