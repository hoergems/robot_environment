#include "include/Terrain.hpp"

namespace shared
{

Terrain::Terrain(const std::string name,
                 const double traversalCost,
                 const double velocityDamping,
                 bool traversable,
                 bool observable):
    frapu::Terrain(traversable, observable),
    name_(name),
    traversalCost_(traversalCost),
    velocityDamping_(velocityDamping)    
{

}

const std::string Terrain::getName() const
{
    return name_;
}

const double Terrain::getTraversalCost() const
{
    return traversalCost_;
}

const double Terrain::getVelocityDamping() const
{
    return velocityDamping_;
}

}
