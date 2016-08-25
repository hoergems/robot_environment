#include "include/Terrain.hpp"

namespace shared {

Terrain::Terrain(const std::string name, 
                 const double traversalCost, 
                 const double velocityDamping,
                 const bool traversable,
		 bool observable) :
    name_(name),
    traversalCost_(traversalCost),
    velocityDamping_(velocityDamping),
    traversable_(traversable),
    observable_(observable)
{

}

const std::string Terrain::getName() const {
    return name_;
}

const double Terrain::getTraversalCost() const {
    return traversalCost_;
}

const double Terrain::getVelocityDamping() const {
    return velocityDamping_;
}

const bool Terrain::isTraversable() const {
    return traversable_;
}

bool Terrain::isObservable() const {
    return observable_;
}

}
