#ifndef TERRAIN_HPP_
#define TERRAIN_HPP_
#include <string>
#include <frapu_core/core.hpp>

namespace frapu
{

class TerrainImpl: public Terrain
{
public:
    TerrainImpl(const std::string name,
            const double traversalCost,
            const double velocityDamping,
            bool traversable,
            bool observable);

    ~TerrainImpl() = default;

    const std::string getName() const;

    const double getTraversalCost() const;

    const double getVelocityDamping() const;

private:
    const std::string name_;
    const double traversalCost_;
    const double velocityDamping_;    
};

}

#endif
