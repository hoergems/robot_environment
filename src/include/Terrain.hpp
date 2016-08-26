#ifndef TERRAIN_HPP_
#define TERRAIN_HPP_
#include <string>
#include <frapu_core/core.hpp>

namespace shared
{

class Terrain: public frapu::Terrain
{
public:
    Terrain(const std::string name,
            const double traversalCost,
            const double velocityDamping,
            bool traversable,
            bool observable);

    ~Terrain() = default;

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
