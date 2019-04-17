#pragma once

#include <vector>

namespace transmission_interface
{

struct PositionActuatorData
{
    std::vector<double*> position;
};

struct VelocityActuatorData
{
    std::vector<double*> velocity;
};

struct EffortActuatorData
{
  std::vector<double*> effort;
};

class ActuatorBase {};

template<typename... Interfaces>
struct ActuatorDataContainer : public ActuatorBase, public Interfaces... 
{
  public:
    ActuatorDataContainer(){}
    ActuatorDataContainer(Interfaces... ifaces) : Interfaces(ifaces)... {}
};


/**
 * \brief Contains pointers to raw data representing the position, velocity and acceleration of a transmission's
 * actuators.
 */
using ActuatorData = ActuatorDataContainer<PositionActuatorData, VelocityActuatorData, EffortActuatorData>;

} // namespace transmission_interface
