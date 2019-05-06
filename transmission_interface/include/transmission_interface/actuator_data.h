#pragma once

#include <vector>
#include <algorithm>

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

class ActuatorDataBase 
{
public:
virtual bool empty() const = 0;
virtual bool hasSize(std::size_t size) const = 0;
virtual bool valid() const = 0;

inline bool hasValidPointers(const std::vector<double*>& data) const
{
return std::all_of(data.cbegin(), data.cend(), [](const double* ptr){ return ptr;});
}
};

template<typename... Interfaces>
struct ActuatorDataContainer : public ActuatorDataBase, public Interfaces... 
{
  public:
    ActuatorDataContainer(){}
    ActuatorDataContainer(Interfaces... ifaces) : Interfaces(ifaces)... {}
    virtual bool empty() const = 0;
    virtual bool hasSize(std::size_t size) const = 0;
    virtual bool valid() const = 0;
};


/**
 * \brief Contains pointers to raw data representing the position, velocity and acceleration of a transmission's
 * actuators.
 */
class ActuatorData : public ActuatorDataContainer<PositionActuatorData, VelocityActuatorData, EffortActuatorData>
{
public:
    bool empty() const override
    {
        return position.empty() && velocity.empty() && effort.empty();
    }

    bool hasSize(std::size_t size) const override
    {
        return ((not empty()) and (position.size() == size) and (velocity.size() == size) and (effort.size() == size));
    }

    bool valid() const override
    {
        return hasValidPointers(position) and hasValidPointers(velocity) and hasValidPointers(effort);
    }
};

} // namespace transmission_interface
