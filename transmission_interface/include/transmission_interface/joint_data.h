#pragma once

#include <vector>
#include <algorithm>

namespace transmission_interface
{
struct PositionJointData
{
    std::vector<double*> position;
};

struct VelocityJointData
{
    std::vector<double*> velocity;
};

struct EffortJointData
{
    std::vector<double*> effort;
};

class JointDataBase
{
  public:
    virtual bool empty() const = 0;
    virtual bool hasSize(std::size_t size) const = 0;
    virtual bool valid() const = 0;

    inline bool hasValidPointers(const std::vector<double*>& data) const
    {
        return std::all_of(data.cbegin(), data.cend(), [](const double* ptr) { return ptr; });
    }
};

template <typename... Interfaces>
struct JointDataContainer : public JointDataBase, public Interfaces...
{
  public:
    JointDataContainer()
    {
    }
    JointDataContainer(Interfaces... ifaces) : Interfaces(ifaces)...
    {
    }
    virtual bool empty() const = 0;
    virtual bool hasSize(std::size_t size) const = 0;
    virtual bool valid() const = 0;
};

/**
 * \brief Contains pointers to raw data representing the position, velocity and acceleration of a transmission's
 * joints.
 */
class JointData : public JointDataContainer<PositionJointData, VelocityJointData, EffortJointData>
{
  public:
    bool empty() const override
    {
        return position.empty() && velocity.empty() && effort.empty();
    }

    bool hasSize(std::size_t size) const override
    {
        if (not position.empty() and position.size() != size)
        {
            return false;
        }

        if (not velocity.empty() and velocity.size() != size)
        {
            return false;
        }

        if (not effort.empty() and effort.size() != size)
        {
            return false;
        }

        return true;
    }

    bool valid() const override
    {
        return hasValidPointers(position) and hasValidPointers(velocity) and hasValidPointers(effort);
    }
};

}  // namespace transmission_interface