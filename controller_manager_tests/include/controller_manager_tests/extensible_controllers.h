#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace controller_manager_tests
{

/**
 * This controller supplies an intentional extension point in the form of the virtual
 * "helper" function that the update method calls.
 */
typedef controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface> BaseControllerInterface;
class ExtensibleController : public virtual BaseControllerInterface
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual int helper();
  void update(const ros::Time&, const ros::Duration&);
};

/**
 * The derived controller not only overrides the virtual helper method, it also adds an additional
 * hardware interface, in order to demonstrate the flexbility of this mechanism.
 */
typedef controller_interface::MultiInterfaceController<
    hardware_interface::VelocityJointInterface, hardware_interface::EffortJointInterface> DerivedControllerInterface;
class DerivedController : public ExtensibleController, public DerivedControllerInterface
{
public:
  bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
      controller_interface::ControllerBase::ClaimedResources& cr);
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual int helper();
};

}  // namespace controller_manager_tests
