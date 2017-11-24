
#include <controller_manager_tests/extensible_controllers.h>

using namespace controller_manager_tests;

bool ExtensibleController::init(hardware_interface::RobotHW* robot_hw,
          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::string vel_joint_name;
  controller_nh.getParam("velocity_joint", vel_joint_name);

  hardware_interface::VelocityJointInterface* joint_iface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  joint_iface->getHandle(vel_joint_name);
  return true;
}

int ExtensibleController::helper()
{
  return 1;
}

void ExtensibleController::update(const ros::Time&, const ros::Duration&)
{
  // TODO: Publish this return value so that the test can validate it?
  int foo = helper();
}


bool DerivedController::initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
    controller_interface::ControllerBase::ClaimedResources& cr)
{
  return DerivedControllerInterface::initRequest(hw, nh, pnh, cr);
}


bool DerivedController::init(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // First initialize the base controller.
  if (!ExtensibleController::init(robot_hw, root_nh, controller_nh))
  {
    return false;
  }

  std::string eff_joint_name;
  controller_nh.getParam("effort_joint", eff_joint_name);

  hardware_interface::EffortJointInterface* joint_iface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_iface->getHandle(eff_joint_name);
  return true;
}

int DerivedController::helper()
{
  return 2;
}

PLUGINLIB_EXPORT_CLASS(controller_manager_tests::ExtensibleController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(controller_manager_tests::DerivedController, controller_interface::ControllerBase)
