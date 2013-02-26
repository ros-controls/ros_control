## Transmission Interface ##

### Overview ###

This is an early draft of a mechanical transmission interface. Its objective is to allow to easily configue and
apply maps between joint and actuator space variables.

The design has expressly been made as simple and minimal as possible.

**Important note** The current implementation is limited to *fully actuated* transmission mechanisms.
An example of what this implies is that joint velocities can be computed directly from actuator velocities
(and vice-versa). In underactuated systems this is in general not the case, hence not only actuator velocities but
also actuator efforts might be required to compute joint velocities.

### Structure ###

There are three main elements in the transmission interface:
*
  - The **Transmission** class defines an abstract interface for mapping position, velocity and effort variables
    between actuator and joint space. Derived classes implement specific transmission types. The following types
    are already implemented:
      - SimpleTransmission
      - DifferentialTransmission
      - FourBarLinkageTransmission

  - The **\*Handle** classes implement the map of a single variable (position, velocity, effort) in one direction
    (joint->actuator or actuator->joint) for a single **Transmission** instance. Note that a single transmission
    might affect multiple joints and actuators.

  - The **\*Interface** classes keep track of a list of multiple **Handle**s of the same type. Useful for transforming
    all robot actuator positions from actuator to joint space, for example.

### Documentation and tests ###
- The Doxygen documentation of the abovementioned classes is fairly complete. In particular, the doc for the
  **Transmission**-derived classes make extensive use of images, HTML and Latex to describe the implemented maps.

- All code is unit tested with high coverage, so heavy refactoring without breaking stuff is possible.

### Example ###

This is a very simple example of a robot with two actuators and two joints. Each actuator is coupled to a single
joint through a reducer. Only the transmission\_interface is shown (ie. there is no hardware\_interface).

```c++
#include <vector>

#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission.h>

using namespace transmission_interface;

class MyRobot
{
public:
  MyRobot()
   : trans1(10.0, 1.0), // 10x reducer, 1.0 joint position offset
     trans2(-10.0)      // 10x reducer, actuator and joint spin in opposite directions
  {
    typedef vector<double*> Vec; // for brevity

    // Connect and register the transmission interface
    // Params: Transmission name, transmission instance, actuator data vector, joint data vector
    act_to_jnt_pos.registerTransmission("tr1", &trans1, Vec(1, &a_pos[0]), Vec(1, &j_pos[0]));
    act_to_jnt_vel.registerTransmission("tr1", &trans1, Vec(1, &a_vel[0]), Vec(1, &j_vel[0]));
    act_to_jnt_eff.registerTransmission("tr1", &trans1, Vec(1, &a_eff[0]), Vec(1, &j_eff[0]));
    jnt_to_act_pos.registerTransmission("tr1", &trans1, Vec(1, &a_cmd[0]), Vec(1, &j_cmd[0]));

    act_to_jnt_pos.registerTransmission("tr2", &trans2, Vec(1, &a_pos[1]), Vec(1, &j_pos[1]));
    act_to_jnt_vel.registerTransmission("tr2", &trans2, Vec(1, &a_vel[1]), Vec(1, &j_vel[1]));
    act_to_jnt_eff.registerTransmission("tr2", &trans2, Vec(1, &a_eff[1]), Vec(1, &j_eff[1]));
    jnt_to_act_pos.registerTransmission("tr2", &trans2, Vec(1, &a_cmd[1]), Vec(1, &j_cmd[1]));
  }

  void read()
  {
    // Read actuator state from hardware
    // ...

    // Propagate current actuator state to joints
    act_to_jnt_pos.propagate();
    act_to_jnt_vel.propagate();
    act_to_jnt_eff.propagate();
  }

  void write()
  {
    // Porpagate joint commands to actuators
    jnt_to_act_pos.propagate();

    // Send actuator command to hardware
    // ...
  }

private:
  ActuatorToJointPositionInterface act_to_jnt_pos;
  ActuatorToJointVelocityInterface act_to_jnt_vel;
  ActuatorToJointEffortInterface   act_to_jnt_eff;

  JointToActuatorPositionInterface jnt_to_act_pos;

  // Actuator space variables
  double a_cmd[2];
  double a_pos[2];
  double a_vel[2];
  double a_eff[2];

  // Joint space variables
  double j_cmd[2];
  double j_pos[2];
  double j_vel[2];
  double j_eff[2];

  // Transmission instances
  SimpleTransmission trans1;
  SimpleTransmission trans2;
};

```
### TODO ###

- Make the current implenentation more general so it can accomodate underactuated transmissions.
- Read transmission configuration from configuration files.
