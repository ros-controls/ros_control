## Transmission Interface ##

### Overview ###

**transmission_interface** contains data structures for representing mechanical transmissions, and methods for
propagating position, velocity and effort variables between actuator and joint spaces.

In the same spirit as the **hardware_interface** package, this package wraps existing raw data (eg. current actuator
positon, reference joint command, etc.) under a consistent interface. By not imposing a specific layout on the raw data,
it becomes easier to support arbitrary hardware drivers to software control.

### Structure ###

There are three main elements involved in setting up a transmission_interface:
  - The **Transmission** class defines an abstract interface for mapping
  position, velocity and effort variables between actuator and joint space.
  Derived classes implement specific transmission types, such as
  simple reducers, differentials and four-bar linkages.
  Note that a single transmission may couple the variables of multiple actuators and joints (eg. a differential couples
  two actuators to two joints).

  - The **TransmissionHandle** class associates a name to a
  Transmission instance and a set of raw data variables
  it operates on (specified through the **ActuatorData** and
  **JointData** structures).
  Derived classes implement specific maps, such as
  **JointToActuatorEffortHandle** or
  **ActuatorToJointStateHandle**.

  - The **TransmissionInterface<HandleType>** class manages
  a set of transmission handles of the same type.
  Note that although the handles are of the same type, the underlying transmissions can be heterogeneous
  eg. a single **ActuatorToJointPositionInterface** can be
  set up to transform position variables from actuator to joint space for an arm with a four-bar-linkage in the
  shoulder, a differential in the wrist, and simple reducers elsewhere.

### Example ###

The following is an example of a robot with three actuators and three joints:
  - The first actuator/joint are coupled through a reducer.
  - The last two actuators/joints are coupled through a differential.

The hardware is such that one can read current actuator position, velocity and effort, and send position commands.

```c++
#include <vector>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

using std::vector;
using namespace transmission_interface;

class MyRobot
{
public:
  MyRobot()
   : sim_trans(-10.0,  // 10x reducer. Negative sign: actuator and joint spin in opposite directions
                 1.0), // joint position offset

     dif_trans(vector<double>(2, 5.0), // 5x reducer on each actuator
               vector<double>(2, 1.0)) // No reducer in joint output
  {
    // Wrapping the raw data is the most verbose part of the setup process... //////////////////////////////////////////

    // Wrap simple transmission raw data - current state
    a_state_data[0].position.push_back(&a_curr_pos[0]);
    a_state_data[0].velocity.push_back(&a_curr_vel[0]);
    a_state_data[0].effort.push_back(&a_curr_eff[0]);

    j_state_data[0].position.push_back(&j_curr_pos[0]);
    j_state_data[0].velocity.push_back(&j_curr_vel[0]);
    j_state_data[0].effort.push_back(&j_curr_eff[0]);

    // Wrap simple transmission raw data - position command
    a_cmd_data[0].position.push_back(&a_cmd_pos[0]); // Velocity and effort vectors are unused

    j_cmd_data[0].position.push_back(&j_cmd_pos[0]); // Velocity and effort vectors are unused

    // Wrap differential transmission raw data - current state
    a_state_data[1].position.push_back(&a_curr_pos[1]); a_state_data[1].position.push_back(&a_curr_pos[2]);
    a_state_data[1].velocity.push_back(&a_curr_vel[1]); a_state_data[1].velocity.push_back(&a_curr_vel[2]);
    a_state_data[1].effort.push_back(&a_curr_eff[1]);   a_state_data[1].effort.push_back(&a_curr_eff[2]);

    j_state_data[1].position.push_back(&j_curr_pos[1]); j_state_data[1].position.push_back(&j_curr_pos[2]);
    j_state_data[1].velocity.push_back(&j_curr_vel[1]); j_state_data[1].velocity.push_back(&j_curr_vel[2]);
    j_state_data[1].effort.push_back(&j_curr_eff[1]);   j_state_data[1].effort.push_back(&j_curr_eff[2]);

    // Wrap differential transmission raw data - position command
    a_cmd_data[1].position.push_back(&a_cmd_pos[1]); a_cmd_data[1].position.push_back(&a_cmd_pos[2]);

    j_cmd_data[1].position.push_back(&j_cmd_pos[1]); j_cmd_data[1].position.push_back(&j_cmd_pos[2]);

    // ...once the raw data has been wrapped, the rest is straightforward //////////////////////////////////////////////

    // Register transmissions to each interface
    act_to_jnt_state.registerTransmission("sim_trans", &sim_trans, a_state_data[0], j_state_data[0]);
    act_to_jnt_state.registerTransmission("dif_trans", &dif_trans, a_state_data[1], j_state_data[1]);

    jnt_to_act_pos.registerTransmission("sim_trans", &sim_trans, a_cmd_data[0], j_cmd_data[0]);
    jnt_to_act_pos.registerTransmission("dif_trans", &dif_trans, a_cmd_data[1], j_cmd_data[1]);

    // Names must be unique within a single transmission interface, but a same name can be used in multiple interfaces,
    // as shown above
  }

  void read()
  {
    // Read actuator state from hardware
    // ...

    // Propagate current actuator state to joints
    act_to_jnt_state.propagate();
  }

  void write()
  {
    // Porpagate joint commands to actuators
    jnt_to_act_pos.propagate();

    // Send actuator command to hardware
    // ...
  }

private:

  // Transmission interfaces
  ActuatorToJointStateInterface    act_to_jnt_state; // For propagating current actuator state to joint space
  JointToActuatorPositionInterface jnt_to_act_pos;   // For propagating joint position commands to actuator space

  // Transmissions
  SimpleTransmission       sim_trans;
  DifferentialTransmission dif_trans;

  // Actuator and joint space variables: wrappers around raw data
  ActuatorData a_state_data[2]; // Size 2: One per transmission
  ActuatorData a_cmd_data[2];

  JointData j_state_data[2];
  JointData j_cmd_data[2];

  // Actuator and joint space variables - raw data:
  // The first actuator/joint are coupled through a reducer.
  // The last two actuators/joints are coupled through a differential.
  double a_curr_pos[3]; // Size 3: One per actuator
  double a_curr_vel[3];
  double a_curr_eff[3];
  double a_cmd_pos[3];

  double j_curr_pos[3]; // Size 3: One per joint
  double j_curr_vel[3];
  double j_curr_eff[3];
  double j_cmd_pos[3];
};

```
### TODO ###

- Read transmission configuration from configuration files.
