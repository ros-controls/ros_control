## Transmission Interface ##

### Overview ###

**transmission_interface** contains data structures for representing mechanical transmissions, and methods for
propagating position, velocity and effort variables between actuator and joint spaces.

In the same spirit as the **hardware_interface** package, this package wraps existing raw data (eg. current actuator
position, reference joint command, etc.) under a consistent interface. By not imposing a specific layout on the raw data,
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

### Examples ###
Please refer to the  [transmission_interface](https://github.com/ros-controls/ros_control/wiki/transmission_interface) wiki page.
