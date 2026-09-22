
#include "MyProject.h"


/******************************************************************************/
float shaft_angle[3];//!< current motor angle [0]=M1 [1]=M2 [2]=M3
float electrical_angle[3];
float shaft_velocity[3];
float current_sp;
float shaft_velocity_sp[3];
float shaft_angle_sp[3];
DQVoltage_s voltage[3];
DQCurrent_s current;

TorqueControlType torque_controller;
MotionControlType controller;

float sensor_offset=0;
float zero_electric_angle;
/******************************************************************************/
float electricalAngle(int motor)
{
  return _normalizeAngle((shaft_angle[motor] + sensor_offset) * pole_pairs - zero_electric_angle);
}
/******************************************************************************/


