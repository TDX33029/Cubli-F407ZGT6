#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "foc_utils.h"
/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
	Type_torque,//!< Torque control
	Type_velocity,//!< Velocity motion control
	Type_angle,//!< Position/angle motion control
	Type_velocity_openloop,
	Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
	Type_voltage, //!< Torque control using voltage
	Type_dc_current, //!< Torque control using DC current (one current magnitude)
	Type_foc_current //!< torque control using dq currents
} TorqueControlType;

extern TorqueControlType torque_controller;
extern MotionControlType controller;
/******************************************************************************/
extern float shaft_angle[3];//!< current motor angle [0]=M1, [1]=M2, [2]=M3
extern float electrical_angle[3];
extern float shaft_velocity[3];
extern float current_sp;
extern float shaft_velocity_sp[3];
extern float shaft_angle_sp[3];
extern DQVoltage_s voltage[3];
extern DQCurrent_s current;

extern float sensor_offset;
extern float zero_electric_angle;
/******************************************************************************/
float shaftAngle(void);
float shaftVelocity(void);
float electricalAngle(int motor);
/******************************************************************************/

#endif

