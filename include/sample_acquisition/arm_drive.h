/* arm_drive.h
 * [ml]
 */

#ifndef _ARM_DRIVE_H
#define _ARM_DRIVE_H

#include "ros/ros.h"
#include "phidgets/stepper_params.h"
#include "sample_acquisition/arm_restrictor.h"
#include "sample_acquisition/ArmMovement.h"
#include "sample_acquisition/stepper_helper.h"

using namespace std;

enum JOINT
{
  PAN_JOINT=0,
  TILT_JOINT=1,
  CABLE_JOINT=2
};

// Class declaration
class ArmDrive
{
public:
    // Constructor. Initializes all variables.
    ArmDrive( const ros::NodeHandle&, string pan_motor_serial, string tilt_motor_serial, string cable_motor_serial, 
                float velocity_mode_time_step, float pan_motor_safe_velocity, float tilt_motor_safe_velocity, float cable_motor_safe_velocity,
                float pan_motor_max_velocity, float tilt_motor_max_velocity, float cable_motor_max_velocity, float all_motors_acceleration );

    // Callbacks
    void movementCallback( const sample_acquisition::ArmMovementConstPtr & );

    // Output to the driver
    bool initializeMotors();

    // Mode getter
    string getMode();

    StepperHelper *steppers[3];

    float getPanPos( bool *at_max );
    float getTiltPos( bool *at_max );
    float getCablePos( bool *at_max );

private:
    // ROS data members.
    ros::NodeHandle nnh;

    ros::Subscriber arm_movement_sub;

    bool pan_init, tilt_init, cable_init;

    // Acceleration to be used by all motors.
    float all_motors_accel;

    // Last mode used.
    string mode;

    // Time step for velocity mode.
    float vel_mode_time_step;

    // Data member used to restrict motion based on the yaml file.
    ArmRestrictor* restrictor;
};
#endif
