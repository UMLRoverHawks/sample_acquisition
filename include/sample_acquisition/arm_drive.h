/* arm_drive.h
 * [ml]
 */

#ifndef _ARM_DRIVE_H
#define _ARM_DRIVE_H

#include "ros/ros.h"
#include "phidgets/stepper_params.h"
#include "sample_acquisition/ArmMovement.h"
#include "sample_acquisition/stepper_helper.h"
#include "std_msgs/Bool.h"

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

   //Callback for the activation subscriber.
    void activationCallback ( const std_msgs::Bool::ConstPtr& );

    // Output to the driver
    bool initializeMotors();

    // Mode getter
    string getMode();

    StepperHelper *steppers[3];

private:
    // ROS data members.
    ros::NodeHandle nnh;

    ros::Subscriber arm_movement_sub;

    ros::Subscriber arm_activation_sub; //Boolean subscriber to /arm/on

    bool pan_init, tilt_init, cable_init;

    bool arm_activated; //Boolean used to know wheter the motors should be activated or not, it is set by the activationCallback function, it is false at initialization

    // Acceleration to be used by all motors.
    float all_motors_accel;

    // Last mode used.
    string mode;

    // Time step for velocity mode.
    float vel_mode_time_step;
};
#endif
