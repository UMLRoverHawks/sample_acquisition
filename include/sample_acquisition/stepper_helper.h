/* arm_drive.h
 * [ml]
 */

#ifndef _STEPPER_HELPER_H
#define _STEPPER_HELPER_H

#include "ros/ros.h"
#include "phidgets/stepper_params.h"
#include "sample_acquisition/ArmMovement.h"

using namespace std;

// Class declaration
class StepperHelper
{
public:
    // Constructor. Initializes all variables.
    StepperHelper( const ros::NodeHandle&, string description, string serial, float motor_safe_velocity, float motor_max_velocity, float all_motors_acceleration );

    // Callbacks
    void callback( const phidgets::stepper_paramsConstPtr & );

    void setMotor( bool );

    // Disengage motor functions
    void disengage();

    // getters
    long long getPos();


    void setTarget(long long f);
    void setVel(float f);

    // Velocity getters
    float getVel();

    float getAccel();

    float getEngaged();

    void usePositionVel();
    void useVelocityVel(float coeff);

private:
    // Indicates whether initialization has occured.
    bool init;

    // ROS data members.
    ros::NodeHandle nnh;

    ros::Publisher motor_pub;

    ros::Subscriber motor_sub;

    // Last position command to be output.
    long long motor_target;

    // Actual position of the motors.
    long long motor_pos;

    // Last velocity command to be output.
    float motor_vel;

    // Velocity used in position mode.
    float motor_safe_vel;

    // Maximum velocities.
    float motor_max_vel;

    // Acceleration to be used by all motors.
    float all_motors_accel;

    string desc;
};
#endif
