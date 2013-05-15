/* arm_drive.h
 * [ml]
 */

#ifndef _ARM_DRIVE_H
#define _ARM_DRIVE_H

#include "ros/ros.h"
#include "phidgets/stepper_params_better.h"
#include "sample_acquisition/arm_restrictor.h"
#include "sample_acquisition/ArmMovement.h"

using namespace std;

// Class declaration
class ArmDrive
{
public:
    // Constructor. Initializes all variables.
    ArmDrive( const ros::NodeHandle&, string pan_motor_serial, string tilt_motor_serial, string cable_motor_serial, 
                float velocity_mode_time_step, float pan_motor_safe_velocity, float tilt_motor_safe_velocity, float cable_motor_safe_velocity,
                float pan_motor_max_velocity, float tilt_motor_max_velocity, float cable_motor_max_velocity, float all_motors_acceleration );

    // Callbacks
    void panCallback( const phidgets::stepper_params_betterConstPtr & );
    void tiltCallback( const phidgets::stepper_params_betterConstPtr & );
    void cableCallback( const phidgets::stepper_params_betterConstPtr & );
    void movementCallback( const sample_acquisition::ArmMovementConstPtr & );

    // Output to the driver
    bool initializeMotors();
    void setPanMotor( bool, bool, bool );
    void setTiltMotor( bool, bool, bool );
    void setCableMotor( bool, bool, bool );

    // Disengage motor functions
    void disengagePanMotor();
    void disengageTiltMotor();
    void disengageCableMotor();

    // Mode getter
    string getMode();

    // Position getters
    float getPanPos( bool * );
    float getTiltPos( bool * );
    float getCablePos( bool *);

    // Velocity getters
    float getPanVel();
    float getTiltVel();
    float getCableVel();

private:
    // Indicates whether initialization has occured.
    bool pan_init, tilt_init, cable_init;

    // ROS data members.
    ros::NodeHandle nnh;

    ros::Publisher pan_motor_pub;
    ros::Publisher tilt_motor_pub;
    ros::Publisher cable_motor_pub;

    ros::Subscriber pan_motor_sub;
    ros::Subscriber tilt_motor_sub;
    ros::Subscriber cable_motor_sub;

    ros::Subscriber arm_movement_sub;

    // Last position command to be output.
    long long pan_motor_target;
    long long tilt_motor_target;
    long long cable_motor_target;

    // Actual position of the motors.
    long long pan_motor_pos;
    long long tilt_motor_pos;
    long long cable_motor_pos;

    // Last velocity command to be output.
    float pan_motor_vel;
    float tilt_motor_vel;
    float cable_motor_vel;

    // Velocity used in position mode.
    float pan_motor_safe_vel;
    float tilt_motor_safe_vel;
    float cable_motor_safe_vel;

    // Maximum velocities.
    float pan_motor_max_vel;
    float tilt_motor_max_vel;
    float cable_motor_max_vel;

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
