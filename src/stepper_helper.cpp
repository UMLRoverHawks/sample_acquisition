/* arm_drive.cpp
 * [ml]
 */

#include "sample_acquisition/stepper_helper.h"
#include <boost/bind.hpp>
#include <cmath>

// Implementation of ArmDrive class member functions
StepperHelper::StepperHelper( const ros::NodeHandle& nh, string description, string serial,
    float motor_safe_velocity, float motor_max_velocity, float all_motors_acceleration) :
    nnh(nh),
    all_motors_accel( all_motors_acceleration ),
    motor_safe_vel(motor_safe_velocity),
    motor_max_vel(motor_max_velocity),
    desc(description),
    motor_vel(0.0),
    motor_pos(0),
    motor_target(0)
{
    motor_pub = nnh.advertise<phidgets::stepper_params>(string("/stepper/").append(serial),10);
    motor_sub = nnh.subscribe<phidgets::stepper_params>(string("/phidgets/stepper/").append(serial),10,boost::bind(&StepperHelper::callback,this,_1));
    phidgets::stepper_params p;
    p.position = 0;
    p.reset_position = true;
    motor_pub.publish(p);
}

bool StepperHelper::isInitialized()
{
    return init;
}


void StepperHelper::callback( const phidgets::stepper_paramsConstPtr& data )
{
    motor_pos = data->position;

    init = true;
    //ROS_INFO("i Motor stuff esta ! : %lld", motor_pos);
}

void StepperHelper::setMotor( bool engaged )
{
    phidgets::stepper_params msg;
    
    msg.engage = engaged;
    msg.velocity = abs(motor_vel);
    msg.acceleration = all_motors_accel;
    msg.position = motor_target;
    msg.reset_position = reset_position;
    reset_position = false;

    ROS_INFO("%s - Pos: %lld, target: %lld, vel: %d, accel: %d", desc.c_str(), motor_pos, motor_target, all_motors_accel, motor_vel);

    motor_pub.publish(msg);
}

void StepperHelper::setTarget(long long int target)
{

  motor_target = target;
}


void StepperHelper::setVel(float vel)
{
  motor_vel = vel;
}

// Lowers the current limit of the pan motor.
void StepperHelper::disengage()
{
    setMotor(false);
}

long long int StepperHelper::getPos()
{
    // Use restrictor class to turn pan_motor_pos into float in range of [-1,1]
    return motor_pos;
}

float StepperHelper::getVel()
{
    return motor_vel; // Convert to [-1,1]??
}

void StepperHelper::usePositionVel()
{
    setVel(motor_safe_vel);
}

void StepperHelper::useVelocityVel(float coeff)
{
    setVel(motor_max_vel * coeff);
}

void StepperHelper::resetPosition()
{
	reset_position = true;
}
