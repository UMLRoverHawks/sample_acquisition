/* arm_drive.cpp
 * [ml]
 */

#include "sample_acquisition/arm_drive.h"
#include "sample_acquisition/stepper_helper.h"
#include "std_msgs/Bool.h"
#include <boost/bind.hpp>
#include <cmath>

// Implementation of ArmDrive class member functions
ArmDrive::ArmDrive( const ros::NodeHandle& nh, string pan_motor_serial, string tilt_motor_serial, string cable_motor_serial,
                    float velocity_mode_time_step, float pan_motor_safe_velocity, float tilt_motor_safe_velocity, float cable_motor_safe_velocity,
                    float pan_motor_max_velocity, float tilt_motor_max_velocity, float cable_motor_max_velocity, float all_motors_acceleration) : 
    nnh(nh),
    vel_mode_time_step( vel_mode_time_step ),
    all_motors_accel( all_motors_acceleration )/*,
    pan_at_max( false ),       //unused, undefined, un-uncommented
    tilt_at_max( false ),
    cable_at_max( false )*/
{
  string serials[3] = {pan_motor_serial, tilt_motor_serial, cable_motor_serial};
  float safes[3] = {pan_motor_safe_velocity, tilt_motor_safe_velocity, cable_motor_safe_velocity};
  float maxes[3] = {pan_motor_max_velocity, tilt_motor_max_velocity, cable_motor_max_velocity};
  string descs[3] = {"pan", "tilt", "gripper"};
  arm_activated = false; //Initialize arm in deactivated form
  for(int i=PAN_JOINT; i<=CABLE_JOINT; i++)
    steppers[i] = new StepperHelper(nnh, descs[i], serials[i], safes[i], maxes[i], all_motors_accel);
  arm_movement_sub = nnh.subscribe<sample_acquisition::ArmMovement>("/arm/movement",10,boost::bind(&ArmDrive::movementCallback,this,_1));
  arm_activation_sub = nnh.subscribe("/arm/on", 50, &ArmDrive::activationCallback, this);
}


void ArmDrive::movementCallback( const sample_acquisition::ArmMovementConstPtr& data )
{
    if(arm_activated) //Check to see if arm is activated, otherwise do nothing.
    {
	    // Check to make sure values are correct. If not, either return or fix them.
	    
	    // If both or neither mode is set, do nothing.

	    // Put any velocity values outside of [-1,1] back into this range.
	    float input_pan_vel, input_tilt_vel;
	    input_pan_vel = -data->pan_motor_velocity; // needs to be opposite since motor spins opposite of correct angle
	    input_tilt_vel = data->tilt_motor_velocity;

	    if ( input_pan_vel > 1.0 )
		input_pan_vel = 1.0;
	    else if ( input_pan_vel < -1.0 )
		input_pan_vel = -1.0;
	    
	    if ( input_tilt_vel > 1.0 )
		input_tilt_vel = 1.0;
	    else if ( input_tilt_vel < -1.0 )
		input_tilt_vel = -1.0;

	    float input_vel[3] = {1000*input_pan_vel, 3000*input_tilt_vel, 10000};

	    for(int i=PAN_JOINT;i<=CABLE_JOINT;i++)
	    {
		steppers[i]->setVel(input_vel[i]);
	    }

	     steppers[PAN_JOINT]->setTarget(steppers[PAN_JOINT]->getPos() + 40000 * input_pan_vel);
	     steppers[TILT_JOINT]->setTarget(steppers[TILT_JOINT]->getPos() + 40000 * input_tilt_vel);
	     steppers[PAN_JOINT]->setMotor(true);
	     steppers[TILT_JOINT]->setMotor(true);
	     if (data->gripper_open)
	     {
			steppers[CABLE_JOINT]->setTarget(3200);
			steppers[CABLE_JOINT]->setMotor(true);
	     }
	     else
	     {
			steppers[CABLE_JOINT]->resetPosition();
			steppers[CABLE_JOINT]->disengage();
	     }
     }
}

void ArmDrive::activationCallback(const std_msgs::Bool::ConstPtr& msg)
{
    arm_activated = msg->data; //set arm_activated to received message
    if(!arm_activated) //if arm_activated was just set to false
    {
	    for(int i=PAN_JOINT;i<=CABLE_JOINT;i++) //for every motor;
	    {
		    steppers[i]->disengage(); //Disengage
	    }
    }
}

// Engage == false && reset_position == true => Asks driver to report back current position without moving the motors.
bool ArmDrive::initializeMotors()
{
    return true;
}

bool ArmDrive::isActive() { return arm_activated; }
