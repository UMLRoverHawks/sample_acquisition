/* arm_drive.cpp
 * [ml]
 */

#include "sample_acquisition/arm_drive.h"
#include "sample_acquisition/stepper_helper.h"
#include "sample_acquisition/arm_restrictor.h"
#include <boost/bind.hpp>
#include <cmath>

// Implementation of ArmDrive class member functions
ArmDrive::ArmDrive( const ros::NodeHandle& nh, string pan_motor_serial, string tilt_motor_serial, string cable_motor_serial,
                    float velocity_mode_time_step, float pan_motor_safe_velocity, float tilt_motor_safe_velocity, float cable_motor_safe_velocity,
                    float pan_motor_max_velocity, float tilt_motor_max_velocity, float cable_motor_max_velocity, float all_motors_acceleration) : 
    nnh(nh),
    vel_mode_time_step( vel_mode_time_step ),
    all_motors_accel( all_motors_acceleration ),
    pan_init( false ),
    tilt_init( false ),
    cable_init( false )/*,
    pan_at_max( false ),       //unused, undefined, un-uncommented
    tilt_at_max( false ),
    cable_at_max( false )*/
{
  string serials[3] = {pan_motor_serial, tilt_motor_serial, cable_motor_serial};
  float safes[3] = {pan_motor_safe_velocity, tilt_motor_safe_velocity, cable_motor_safe_velocity};
  float maxes[3] = {pan_motor_max_velocity, tilt_motor_max_velocity, cable_motor_max_velocity};
  string descs[3] = {"pan", "tilt", "gripper"};
  for(int i=PAN_JOINT; i<=CABLE_JOINT; i++)
    steppers[i] = new StepperHelper(nnh, descs[i], serials[i], safes[i], maxes[i], all_motors_accel);
  arm_movement_sub = nnh.subscribe<sample_acquisition::ArmMovement>("/arm/movement",10,boost::bind(&ArmDrive::movementCallback,this,_1));
}


void ArmDrive::movementCallback( const sample_acquisition::ArmMovementConstPtr& data )
{
    // Check to make sure values are correct. If not, either return or fix them.
    
    // If both or neither mode is set, do nothing.
    if ( data->position && data->velocity )
        return;
    if ( !(data->position) && (!data->velocity) )
        return;

    // Put any position values outside of [-1,1] back into this range.
    float input_pan_pos, input_tilt_pos, input_cable_pos;
    input_pan_pos = data->pan_motor_position;
    input_tilt_pos = data->tilt_motor_position;
    input_cable_pos = data->cable_motor_position;

    if ( input_pan_pos > 1.0 )
        input_pan_pos = 1.0;
    else if ( input_pan_pos < -1.0 )
        input_pan_pos = -1.0;
    
    if ( input_tilt_pos > 1.0 )
        input_tilt_pos = 1.0;
    else if ( input_tilt_pos < -1.0 )
        input_tilt_pos = -1.0;

    if ( input_cable_pos > 1.0 )
        input_cable_pos = 1.0;
    else if ( input_cable_pos < -1.0 )
        input_cable_pos = -1.0;

    // Put any velocity values outside of [-1,1] back into this range.
    float input_pan_vel, input_tilt_vel, input_cable_vel;
    input_pan_vel = data->pan_motor_velocity; // needs to be opposite since motor spins opposite of correct angle
    input_tilt_vel = data->tilt_motor_velocity;
    input_cable_vel = data->cable_motor_velocity;

    if ( input_pan_vel > 1.0 )
        input_pan_vel = 1.0;
    else if ( input_pan_vel < -1.0 )
        input_pan_vel = -1.0;
    
    if ( input_tilt_vel > 1.0 )
        input_tilt_vel = 1.0;
    else if ( input_tilt_vel < -1.0 )
        input_tilt_vel = -1.0;

    if ( input_cable_vel > 1.0 )
        input_cable_vel = 1.0;
    else if ( input_cable_vel < -1.0 )
        input_cable_vel = -1.0;

    float input_vel[3] = {input_pan_vel, input_tilt_vel, input_cable_vel};

    // Update variables for position mode.
    if ( data->position) {
        steppers[PAN_JOINT]->setTarget(restrictor->getPanTarget(input_pan_pos, steppers[TILT_JOINT]->getPos()));
        steppers[TILT_JOINT]->setTarget(restrictor->getTiltTarget(input_tilt_pos, steppers[CABLE_JOINT]->getPos()));
        steppers[CABLE_JOINT]->setTarget(restrictor->getCableTarget(input_cable_pos));

        for(int i=PAN_JOINT;i<=CABLE_JOINT;i++)
          steppers[i]->usePositionVel();

        mode = string("position");
    }

    // Update variables for velocity mode.
    if ( data->velocity ) {
        for(int i=PAN_JOINT;i<=CABLE_JOINT;i++)
          steppers[i]->useVelocityVel(input_vel[i]);

        steppers[PAN_JOINT]->setTarget(restrictor->getPanTarget( ( (long long)(steppers[PAN_JOINT]->getVel()*vel_mode_time_step) ) + steppers[PAN_JOINT]->getPos(), steppers[TILT_JOINT]->getPos() ));
        steppers[TILT_JOINT]->setTarget(restrictor->getTiltTarget( ( (long long)(steppers[TILT_JOINT]->getVel()*vel_mode_time_step) ) + steppers[TILT_JOINT]->getVel(), steppers[CABLE_JOINT]->getPos() ));
        steppers[CABLE_JOINT]->setTarget(restrictor->getCableTarget( ( (long long)(steppers[CABLE_JOINT]->getVel()*vel_mode_time_step) ) + steppers[CABLE_JOINT]->getPos() ));

        mode = string("velocity");
    }

    // Call the set functions that move the motors
    for(int i=PAN_JOINT;i<=CABLE_JOINT;i++)
      steppers[i]->setMotor(true);
}

// Engage == false && reset_position == true => Asks driver to report back current position without moving the motors.
bool ArmDrive::initializeMotors()
{
    // Do this continually for 1.0 seconds at 10Hz... just to make sure the motors have initialized and reported their position.
    if ( pan_init && tilt_init && cable_init ) {
        steppers[PAN_JOINT]->setTarget(steppers[PAN_JOINT]->getPos());
        steppers[TILT_JOINT]->setTarget(steppers[TILT_JOINT]->getPos());
        steppers[CABLE_JOINT]->setTarget(steppers[CABLE_JOINT]->getPos());

        ROS_INFO("Initialization Phase. pan_pos: %lld, tilt_pos: %lld, cable_pos: %lld", steppers[PAN_JOINT]->getPos(), steppers[TILT_JOINT]->getPos(), steppers[CABLE_JOINT]->getPos());

        // This must go here, because it requires initial motor positions to have been read.
        restrictor = new ArmRestrictor( nnh, steppers[PAN_JOINT]->getPos(), steppers[TILT_JOINT]->getPos(), steppers[CABLE_JOINT]->getPos() );

        return true;
    }
    
    return false;
}

string ArmDrive::getMode()
{
    return mode;
}

float ArmDrive::getPanPos( bool *at_max )
{
    // Use restrictor class to turn pan_motor_pos into float in range of [-1,1]
    return restrictor->getPanPosition( steppers[PAN_JOINT]->getPos(), at_max );
}

float ArmDrive::getTiltPos( bool *at_max )
{
    // Use restrictor class to turn tilt_motor_pos into float in range of [-1,1]
    return restrictor->getTiltPosition( steppers[TILT_JOINT]->getPos(), at_max );
}

float ArmDrive::getCablePos( bool *at_max )
{
    // Use restrictor class to turn cable_motor_pos into float in range of [-1,1]
    return restrictor->getCablePosition( steppers[CABLE_JOINT]->getPos(), at_max );
}
