/* arm_drive.cpp
 * [ml]
 */

#include "sample_acquisition/arm_drive.h"
#include "sample_acquisition/arm_restrictor.h"
#include <boost/bind.hpp>
#include <cmath>

// Implementation of ArmDrive class member functions
ArmDrive::ArmDrive( const ros::NodeHandle& nh, string pan_motor_serial, string tilt_motor_serial, string cable_motor_serial,
                    float velocity_mode_time_step, float pan_motor_safe_velocity, float tilt_motor_safe_velocity, float cable_motor_safe_velocity,
                    float pan_motor_max_velocity, float tilt_motor_max_velocity, float cable_motor_max_velocity, float all_motors_acceleration) : 
    nnh(nh),
    pan_motor_target( 0 ),
    tilt_motor_target( 0 ),
    cable_motor_target( 0 ),
    vel_mode_time_step( vel_mode_time_step ),
    pan_motor_safe_vel( pan_motor_safe_velocity ),
    tilt_motor_safe_vel( tilt_motor_safe_velocity ),
    cable_motor_safe_vel( cable_motor_safe_velocity ),
    pan_motor_max_vel( pan_motor_max_velocity ),
    tilt_motor_max_vel( tilt_motor_max_velocity ),
    cable_motor_max_vel( cable_motor_max_velocity ),
    all_motors_accel( all_motors_acceleration ),
    pan_init( false ),
    tilt_init( false ),
    cable_init( false ),
    pan_at_max( false ),
    tilt_at_max( false ),
    cable_at_max( false )
{
    pan_motor_pub = nnh.advertise<phidgets::stepper_params_better>(string("/stepper/").append(pan_motor_serial),10);
    tilt_motor_pub = nnh.advertise<phidgets::stepper_params_better>(string("/stepper/").append(tilt_motor_serial),10);
    cable_motor_pub = nnh.advertise<phidgets::stepper_params_better>(string("/stepper/").append(cable_motor_serial),10);

    pan_motor_sub = nnh.subscribe<phidgets::stepper_params_better>(string("/phidgets/stepper/").append(pan_motor_serial),10,boost::bind(&ArmDrive::panCallback,this,_1));
    tilt_motor_sub = nnh.subscribe<phidgets::stepper_params_better>(string("/phidgets/stepper/").append(tilt_motor_serial),10,boost::bind(&ArmDrive::tiltCallback,this,_1));
    cable_motor_sub = nnh.subscribe<phidgets::stepper_params_better>(string("/phidgets/stepper/").append(cable_motor_serial),10,boost::bind(&ArmDrive::cableCallback,this,_1));

    arm_movement_sub = nnh.subscribe<sample_acquisition::ArmMovement>("/arm/movement",10,boost::bind(&ArmDrive::movementCallback,this,_1));
}


void ArmDrive::panCallback( const phidgets::stepper_params_betterConstPtr& data )
{
    pan_init = true;
    pan_motor_pos = data->position;

    ROS_INFO("Pan position: %lld", pan_motor_pos);

    /*
    // Disengage the motor if position=target
    if (pan_motor_pos == pan_motor_target) {
        disengagePanMotor();
    }
    */
}

void ArmDrive::tiltCallback( const phidgets::stepper_params_betterConstPtr& data )
{
    tilt_init = true;
    tilt_motor_pos = data->position;

    /*
    // Disengage the motor if position=target
    if (tilt_motor_pos == tilt_motor_target) {
        disengageTiltMotor();
    }
    */
}

void ArmDrive::cableCallback( const phidgets::stepper_params_betterConstPtr& data )
{
    cable_init = true;
    cable_motor_pos = data->position;

    /*
    // Disengage the motor if position=target
    if (cable_motor_pos == cable_motor_target) {
        disengageCableMotor();
    }
    */
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


    // Update variables for position mode.
    if ( data->position) {
        pan_motor_target = restrictor->getPanTarget(input_pan_pos, tilt_motor_pos);
        tilt_motor_target = restrictor->getTiltTarget(input_tilt_pos, pan_motor_pos);
        cable_motor_target = restrictor->getCableTarget(input_cable_pos);

        pan_motor_vel = pan_motor_safe_vel;
        tilt_motor_vel = tilt_motor_safe_vel;
        cable_motor_vel = cable_motor_safe_vel;

        mode = string("position");
    }

    // Update variables for velocity mode.
    if ( data->velocity ) {
        pan_motor_vel =  pan_motor_max_vel * input_pan_vel;
        tilt_motor_vel = tilt_motor_max_vel * input_tilt_vel;
        cable_motor_vel = cable_motor_max_vel * input_cable_vel;

        pan_motor_target = restrictor->getPanTarget( ( (long long)(pan_motor_vel*vel_mode_time_step) ) + pan_motor_pos, tilt_motor_pos );
        tilt_motor_target = restrictor->getTiltTarget( ( (long long)(tilt_motor_vel*vel_mode_time_step) ) + tilt_motor_pos, cable_motor_pos );
        cable_motor_target = restrictor->getCableTarget( ( (long long)(cable_motor_vel*vel_mode_time_step) ) + cable_motor_pos );

        mode = string("velocity");
    }

    // Call the set functions that move the motors
    setPanMotor( true, false, false );
    setTiltMotor( true, false, false );
    setCableMotor( true, false, false );

}

// Engage == false && reset_position == true => Asks driver to report back current position without moving the motors.
bool ArmDrive::initializeMotors()
{
    // Call the set functions to request position without moving the motors.
    setPanMotor( false, true, false );
    setTiltMotor( false, true, false );
    setCableMotor( false, true, false );

    // Do this continually for 1.0 seconds at 10Hz... just to make sure the motors have initialized and reported their position.
    if ( pan_init && tilt_init && cable_init ) {
        pan_motor_target = pan_motor_pos;
        tilt_motor_target = tilt_motor_pos;
        cable_motor_target = cable_motor_target;

        ROS_INFO("Initialization Phase. pan_pos: %lld, tilt_pos: %lld, cable_pos: %lld", pan_motor_pos, tilt_motor_pos, cable_motor_pos);

        // This must go here, because it requires initial motor positions to have been read.
        restrictor = new ArmRestrictor( nnh, pan_motor_pos, tilt_motor_pos, cable_motor_pos );

        return true;
    }
    
    return false;
}

void ArmDrive::setPanMotor( bool spin_motor, bool request_position, bool lower_current )
{
    phidgets::stepper_params_better msg;
    
    msg.spin_motor = spin_motor;
    msg.request_position = request_position;
    msg.lower_current = lower_current;
    msg.velocity = abs(pan_motor_vel);
    msg.acceleration = all_motors_accel;
    msg.position = pan_motor_target;

    ROS_INFO("PAN: Pos: %lld, target: %lld", pan_motor_pos, pan_motor_target);

    pan_motor_pub.publish(msg);
}

void ArmDrive::setTiltMotor( bool spin_motor, bool request_position, bool lower_current )
{
    phidgets::stepper_params_better msg;
    
    msg.spin_motor = spin_motor;
    msg.request_position = request_position;
    msg.lower_current = lower_current;
    msg.velocity = abs(tilt_motor_vel);
    msg.acceleration = all_motors_accel;
    msg.position = tilt_motor_target;

    tilt_motor_pub.publish(msg);
}

void ArmDrive::setCableMotor( bool spin_motor, bool request_position, bool lower_current )
{
    phidgets::stepper_params_better msg;
    
    msg.spin_motor = spin_motor;
    msg.request_position = request_position;
    msg.lower_current = lower_current;
    msg.velocity = abs(cable_motor_vel);
    msg.acceleration = all_motors_accel;
    msg.position = cable_motor_target;

    cable_motor_pub.publish(msg);
}

// Lowers the current limit of the pan motor.
void ArmDrive::disengagePanMotor()
{
    setPanMotor(false, false, true);
}

// Lowers the current limit of the tilt motor.
void ArmDrive::disengageTiltMotor()
{
    setTiltMotor(false, false, true);
}

// Lowers the current limit of the cable motor.
void ArmDrive::disengageCableMotor()
{
    setCableMotor(false, false, true);
}

string ArmDrive::getMode()
{
    return mode;
}

float ArmDrive::getPanPos( bool *at_max )
{
    // Use restrictor class to turn pan_motor_pos into float in range of [-1,1]
    return restrictor->getPanPosition( pan_motor_pos, at_max );
}

float ArmDrive::getTiltPos( bool *at_max )
{
    // Use restrictor class to turn tilt_motor_pos into float in range of [-1,1]
    return restrictor->getTiltPosition( tilt_motor_pos, at_max );
}

float ArmDrive::getCablePos( bool *at_max )
{
    // Use restrictor class to turn cable_motor_pos into float in range of [-1,1]
    return restrictor->getCablePosition( cable_motor_pos, at_max );
}

float ArmDrive::getPanVel()
{
    return pan_motor_vel; // Convert to [-1,1]??
}

float ArmDrive::getTiltVel()
{
    return tilt_motor_vel;
}

float ArmDrive::getCableVel()
{
    return cable_motor_vel;
}

