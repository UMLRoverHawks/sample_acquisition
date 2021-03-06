/* sample_acquisition.cpp
 * [ml]
 */

#include "ros/ros.h"

#include <boost/bind.hpp>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>

#include "sample_acquisition/stepper_helper.h"
#include "sample_acquisition/arm_drive.h"
#include "sample_acquisition/ArmMovement.h"
#include "sample_acquisition/ArmStatus.h"


using namespace std;

// ROS parameters. 
string pan_motor_serial;
string tilt_motor_serial;
string cable_motor_serial;

double pan_motor_safe_vel;
double tilt_motor_safe_vel;
double cable_motor_safe_vel;

double pan_motor_max_vel;
double tilt_motor_max_vel;
double cable_motor_max_vel;

double all_motors_accel;
double velocity_mode_time_step;


void output_status( ArmDrive &, ros::Publisher & );

int main( int argc, char** argv)
{
    ros::init(argc,argv,"sample_acquisition");
    ros::NodeHandle nh;

    // Get ROS parameters
    ros::NodeHandle pnh("~");

    // Get serials as integers.
    int pan_ser, tilt_ser, cable_ser;
    pnh.getParam("pan_motor_serial", pan_ser);
    pnh.getParam("tilt_motor_serial", tilt_ser);
    pnh.getParam("cable_motor_serial", cable_ser);

    // Then convert them to strings.
    stringstream pan_ser_s, tilt_ser_s, cable_ser_s;
    pan_ser_s << pan_ser;
    tilt_ser_s << tilt_ser;
    cable_ser_s << cable_ser;

    pan_motor_serial = pan_ser_s.str();
    tilt_motor_serial = tilt_ser_s.str();
    cable_motor_serial = cable_ser_s.str();

    // Get the rest of the parameters.
    pnh.getParam("pan_motor_safe_vel", pan_motor_safe_vel);
    pnh.getParam("tilt_motor_safe_vel", tilt_motor_safe_vel);
    pnh.getParam("cable_motor_safe_vel", cable_motor_safe_vel);

    pnh.getParam("pan_motor_max_vel", pan_motor_max_vel);
    pnh.getParam("tilt_motor_max_vel", tilt_motor_max_vel);
    pnh.getParam("cable_motor_max_vel", cable_motor_max_vel);

    pnh.getParam("all_motors_accel", all_motors_accel);

    pnh.getParam("velocity_mode_time_step",velocity_mode_time_step);

    // Initializes motor communications.
    ArmDrive drive(nh, pan_motor_serial, tilt_motor_serial, cable_motor_serial, 
                    velocity_mode_time_step, pan_motor_safe_vel, tilt_motor_safe_vel, cable_motor_safe_vel,
                    pan_motor_max_vel, tilt_motor_max_vel, cable_motor_max_vel, all_motors_accel);

    // Advertise status updates.
    ros::Publisher arm_pub = nh.advertise<sample_acquisition::ArmStatus>("/arm/status",10);

    // Initialization phase. This is a pretty messed up/hacky way to get this done.
    ros::Rate init_loop(10);
    bool initialized = false;
    while( !initialized ){
        // Sends message to Phidgets drivers to ask for initial motor position.
        // After it hears back from the drivers, initializes the motion restriction
        // class. This requires the motors' initial positions to properly define 
        // ranges of motion. 
        initialized = drive.initializeMotors();

        ros::spinOnce();
        init_loop.sleep();    
    }

    // Begin outputting status messages and accepting commands.
    ros::Rate loop(100);
    int loop_count = 1;
    while(ros::ok()){

        // Output status update at 10Hz
        if ( loop_count >= 10 ) {
            output_status(drive, arm_pub);
            loop_count = 0;
        }

        ros::spinOnce();
        loop.sleep();
        loop_count++;
    }

    return 0;
}


void output_status( ArmDrive &drive, ros::Publisher &arm_pub )
{
    // Put data into a ROS message.
    sample_acquisition::ArmStatus message;
    
    message.pan_position = drive.steppers[PAN_JOINT]->getPos();
    message.tilt_position = drive.steppers[TILT_JOINT]->getPos();
    message.cable_position = drive.steppers[CABLE_JOINT]->getPos();
    message.engaged = drive.isActive();

    // Output the ROS message.
    arm_pub.publish( message );
}
