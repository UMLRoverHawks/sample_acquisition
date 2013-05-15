/* keyboard_drive.cpp
 * [ml]
 */

#include "ros/ros.h"

#include <boost/bind.hpp>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>

#include "sample_acquisition/ArmMovement.h"


using namespace std;

// ROS parameter. 
double position_step;

// Positions.
float pan_pos = 0.0;
float tilt_pos = 0.0;
float cable_pos = 0.0; 

// Helper functions for keyboard input.
bool run( ros::Publisher & );
void nonblock(bool state);
int kbhit();
void shutdown(int signo);


void statusCallback( const sample_acquisition::ArmMovementConstPtr &data )
{
    pan_pos = data->pan_motor_position;
    tilt_pos = data->tilt_motor_position;
    cable_pos = data->cable_motor_position;
}

int main( int argc, char** argv)
{

    signal(SIGINT,shutdown);
    signal(SIGTERM,shutdown);

    ros::init(argc,argv,"keyboard_drive");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("position_step", position_step);

    ros::Subscriber status_sub = nh.subscribe<sample_acquisition::ArmMovement>("/arm/status",10,statusCallback);
    ros::Publisher movement_pub = nh.advertise<sample_acquisition::ArmMovement>("/arm/movement",10);

    // Begin talking to the motors and taking keyboard commands
    ros::Rate loop_rate(100); // Loop at 100Hz
    nonblock(true);
    while(ros::ok()){
        run(movement_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
    nonblock(false);

    return 0;
}


bool run( ros::Publisher &pub )
{
    char c = '\0';
    if(kbhit()){
        c = fgetc(stdin);
        switch(c){
        case 'a':
            pan_pos += position_step;
            break;
        case 'd':
            pan_pos -= position_step;
            break;
        case 'w':
            tilt_pos += position_step;
            break;
        case 's':
            tilt_pos -= position_step;
            break;
        case 'f':
            cable_pos += position_step;
            break;
        case 'g':
            cable_pos -= position_step;
            break;
        }

        sample_acquisition::ArmMovement msg;
        msg.position = true;
        msg.velocity = false;
        msg.pan_motor_position = pan_pos;
        msg.tilt_motor_position = tilt_pos;
        msg.cable_motor_position = cable_pos;

        pub.publish( msg );
    }

    usleep(10);
    return true;
}

void nonblock(bool state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if(state){
        ttystate.c_lflag &= ~ICANON & ~ECHO;
        ttystate.c_cc[VMIN] = 1;
    }
    else{
        ttystate.c_lflag |= ICANON | ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO,&fds);
    select(STDIN_FILENO+1,&fds,NULL,NULL,&tv);
    return FD_ISSET(STDIN_FILENO,&fds);
}

void shutdown(int signo)
{
    nonblock(false);
    exit(1);
}
