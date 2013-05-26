/* arm_restrictor.h
 * [ml]
 */

#ifndef _ARM_RESTRICTOR_H
#define _ARM_RESTRICTOR_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include <map>

using namespace std;

class ArmRestrictor
{
public:
    // Constructor. Takes initial position of each motor and the yaml file path.
    ArmRestrictor( ros::NodeHandle &nh, long long int pan_init, long long int tilt_init, long long int cable_init );

    // Input desired target [-1,1], output closest feasible target as motor position.
    long long int getPanTarget( float, long long int );
    long long int getTiltTarget( float, long long int );
    long long int getCableTarget( float );

    // Input desired target in motor position, output closest feasible target as motor position.
    long long int getPanTarget( long long int, long long int );
    long long int getTiltTarget( long long int, long long int );
    long long int getCableTarget( long long int );

    // Input current position, return position in [-1,1] range.
    float getPanPosition( long long int, bool * );
    float getTiltPosition( long long int, bool * );
    float getCablePosition( long long int, bool * );


private:
    // Absolute max and mins.
    long long int pan_min;   // Found from initial position
    long long int pan_max;   // and input range.

    long long int tilt_min;   // Found from initial position
    long long int tilt_max;   // and input range.

    long long int cable_min;   // Found from initial position
    long long int cable_max;   // and input range.

    // Step functions for pan and tilt.
    // Cable is only restricted by max and min.
    // Hardcoded... this is very bad, yes.
    long long int pan_endpoint1, pan_endpoint2, pan_endpoint3;
    long long int tilt_endpoint1, tilt_endpoint2, tilt_endpoint3;
    long long int pan_range1_low, pan_range1_high, pan_range2_low, pan_range2_high;
    long long int tilt_range1_low, tilt_range1_high, tilt_range2_low, tilt_range2_high;

    // If pan pos is between pan endpoints 1 and 2, tilt range 1 is allowed.
    // If pan pos is between pan endpoints 2 and 3, tilt range 2 is allowed.
    // Etc...
};
#endif
