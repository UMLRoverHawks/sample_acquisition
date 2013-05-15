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
    ArmRestrictor( ros::NodeHandle &nh, long long pan_init, long long tilt_init, long long cable_init );

    // Input desired target [-1,1], output closest feasible target as motor position.
    long long getPanTarget( float, long long );
    long long getTiltTarget( float, long long );
    long long getCableTarget( float );

    // Input desired target in motor position, output closest feasible target as motor position.
    long long getPanTarget( long long, long long );
    long long getTiltTarget( long long, long long );
    long long getCableTarget( long long );

    // Input current position, return position in [-1,1] range.
    float getPanPosition( long long, bool * );
    float getTiltPosition( long long, bool * );
    float getCablePosition( long long, bool * );


private:
    // Absolute max and mins.
    long long pan_min;   // Found from initial position
    long long pan_max;   // and input range.

    long long tilt_min;   // Found from initial position
    long long tilt_max;   // and input range.

    long long cable_min;   // Found from initial position
    long long cable_max;   // and input range.

    // Step functions for pan and tilt.
    // Cable is only restricted by max and min.
    // Hardcoded... this is very bad, yes.
    long long pan_endpoint1, pan_endpoint2, pan_endpoint3;
    long long tilt_endpoint1, tilt_endpoint2, tilt_endpoint3;
    long long pan_range1_low, pan_range1_high, pan_range2_low, pan_range2_high;
    long long tilt_range1_low, tilt_range1_high, tilt_range2_low, tilt_range2_high;

    // If pan pos is between pan endpoints 1 and 2, tilt range 1 is allowed.
    // If pan pos is between pan endpoints 2 and 3, tilt range 2 is allowed.
    // Etc...
};
#endif
