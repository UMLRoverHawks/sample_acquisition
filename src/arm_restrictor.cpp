/* arm_restrictor.cpp
 * [ml]
 */

#include "sample_acquisition/arm_restrictor.h"

using namespace std;

ArmRestrictor::ArmRestrictor( ros::NodeHandle &nh, long long pan_init, long long tilt_init, long long cable_init )
{
    // Initial positions as angle measures.
    double pan_initial, tilt_initial, cable_initial;
    double gear_ratio;

    nh.getParam("/pan_motor_initial_angle",pan_initial);
    nh.getParam("/tilt_motor_initial_angle",tilt_initial);
    nh.getParam("/cable_motor_initial_angle",cable_initial);
    nh.getParam("/sample_acquisition/gear_ratio",gear_ratio);

    // THIS IS WHERE THE RANGES AND STEP FUNCTIONS ARE SET UP.
    // Hardcoded values... BAD BAD BAD.
    double pan_minimum = -1.570796;     // -pi/2
    double pan_maximum = 1.570796;      // pi/2
    double tilt_minimum = -0.8726646;   // -5pi/18
    double tilt_maximum = 0.174532925;  // pi/18
    double cable_minimum = -2.094395;   // 0
    double cable_maximum = 0.0;         // 2pi/3

    // Used to convert from angles to motor positions.
    double motor_clicks_per_radian = 509.2958178;

    // Convert these max and mins to motor positions.
    pan_min = pan_init - ( (long long)( (pan_maximum-pan_initial) * motor_clicks_per_radian * gear_ratio ) ); // These seem opposite because pan motor spins
    pan_max = pan_init + ( (long long)( (pan_initial-pan_minimum) * motor_clicks_per_radian * gear_ratio ) ); // wrong way to make the angles match motor position
    tilt_max = tilt_init + ( (long long)( (tilt_maximum-tilt_initial) * motor_clicks_per_radian * gear_ratio ) );
    tilt_min = tilt_init - ( (long long)( (tilt_initial-tilt_minimum) * motor_clicks_per_radian * gear_ratio ) );
    cable_max = cable_init + ( (long long)( (cable_maximum-cable_initial) * motor_clicks_per_radian * gear_ratio ) );
    cable_min = cable_init - ( (long long)( (cable_initial-cable_minimum) * motor_clicks_per_radian * gear_ratio ) );   

    // Code for debugging... has come in handy a few times.
    
    ROS_INFO("Calculated Maxes. pan_max: %lld, pan_min: %lld, pan_init: %lld", pan_max, pan_min, pan_init);
    ROS_INFO("Calculation. motor_clicks*gear_ratio: %2f", motor_clicks_per_radian*gear_ratio);
    ROS_INFO("(pan_maximum-pan_initial) * motor_clicks_per_radian * gear_ratio: %2f", (pan_maximum-pan_initial)*motor_clicks_per_radian*gear_ratio);
    

    // Hardcoded values that will be converted to calculated ranges.
    double pan_endpt1 = pan_maximum; // min motor pos = max angle
    double pan_endpt2 = 0.0;
    double pan_endpt3 = pan_minimum; // max motor pos = min angle

    double tilt_endpt1 = tilt_minimum;
    double tilt_endpt2 = 0.0;
    double tilt_endpt3 = tilt_maximum;

    //pan_range1_l = pan_endpt2; // This corresponds to the last sections below.
    //pan_range1_h = pan_endpt3;
    //pan_range2_l = pan_endpt1;
    //pan_range2_h = pan_endpt3;
    //
    //tilt_range1_l = tilt_endpt2;
    //tilt_range1_h = tilt_endpt3;
    //tilt_range2_l = tilt_endpt1;
    //tilt_range2_h = tilt_endpt3;

    // endpoints 1 and last are always min and max. The ones in between need to be calculated as shown below.
    pan_endpoint1 = pan_min; // max angle
    pan_endpoint2 = pan_min + (long long)((pan_maximum - pan_endpt2) * motor_clicks_per_radian * gear_ratio);
    pan_endpoint3 = pan_max; // min angle

    tilt_endpoint1 = tilt_min;
    tilt_endpoint2 = tilt_min + (long long)((tilt_endpt2 - tilt_minimum) * motor_clicks_per_radian * gear_ratio);
    tilt_endpoint3 = tilt_max;

    pan_range1_low = pan_endpoint2;
    pan_range1_high = pan_endpoint3;
    pan_range2_low = pan_endpoint1;
    pan_range2_high = pan_endpoint3;

    tilt_range1_low = tilt_endpoint1;
    tilt_range1_high = tilt_endpoint3;
    tilt_range2_low = tilt_endpoint2;
    tilt_range2_high = tilt_endpoint3;
}


long long ArmRestrictor::getPanTarget( float pan_target, long long tilt_pos )
{
    long long target = (long long)( (pan_target + 1.0)*(pan_max - pan_min)/2.0 + pan_min );

    long long pan_maximum = pan_max;
    long long pan_minimum = pan_min;

    /*
    // TODO: This range checking would also need to be changed in all of these functions..
    if ( tilt_pos <= tilt_endpoint2 ) {
        pan_maximum = pan_range1_high;
        pan_minimum = pan_range1_low;
    }
    else if ( tilt_pos <= tilt_endpoint3 ){
        pan_maximum = pan_range2_high;
        pan_minimum = pan_range2_low;
    }
    */

    // Check target against ranges.
    if (target > pan_maximum)
        target = pan_maximum;

    if (target < pan_minimum)
        target = pan_minimum;

    ROS_INFO("Target: %lld",target);

    return target;
}

long long ArmRestrictor::getTiltTarget( float tilt_target, long long pan_pos )
{
    long long target = (long long)( (tilt_target + 1.0)*(tilt_max - tilt_min)/2.0 + tilt_min );

    long long tilt_maximum = tilt_max;
    long long tilt_minimum = tilt_min;

    /*
    // TODO: This range checking would also need to be changed in all of these functions..
    if ( pan_pos < pan_endpoint2 ) {
        tilt_maximum = tilt_range1_high;
        tilt_minimum = tilt_range1_low;
    }
    else if ( pan_pos < pan_endpoint3 ){
        tilt_maximum = tilt_range2_high;
        tilt_minimum = tilt_range2_low;
    }
    */

    // Check target against ranges.
    if (target > tilt_maximum)
        target = tilt_maximum;

    if (target < tilt_minimum)
        target = tilt_minimum;

    return target;
}

long long ArmRestrictor::getCableTarget( float cable_target )
{
    long long target = (long long)( (cable_target + 1.0)*(cable_max - cable_min)/2.0 + cable_min );

    // Check target against ranges.
    if (target > cable_max)
        target = cable_max;

    if (target < cable_min)
        target = cable_min;

    return target;
}

long long ArmRestrictor::getPanTarget( long long pan_target, long long tilt_pos )
{
    long long pan_maximum = pan_max;
    long long pan_minimum = pan_min;

    /*

    // TODO: This range checking would also need to be changed in all of these functions..
    if ( tilt_pos < tilt_endpoint2 ) {
        pan_maximum = pan_range1_high;
        pan_minimum = pan_range1_low;
    }
    else if ( tilt_pos < tilt_endpoint3 ){
        pan_maximum = pan_range2_high;
        pan_minimum = pan_range2_low;
    }
    */

    // Check target against ranges.
    if (pan_target > pan_maximum)
        pan_target = pan_maximum;

    if (pan_target < pan_minimum)
        pan_target = pan_minimum;

    return pan_target;
}

long long ArmRestrictor::getTiltTarget( long long tilt_target, long long pan_pos )
{
    long long tilt_maximum = tilt_max;
    long long tilt_minimum = tilt_min;

    /*

    // TODO: This range checking would also need to be changed in all of these functions..
    if ( pan_pos < pan_endpoint2 ) {
        tilt_maximum = tilt_range1_high;
        tilt_minimum = tilt_range1_low;
    }
    else if ( pan_pos < pan_endpoint3 ){
        tilt_maximum = tilt_range2_high;
        tilt_minimum = tilt_range2_low;
    }
    */

    // Check target against ranges.
    if (tilt_target > tilt_maximum)
        tilt_target = tilt_maximum;

    if (tilt_target < tilt_minimum)
        tilt_target = tilt_minimum;

    return tilt_target;
}

long long ArmRestrictor::getCableTarget( long long cable_target )
{
    // Check target against ranges.
    if (cable_target > cable_max)
        cable_target = cable_max;

    if (cable_target < cable_min)
        cable_target = cable_min;

    return cable_target;
}

float ArmRestrictor::getPanPosition( long long pan_pos, bool *at_max )
{
    *at_max = (pan_pos == pan_max) || (pan_pos == pan_min);
    return ( (float)(pan_pos - pan_min)/(float)(pan_max - pan_min) ) * 2.0 - 1.0;
}

float ArmRestrictor::getTiltPosition( long long tilt_pos, bool *at_max )
{
    *at_max = (tilt_pos == tilt_max) || (tilt_pos == tilt_min);
    return ( (float)(tilt_pos - tilt_min)/(float)(tilt_max - tilt_min) ) * 2.0 - 1.0;
}

float ArmRestrictor::getCablePosition( long long cable_pos, bool *at_max )
{
    *at_max = (cable_pos == cable_max) || (cable_pos == cable_min);
    return ( (float)(cable_pos - cable_min)/(float)(cable_max - cable_min) ) * 2.0 - 1.0;
}
