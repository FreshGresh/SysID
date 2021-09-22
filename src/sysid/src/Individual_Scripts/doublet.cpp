#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h> // linear_acceleration, angular_velocity, orientation (quaternion)
#include <mavros_msgs/ActuatorControl.h> // de, da, dr, dt (not directly, pwm)
#include <geometry_msgs/PoseStamped.h> // position and orientation
#include <geometry_msgs/TwistStamped.h> // linear and angular body velocity
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Altitude.h>
#include <iostream>
#include <fstream>
#include "rxtx.hpp"

//What ROS does is read the data from Pixhawk and dumps it in ROS topics.
//The lines of code below takes the data from the ROS topic called and stores to be used in the code.
mavros_msgs::State current_state;//State of pixhawk e.g. armed/disarmed
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::RCIn rc_input;//RC input values this step initializes the variable within the right class
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg){//This is a callback function to access the data in the topic
    rc_input = *msg;//This stores the data into the variable (This is the same for all the seubsequent lines till line 84)
}

mavros_msgs::ManualControl manual_input;//The default commanded servo values
void manual_cb(const mavros_msgs::ManualControl::ConstPtr& msg){
    manual_input = *msg;
}

sensor_msgs::Imu imu_data;//IMU data
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data = *msg;
}

std_msgs::Float64 float64_data;//Not exactly sure what this one does
void float64_cb(const std_msgs::Float64::ConstPtr& msg){
    float64_data = *msg;
}

sensor_msgs::NavSatFix NavSat_data;//Data on GPS satellites..
void NavSat_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    NavSat_data = *msg;
}

mavros_msgs::ActuatorControl actuator_data;//Actuator values
void actuator_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg){
    actuator_data = *msg;
}

geometry_msgs::PoseStamped pose_data;//Attitude of aircraft
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_data = *msg;
}

geometry_msgs::PoseWithCovarianceStamped pose_wcv_data;//Attitude of aircraft+covariance
void pose_wcv_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    pose_wcv_data = *msg;
}

nav_msgs::Odometry odom_data;//Odometry data
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom_data = *msg;
}

geometry_msgs::TwistStamped twist_data;//Not sure what this one does
void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    twist_data = *msg;
}


mavros_msgs::Altitude alt_data;//Altitude data
void alt_data_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    alt_data = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_node");//Initializes ROS
    ros::NodeHandle nh;//Creates Node handle
    ROS_INFO_STREAM("Init!");//Prints Init! (for debugging purposes)

    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>//Lines 91 through 123 Establish connection with the desired ROS topics and defines the frequency at which we take data from it
           ("mavros/rc/in", 1, rcin_cb);
    ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>
           ("mavros/manual_control/control", 1, manual_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>
	("mavros/actuator_control", 1);
    ros::Publisher rc_override = nh.advertise<mavros_msgs::OverrideRCIn>
	("mavros/rc/override", 1);
    ros::Subscriber alt_data_sub = nh.subscribe<mavros_msgs::Altitude>
           ("mavros/altitude", 1, alt_data_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>
           ("mavros/imu/data", 1, imu_cb);
    ros::Subscriber actuator_data_sub = nh.subscribe<mavros_msgs::ActuatorControl>
           ("mavros/target_actuator_control", 1, actuator_cb);
    ros::Subscriber pose_data_sub = nh.subscribe<geometry_msgs::PoseStamped>
           ("mavros/local_position/pose", 1, pose_cb);
    //ros::Subscriber pose_wcv_data_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
           //("mavros/global_position/local", 1, pose_wcv_cb);
    /*ros::Subscriber odom_data_sub = nh.subscribe<nav_msgs::Odometry>
           ("mavros/local_position/odom", 25, odom_cb);*/
    ros::Subscriber float64_data_sub = nh.subscribe<std_msgs::Float64>
           ("mavros/global_position/rel_alt", 1, float64_cb);

    //ros::Subscriber NavSat_data_sub = nh.subscribe<sensor_msgs::NavSatFix>
           //("mavros/global_position/global", 1, NavSat_cb);
    //ros::Subscriber twist_data_sub = nh.subscribe<geometry_msgs::TwistStamped>
           //("mavros/global_position/gp_vel", 1, twist_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);//Defines the rate at which your code will run. Note that the code will not run faster than this rate but may run slower if its computationally taxing

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO_STREAM("Waiting!");
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::ActuatorControl act_con;//Creates the variable used to actuate servos
    double t0 = ros::Time::now().toSec();//Records initial time
    double t;
        if (rc_input.channels[5]>1500){
            t = 0.4;}//elevator doublet half period
        else {
            t = 0.6;}//rudder doublet half period
    double t1 = t;
    double t2 = t;
    double t3 = 3.0;
    double q0 = imu_data.orientation.w;
    double q1 = imu_data.orientation.x;
    double q2 = imu_data.orientation.y;
    double q3 = imu_data.orientation.z;
    double xi = 1/(1-pow(2*(q1*q3-q0*q2),2));
    double sin_phi = 0.0;
    double sin_theta = 0.0;
    double p = imu_data.angular_velocity.x;
    double q = imu_data.angular_velocity.y;
    double r = imu_data.angular_velocity.z;
    int phase = 0;
    double de_com = 0.0;
    double dr_com = 0.0;
    double da_com = 0.0;
    double com = 0.0;
    double com_high = 1;
    double com_low = -1;
    double temp = 0;
    double amp = 1;
    double k_ph = 1.1;
    double k_th = 1.0;
    double k_p = 0.08;
    double k_q = 0.08;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        ros::spinOnce();
        rate.sleep();
    }
    


    while(ros::ok()){
        q0 = imu_data.orientation.w;
        q1 = imu_data.orientation.x;
        q2 = imu_data.orientation.y;
        q3 = imu_data.orientation.z;
        xi = 1/(1-pow(2*(q1*q3-q0*q2),2));
        sin_phi = 2*(q0*q1+q2*q3)*xi;
        sin_theta = 2*(q1*q3-q0*q2);
        p = imu_data.angular_velocity.x;
        q = imu_data.angular_velocity.y;
        r = imu_data.angular_velocity.z;

        if (rc_input.channels[4]> 1500){//Resets maneuver if pilot desires
            phase = 0;}

        if (rc_input.channels[5]>1500){//Longitudinal sysid maneuver

            switch (phase) {//This phase is the idle phase to capture the effects of the doublet. Flipping Channel 5 to low resets the process and sets the phase to stand by phase
                case 0:
                    de_com= 0.0;
                    if (rc_input.channels[4]>1500){
                        phase = 1;}
                    break;
                case 1://This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet
                    de_com= 0.0;
                    if (rc_input.channels[4]<1500){
                        t0 = ros::Time::now().toSec();
                        temp = com_high;
                        com_high = com_low;
                        com_low = temp;
                        de_com = com_high;
                        phase = 2;}
                    break;
                case 2://This is the high phase of the doublet where the elevator is set to the high setting for the desired duration
                    de_com = com_high;
                    if (ros::Time::now().toSec()-t0 >= t1){
                        de_com = com_low;
                        phase = 3;}
                    break;
                case 3://This is the low phase of the doublet where the elevator is set to its low setting for the desired duration
                    de_com = com_low;
                    if (ros::Time::now().toSec()-t0 >= t1 + t2){
                        de_com = 0.0;
                        phase = 0;}
                    break;}
                dr_com = 0.0;
                da_com = 0.0;
                amp = 0.8;//elevator doublet amplitude
                act_con.controls[0] = std::max( std::min( 0.0 + manual_input.y, 1.0), -1.0 );//Aileron servo (-1.0,1.0)
                act_con.controls[1] = std::max( std::min( amp*de_com - manual_input.x, 1.0), -1.0 );//Elevator servo (-1.0,1.0)
                act_con.controls[2] = std::max( std::min( 0.0 + manual_input.r, 1.0), -1.0 );//Rudder servo (-1.0,1.0)
            }
        else{//LatDir sysid maneuver
            switch (phase) {//This phase is the idle phase to capture the effects of the doublet. Flipping Channel 5 to low resets the process and sets the phase to stand by phase
                case 0:
                    dr_com= 0.0;
                    da_com= 0.0;
                    if (rc_input.channels[4]>1500){
                        phase = 1;}
                    break;
                case 1://This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet
                    dr_com= 0.0;
                    da_com = 0.0;
                    if (rc_input.channels[4]<1500){
                        t0 = ros::Time::now().toSec();
                        temp = com_high;
                        com_high = com_low;
                        com_low = temp;
                        dr_com = com_high;
                        phase = 2;}
                    break;
                case 2://This is the high phase of the doublet where the elevator is set to the high setting for the desired duration
                    dr_com = com_high;
                    da_com = 0.0;
                    if (ros::Time::now().toSec()-t0 >= t1){
                        dr_com = com_low;
                        phase = 3;}
                    break;
                case 3://This is the low phase of the doublet where the elevator is set to its low setting for the desired duration
                    dr_com = com_low;
                    da_com = 0.0;
                    if (ros::Time::now().toSec()-t0 >= t1 + t2){
                        dr_com = 0.0;
                        phase = 4;}//if aileron 1-2-1 is not needed change the value from 4 to 0
                    break;
                case 4://Interim phase to capture effects of rudder double
                    dr_com= 0.0;
                    da_com= 0.0;
                    if (ros::Time::now().toSec()-t0 >= t1 + t2 + t3){
                        da_com = com_low;
                        phase = 5;}
                    break;
                case 5://first part of 1-2-1 aileron maneuver
                    dr_com= 0.0;
                    da_com= com_low;
                    if (sin_phi * com_low >= 0.707){
                        da_com = com_high;
                        phase = 6;}
                    break;
                case 6://second part of 1-2-1 aileron maneuver
                    dr_com= 0.0;
                    da_com= com_high;
                    if (sin_phi * com_high >= 0.707){
                        da_com = com_low;
                        phase = 7;}
                    break;
                case 7://third and last part of 1-2-1 aileron maneuver
                    dr_com= 0.0;
                    da_com= com_low;
                    if (sin_phi * com_low >= -0.0871557427){
                        da_com = 0.0;
                        phase = 0;}
                    break;}
                de_com = 0.0;
                amp = 0.6;//rudder doublet amplitude
                act_con.controls[0] = std::max( std::min( amp*da_com + manual_input.y, 1.0), -1.0 );//Aileron servo (-1.0,1.0)
                act_con.controls[1] = std::max( std::min( 0.0 - manual_input.x , 1.0), -1.0 );//Elevator servo (-1.0,1.0)
                act_con.controls[2] = std::max( std::min( amp*dr_com + manual_input.r, 1.0), -1.0 );//Rudder servo (-1.0,1.0)
            }

        act_con.controls[3] = std::max( std::min( 0.0 + manual_input.z, 1.0), -1.0 );//Propeller throttle (0.0,1.0)

        actuator_pub.publish(act_con);//Writes actuator commands based on variable values

        ros::spinOnce();//Calls all callbacks waiting to becalled at that point in time
        rate.sleep();//Sleeps for a period of time based on the frequency or rate defined for ROS to run
    }

    return 0;
}

