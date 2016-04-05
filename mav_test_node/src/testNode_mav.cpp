//Author: Bishwamoy Sinha Roy

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/UInt16.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //ros::Publisher accel_pub = nh.advertise<geometry_msgs::Vector3Stamped>
      //      ("mavros/setpoint_accel/accel", 10);
    //ros::Publisher thr_pub = nh.advertise<mavros_msgs::OverrideRCIn>
    //        ("mavros/rc/override", 10);
    ros::Publisher thr_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /*geometry_msgs::Vector3Stamped accel;
    accel.vector.x = 4;
    accel.vector.y = 20;
    accel.vector.z = 10;
    int throttles_int[] = {1600, 1600, 1600, 1600, 0, 0, 0, 0};*/
    float thrust_ = 0.1;
    mavros_msgs::AttitudeTarget val;
    val.thrust = thrust_;
    //std_msgs::UInt16 throttles[8];
    /*mavros_msgs::OverrideRCIn val;

    for(int i =0; i < 8; i++)
    {
        val.channels[i] = throttles_int[i];
    }*/

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        thr_pub.publish(val);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    	//make sure the pixhawk is in offboard mode and is armed
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        thr_pub.publish(val);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}