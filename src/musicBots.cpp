
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <keyboard/Key.h>
#include <fstream>
#include <string>

#include <musicBots.h>






int main(int argc, char **argv)
{   
    

    ros::init(argc, argv, "mav_test");
    ros::NodeHandle nh;

    attitudeControl attSet("/home/shivam/catkin_ws/src/mav_test/src/test.txt");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber keyboard =nh.subscribe<keyboard::Key>
            ("/keyboard/keydown",10,&attitudeControl::updateThrust,&attSet);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    

    mavros_msgs::AttitudeTarget attitude;
    attitude.thrust= attSet.getthrust();


    //send a few setpoints before starting
 
    for(int i = 100; ros::ok() && i > 0; --i){
        local_att_pub.publish(attitude);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd, disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        else if (!attSet.arm && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
        }

        else {
            if( !current_state.armed && attSet.arm && 
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                
            }
        }
        

        last_request = ros::Time::now();
        attitude.thrust= attSet.getthrust();
        local_att_pub.publish(attitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}