#include <ros/ros.h>
#include <musicBots.h>

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "mav_test");
    ros::NodeHandle nh;

    attitudeControl attSet("/home/shivam/catkin_ws/src/mav_test/src/test.txt", nh);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && attSet.get_CurState().connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        attSet.publish_Thrust();
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        attSet.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}