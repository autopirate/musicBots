
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <keyboard/Key.h>
#include <fstream>
#include <string>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

typedef struct 
        {
            double midi_;
            double time_;
        } music_command_type;


class attitudeControl
{
    public:

        attitudeControl(std::string name)
        {
            thrust=0;
            resolution=0.025;
            file.open(name.c_str());
            index_in_sequence = 0;
            started = false;
            manualControl=true;

            if( file.is_open() )
            {
                getSequenceFromFile();
            }

            else
            {
                ROS_INFO("ERROR opening sequence file");
            }

        }

        ~attitudeControl()
        {
            file.close();
        }
    
        void updateThrust(const keyboard::Key::ConstPtr& key)
        {   
            if (key->code==112)
                this->manualControl=false;
            else if (key->code==109)
                this->manualControl=true;
            else if (key->code==27)
                this->arm=false;
            else if (key->code==97)
                this->arm=true;
            
            if (manualControl && this->arm)
            {
                if (key->code==273)
                    this->thrust+=resolution;
                else if (key->code==274)
                    this->thrust-=resolution;
            }

            else if (!manualControl && this->arm)
            {
                play_music();
            }


            ROS_INFO(" Thrust Value : %f\n",this->thrust);
        //ROS_INFO("Resolution : %f \n", this->resolution);
        
        }

        void getSequenceFromFile()
        {
            double midi_read;
            double time_read;
            music_command_type input_cmd;
                

            while(file >> midi_read >> time_read) //simply reading the file for now
            {                           //need to process sequence
                // double midi_d = std::stod(midi_read);
                // double time_d = std::stod(time_read);
                std::cout << midi_read << " " << time_read << std::endl;
                input_cmd.midi_ = midi_read;
                input_cmd.time_ = time_read;
                sequence.push_back(input_cmd);
            }

        }

        double getthrust()
        {
            return thrust;
        }
        bool arm;
    private:
        double thrust;
        float resolution;
        
        std::vector<music_command_type> sequence; //vector that stores x,y of the sequence
        std::ifstream file;
        int index_in_sequence;
        std::clock_t start_time, end_time;
        bool started;
        bool manualControl;

        /*
         * Private function to update the thrust depending on time and music file read
         */
        void play_music()
        {
            if(!started)
            {
                index_in_sequence = 0;
                started = true;
                thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //set thrust
                time(&start_time); //start timer
                return;
            }

            time(&end_time);
            double elapsed_time = difftime(end_time, start_time);
            if( elapsed_time >= sequence[index_in_sequence].time_ 
                        && index_in_sequence < sequence.size() )
            {
                index_in_sequence++;
                thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //update thrust
                time(&start_time); //start timer
                return;
            }

            else if (index_in_sequence==sequence.size())
            {
                started=false;
                thrust= 0;
                manualControl=true;
            }
        }

        double calc_thrust_fromMIDI(double MIDI)
        {
            //TODO
            double thrust_ = 0.0;
            double m = 0.05743;
            double b = -2.752;
            thrust_ = m*MIDI + b;
            return thrust_;
        }
};


int main(int argc, char **argv)
{   
    attitudeControl attSet("test.txt");

    ros::init(argc, argv, "mav_test");
    ros::NodeHandle nh;

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

        else if (current_state.armed && !attSet.arm && (ros::Time::now() - last_request > ros::Duration(5.0)))
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
                last_request = ros::Time::now();
            }
        }
        
        attitude.thrust= attSet.getthrust();
        local_att_pub.publish(attitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}