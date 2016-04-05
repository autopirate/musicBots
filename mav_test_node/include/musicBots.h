#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <keyboard/Key.h>

#include <fstream>
#include <string>

typedef struct
{
    double midi_;
    double time_;
} music_command_type;

class attitudeControl
{
    public:

        attitudeControl(std::string name, ros::NodeHandle nh)
        {
            this->init_variables(name);
            this->init_subscribers(nh);
            this->init_publishers(nh);
            this->init_serviceClients(nh);
        }

        ~attitudeControl()
        {
            file.close();
        }

        double getThrust()
        {
            return this->thrust;
        }

        bool getArm()
        {
            return this->arm;
        }

        ros::Time getLastRequestTime()
        {
            return this->last_request;
        }

        mavros_msgs::State get_CurState()
        {
            return this->current_state;
        }

        void setThrust(const float thrust_)
        {
            this->thrust = thrust_;
        }

        void setLastRequestTime()
        {
            this->last_request = ros::Time::now();
        }

        void publish_Thrust()
        {
            mavros_msgs::AttitudeTarget attitude;
            attitude.thrust= this->getThrust();
            local_att_pub.publish(attitude);
        }

        void publish_Thrust(float val)
        {
            mavros_msgs::AttitudeTarget attitude;
            attitude.thrust= val;
            local_att_pub.publish(attitude);
        }

        void run()
        {

            setOffBoard();
            
            this->setLastRequestTime();

            this->publish_Thrust();
        }
        
        void setOffBoard()
        {

            if( this->get_CurState().mode != "OFFBOARD" &&
                (ros::Time::now() - this->getLastRequestTime() > ros::Duration(5.0)))
            {
                if( this->set_mode_client.call(this->offb_set_mode) &&
                    this->offb_set_mode.response.success)
                {
                    ROS_INFO("Offboard enabled");
                }
            } 
        }

        void arm_cb()
        {
          
                    if( this->arming_client.call(this->arm_cmd) && this->arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    else
                    {
                        ROS_INFO("Arming Failed");
                    }                    
          
    
        }

        void disarm_cb()
        {
                if( this->arming_client.call(this->disarm_cmd) && this->disarm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                }
                else
                {
                    ROS_INFO("disarming Failed");
                }  
        }
    private:
        //Variables
        double thrust;
        float resolution;
        float min_thrust;
        std::vector<music_command_type> sequence; //vector that stores x,y of the sequence
        std::ifstream file;
        int index_in_sequence;
        std::clock_t start_time, end_time;
        double elapsed_time;
        bool started;
        bool manualControl;
        bool arm;
        ros::Time last_request;
        //calback updated variables
        mavros_msgs::State current_state;

        void init_variables(std::string name)
        {
            resolution=0.025;
            
            file.open(name.c_str());
            index_in_sequence = 0;

            started = false;
            manualControl=true;
            
            min_thrust = 0; //minimum thrust setpoint for hover
            thrust = min_thrust;
            this->setLastRequestTime();            
            
            if( file.is_open() )
            {
                getSequenceFromFile();
            }

            else
            {
                ROS_INFO("ERROR opening sequence file");
            }
        }

        //Publishers
        ros::Publisher local_att_pub;
        void init_publishers(ros::NodeHandle nh)
        {
            this->local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                ("mavros/setpoint_raw/attitude", 10);
        }

        //Service Clients
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::CommandBool arm_cmd, disarm_cmd;
        mavros_msgs::SetMode offb_set_mode;
        void init_serviceClients(ros::NodeHandle nh)
        {
            this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
            this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
            
            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            disarm_cmd.request.value = false;
        }

        //Subsribers
        ros::Subscriber state_sub;
        ros::Subscriber keyboard;
        void init_subscribers(ros::NodeHandle nh)
        {
            this->state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, &attitudeControl::state_cb, this);
            this->keyboard = nh.subscribe<keyboard::Key>
                ("/keyboard/keydown", 10, &attitudeControl::updateThrust_cb, this);
        }

        //Callbacks
        void state_cb(const mavros_msgs::State::ConstPtr& msg){
           // std::cout << "State Read" << std::endl;
            
            this->current_state = *msg;
            std::cout<<"Thrust: "<<this->thrust<<std::endl;
        }

        void updateThrust_cb(const keyboard::Key::ConstPtr& key)
        {   
            if (key->code==112 && this->arm)
                {
                    this->manualControl=false;
                    play_music();
                }
            
            else if (key->code==109)
                this->manualControl=true;

            else if (key->code==27)
            {    
                this->thrust=0;
                this->arm=false;
                disarm_cb();
            }
            
            else if (key->code==97)
            {   this->arm=true;
                arm_cb();
            }

            if (manualControl && this->arm)
            {
                if (key->code==273)
                    this->thrust+=resolution;
                else if (key->code==274)
                    this->thrust-=resolution;
            }


            ROS_INFO(" Thrust Value : %f\n",this->thrust);
            //ROS_INFO("Resolution : %f \n", this->resolution);
        
        }


        //Functions
        /*
         * Private function to read music sequence from file (called in init_variables(...))
         */
        void getSequenceFromFile()
        {
            double midi_read;
            double time_read;
            music_command_type input_cmd;
                

            while(file >> midi_read >> time_read)   //simply reading the file for now
            {                                       //need to process sequence
                // double midi_d = std::stod(midi_read);
                // double time_d = std::stod(time_read);
                std::cout << midi_read << " " << time_read << std::endl;
                input_cmd.midi_ = midi_read;
                input_cmd.time_ = time_read;
                sequence.push_back(input_cmd);
            }

        }

        /*
         * Private function to update the thrust depending on time and music file read
         */
        void play_music()
        {
            if(!started)
            {
                index_in_sequence = 0;
                started = true;
                this->thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //set thrust
                this->publish_Thrust();
                ROS_INFO(" Thrust Value : %f\n",this->thrust);
                time(&start_time); //start timer
            }

            
            while ( started && index_in_sequence<sequence.size())
            {   
                time(&end_time);
                elapsed_time = difftime(end_time, start_time);
                this->publish_Thrust();
                if( elapsed_time >= sequence[index_in_sequence].time_ && index_in_sequence!=sequence.size())
                {   
                    index_in_sequence++;
                    this->thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //update thrust
                    
                    ROS_INFO(" Thrust Value : %f\n",this->thrust); 
                    time(&start_time);
                    time(&end_time);  
                                   
                }

            }

            manualControl=true;
            this->thrust=min_thrust;
            started=false;
            setOffBoard();
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
