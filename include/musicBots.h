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
            
            resolution=0.025;
            
            file.open(name.c_str());
            index_in_sequence = 0;

            started = false;
            manualControl=true;
            
            min_thrust=0; //minimum thrust setpoint for hover
            thrust=min_thrust;
            
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
            }
            
            else if (key->code==97)
                this->arm=true;
            
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

        void setThrust(const float thrust_)
        {
            this->thrust=thrust_;
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
        float min_thrust;
        std::vector<music_command_type> sequence; //vector that stores x,y of the sequence
        std::ifstream file;
        int index_in_sequence;
        std::clock_t start_time, end_time;
        double elapsed_time;
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
                this->thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //set thrust
                ROS_INFO(" Thrust Value : %f\n",this->thrust);
                time(&start_time); //start timer
            }

            
            while ( started && index_in_sequence<=sequence.size())
            {   
                time(&end_time);
                elapsed_time = difftime(end_time, start_time);
                
                if( elapsed_time >= sequence[index_in_sequence].time_)
                {   
                    this->thrust = calc_thrust_fromMIDI(sequence[index_in_sequence].midi_); //update thrust
                    ROS_INFO(" Thrust Value : %f\n",this->thrust);
                    index_in_sequence++;
                    time(&start_time);
                    time(&end_time);                   
                }

            }

            manualControl=true;
            this->thrust=min_thrust;
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
