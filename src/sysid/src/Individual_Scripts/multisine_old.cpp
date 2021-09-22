#include<ros/ros.h> // Header file which allows the use of ROS related commands and functions
#include<mavros_msgs/State.h> // Header file which enables processing data related to the Pixhawk status (e.g. Armed, connected, etc...)
#include<mavros_msgs/ActuatorControl.h> // Header file which enables processing data related to actuator control
#include<mavros_msgs/ManualControl.h> // Header file which enables processing data related to actuator control based on RC input
#include<mavros_msgs/RCIn.h> // Header file which enables processing data related to RC signal values
#include<vector> // Header file which allows the use of vectors
#include<fstream> // Header file which enables reading and writing files via C++
#include<string> // Header file  which allows the use of strings in C++
using namespace std; // Saves us the trouble of having to write "std::" in front of every other function we use

bool read_flag = false;

mavros_msgs::State current_state;// Initializes global variable to store the state of the Pixhawk (e.g. if connected or not)
void state_cb(const mavros_msgs::State::ConstPtr& msg){// call back function to read & store the state of the Pixhawk
	current_state = *msg;}

mavros_msgs::RCIn rc_input;// Initializes global variable to store the read RC inputs
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg){// call back function to read & store the RC input
	rc_input = *msg;
	read_flag = true;}

mavros_msgs::ManualControl manual_input;// Initializes global variable to store the read normalized manual control input values
void manual_cb(const mavros_msgs::ManualControl::ConstPtr& msg){// Call back function to read and store normalized manual control inputs
	manual_input = *msg;}
/*
mavros_msgs::Actuator actuator_data;// Initializes global variable to store the read actuator control
void actuator_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg){
	actuator_data = *msg;}*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "sysid"); // This function need to see argc and argv so that it can perform any ROS arguments and name remapping that were provided at the command line. Third argument is the name of the node
	ros::NodeHandle nh; // NodeHandle is the main access point to communications with the ROS system.
	
	// In the next 4 lines, the node would informs ROS that it wants to either subscribe (read) or publish (write) to a specific topic. The main ROS node would in turn negotiates a peer-to-peer connection with the node. Once the code is terminated, all connections are automatically terminated. The first argument is the topic in question, the second is the message queue (in case messages are published more quickly than they can be sent; the number specifies the number of messages to buffer before throwing the others away). When present, the third argument specifies which call back function to use to store the message in a corresponding global variable.
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);// Subscribes to the relevant ROS topic to read the current state of Pixhawk
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 100, rcin_cb);// Subscribes to the relevant ROS topic to read RC channel values
	ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>("mavros/manual_control/control", 100, manual_cb);// Subscribes to relevant ROS topic to read manual control (trims + normalized pilot inputs
	ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 100);// Publishes in the relevant ROS topic to actuate servos/ props to desired normalized values

	ros::Rate rate(200.0);// Sets the rate at which ROS refreshes the while loop (basically helps determine how much it has to wait for the next cycle to obtain the necessary rate)

	while (ros::ok() && !current_state.connected){// Waits until the program determines that the connection to the Pixhawk is established
		ros::spinOnce();// Ensures that all the processes ROS has in queue are finished
		rate.sleep();}// Waits the appropriate time to ensure the rate prescribed is achieved

	//Since the multisine sequence has a 1001 step sequence, I decided to store the values in a .csv file (generated via Matlab) and load it into appropriately sized arrays
	ifstream doc;// Create variable to load .csv file
	doc.open("/home/nsl/sysid3_ws/src/sysid/src/ms_input_100.csv");// loads .csv file
	vector<double> t, dA, dE, dR;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
	int counter = 0;// variable to keep track of the line at which the .csv file the code is and to assign its values to  the corresponding position along the arrays
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dR.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		ROS_INFO_STREAM("t = " + to_string(t.back()) + ", dA = " + to_string(dA.back()) + ", dE = " + to_string(dE.back()) + ", dR = " + to_string(dR.back())); // Prints the last datapoints successfully extracted (for debugging purposes only)
		counter++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
	doc.close(); // Clearing variable since we no longer need it
	doc.clear(); // Clearing variable since we no longer need it
	line.clear(); // Clearing variable since we no longer need it
	word.clear(); // Clearing variable since we no longer need it
	temp.clear(); // Clearing variable since we no longer need it

	mavros_msgs::ActuatorControl act_con;// Initializes variable used to actuate servos
	double t0 = 0;// Records initial time
	double dt = 0.01; // Length of time step of multisine maneuver
	int PWM_max = 2073;// Max PWM value for channel 6
	int PWM_min = 967;// Min PWM value for channel 6
	double amp_gain = 1.0/(1.0 * (PWM_max - PWM_min));// Determines the gain for the amplifier of the multisine signal (based of the value of channel 6). Range = {0,1}
	double amp = 0.0; // Amplifier based on the rc input from channel 6
	int phase = 0;// The phase of the maneuve. 0 when off and 1 when it is being executed
	double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0;// Initializes the commanded actuator inputs
    int count_max = counter;
	counter = 0;
	double trigger = 0;
	double PWM = PWM_min;
	
	//Main while loop of the script. This is where the script monitors pilot's input to channels 5 and 6. Decides to executes the maneuver if channel 5 is triggered. Channel 6 determines amplitude
	while(ros::ok()) {
		
		if (read_flag){
			trigger = rc_input.channels[4];
			PWM = rc_input.channels[5];
			read_flag = false;}
		
		switch (phase) {
			case 0:// The "off" phase: the script merely passes pilot's manual inputs. If switch for channel 5 is flipped, it resets initial time, intiializes maneuver and transitions to phase 1
				dA_com = 0.0; //Sets demanded aileron deflection to zero
				dE_com = 0.0; //Sets demanded elevator deflection to zero
				dR_com = 0.0; //Sets demanded rudder deflection to zero
				if (trigger > 1500){ //Condition to check if the switch for channel 5 is flipped
					t0 = ros::Time::now().toSec(); //sets the initial time of the maneuver 
					counter = 0; //sets the counter of the sequence to zero (uses same counter used to extract data from the .csv file
					phase = 1; //Changes the phase to "on"
					amp = amp_gain * (PWM - PWM_min); //computes the gain of the multisine maneuver deflection of the control surfaces based of rc channel 6;
					dA_com = dA[0] * amp; //assigns initial demanded aileron deflection of the maneuver 
					dE_com = dE[0] * amp; //assigns initial demanded elevator deflection of the maneuver
					dR_com = dR[0] * amp;} //assigns initial demanded rudder deflection of the maneuver
				break;
			case 1://The "on" phase: the script sequentially executes the pre-scripted multisine maneuver. Flipping the switch of channel 5 terminates and resets the maneuver. Channel 6 controls the gain of the multisine
				amp = amp_gain * (PWM - PWM_min); //computes the gain of the multisine maneuver deflection of the control surfaces based of rc channel 6;
				while ( t[counter] + dt + t0 < ros::Time::now().toSec() && counter <= count_max) { //Assumes the code's refresh rate is significantly faster than the prescribed time step of maneuver. Checks if the time elapsed since the start of the maneuver has exceeded the next time step. If so it updates the counter until it reaches the position of the highest time step during the maneuver less than the elapsed time
					counter++;} // Updates counter
				if (counter > 1000 || trigger < 1500){ //Checks for either of the conditions which would terminate the maneuver: Whether the time elapsed exceeds the total time of the maneuver or whether the switch for channel 5 have been flipped to the "off" setting
					dA_com = 0.0; // resets the aileron deflection to zero
					dE_com = 0.0; // resets the elevator deflection to zero
					dR_com = 0.0;// resets the rudder deflection to zero
					phase = 0;} // Resets the phase to "off"
				else {
					dA_com = dA[counter] * amp; // Passes the desired aileron deflection at the corresponding time step
					dE_com = dE[counter] * amp; // Passes the desired elevator deflection at the corresponding time step
					dR_com = dR[counter] * amp;} // Passes the desired rudder deflection at the corresponding time step
				break;
		}

		act_con.controls[0] = max( min( dA_com + manual_input.y, 1.0), -1.0); // writes the desired aileron deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[1] = max( min( dE_com - manual_input.x, 1.0), -1.0); // writes the desired elevator deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[2] = max( min( dR_com + manual_input.r, 1.0), -1.0);// writes the desired rudder deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[3] = max( min( 0.0 + manual_input.z, 1.0), 0.0); // writes the manual throttle setting and clips the signal to the range (0, 1)

		actuator_pub.publish(act_con); // Publishes the desired actuator control to the relevant ROS topic. MAVLINK and the Pixhawk take care of the rest from here on.
		ros::spinOnce(); // Nudges ROS to take wrap up any lingering threads or processes before the beginning of the next iteration of the while loop 
		rate.sleep(); // Pauses the while loop for enough time to ensure the prescribed rate is respected. Note that this may not be possible if the time required to process an iteration of the while loop exceeds the period corresponding to the rate.
	}

	return 0;
}
