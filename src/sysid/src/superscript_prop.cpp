#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <sensor_msgs/Imu.h> // linear_acceleration, angular_velocity, orientation (quaternion)
#include <mavros_msgs/ActuatorControl.h> // de, da, dr, dt (not directly, pwm)
#include<vector> // Header file which allows the use of vectors
#include<fstream> // Header file which enables reading and writing files via C++
#include<string> // Header file  which allows the use of strings in C++
using namespace std; // Saves us the trouble of having to write "std::" in front of every other function we use
//#include "rxtx.hpp"

//What ROS does is read the data from Pixhawk and dumps it in ROS topics.
//The lines of code below takes the data from the ROS topic called and stores to be used in the code.
mavros_msgs::State current_state;//State of pixhawk e.g. armed/disarmed
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

mavros_msgs::RCIn rc_input;//RC input values this step initializes the variable within the right class
int PTI = 0;
double amp = 0.0;
double amp_prop = 0;
int mode = 0;
int submode = 0;
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg){//This is a callback function to access the data in the topic
	rc_input = *msg;//This stores the data into the variable (This is the same for all the seubsequent lines till line 84)
	PTI = rc_input.channels[4];
	amp = 1.0*(rc_input.channels[5]-1100)/800.0;
	mode = rc_input.channels[8];
	submode = rc_input.channels[9];
	amp_prop = 0.2*(rc_input.channels[11]-1100)/800.0; //throttle amp is set by a three place switch to 0%, 10%, 20% deflection
}

mavros_msgs::ManualControl manual_input;//The default commanded servo values
void manual_cb(const mavros_msgs::ManualControl::ConstPtr& msg){
	manual_input = *msg;
}

sensor_msgs::Imu imu_data;//IMU data
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
	imu_data = *msg;
}

ros::Publisher actuator_pub;
int counter = 0;

//Multisine Initialization - aero only
vector<double> t_ms, dA_ms, dE_ms, dR_ms;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
int length_ms = 0;// variable to keep track of the length of the multisine vectors
void load_multisine(){
	ifstream doc;// Create variable to load .csv file
	//doc.open("/home/nsl/sysID/src/sysid/src/ms_input_100Hz.csv");// loads .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/ms_input_100Hz_0.1HzMin_2.0HzMax_aero_doublelength.csv");// loads .csv file for aerodynamic modeling
	//doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/ms_input_100Hz_0.5HzMin_6.0HzMax_servo.csv");// loads .csv file for servo modeling
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_ms.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA_ms.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE_ms.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dR_ms.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_ms++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
}

/* //Multisine Initialization - propulsion uniform bandwidth
vector<double> t_ms_uni, dA_ms_uni, dE_ms_uni, dR_ms_uni, dT_ms_uni;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
int length_ms_uni = 0;// variable to keep track of the length of the multisine vectors
void load_multisine_uni(){
	ifstream doc;// Create variable to load .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/UNIprop_ms_T40_0p05Hz_to_1p825Hz_prop.csv");// loads .csv file
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_ms_uni.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA_ms_uni.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE_ms_uni.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dR_ms_uni.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dT_ms_uni.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_ms_uni++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
} */

//Multisine Initialization - propulsion low bandwidth
vector<double> t_ms_lbw, dA_ms_lbw, dE_ms_lbw, dR_ms_lbw, dT_ms_lbw;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
int length_ms_lbw = 0;// variable to keep track of the length of the multisine vectors
void load_multisine_lbw(){
	ifstream doc;// Create variable to load .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/LBWprop_ms_T40_0p05Hz_to_1p825Hz_prop.csv");// loads .csv file
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_ms_lbw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA_ms_lbw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE_ms_lbw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dR_ms_lbw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dT_ms_lbw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_ms_lbw++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
}

//Multisine and Square wave Initialization - Aero multisine with propulsion square wave
vector<double> t_ms_sqwv, dA_ms_sqwv, dE_ms_sqwv, dR_ms_sqwv, dT_ms_sqwv;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
int length_ms_sqwv = 0;// variable to keep track of the length of the multisine vectors
void load_multisine_sqwv(){
	ifstream doc;// Create variable to load .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/4axis_sqw_input_prop.csv");// loads .csv file (note currently a placeholder)
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_ms_sqwv.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA_ms_sqwv.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE_ms_sqwv.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dR_ms_sqwv.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dT_ms_sqwv.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_ms_sqwv++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
}

//Frequency Sweep Initialization
vector<double> t_fs, dCS;// initializes arrays to store values from csv file. t -> time, dCS-> control surface command
int length_fs = 0;// variable to keep track of the length of the frequency sweep  vectors
void load_freq_sweep(){
	ifstream doc;// Create variable to load .csv file
	//doc.open("/home/nsl/sysID/src/sysid/src/freqswlog_input_100Hz.csv");// loads .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/freqswlog_input_100Hz_0.1HzMin_2.0HzMax_aero.csv");// loads .csv file for aerodynamic modeling frequency sweep
	//doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/freqswlog_input_100Hz_0.5HzMin_6.0HzMax_servo.csv");// loads .csv file for servo modeling frequency sweep
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() > 0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_fs.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dCS.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_fs++;}//increments the counter to keep track of how many steps imported from file
	doc.close(); // Clearing variable since we no longer need it
}

 //Square Wave Initialization - aero and propulsion 3-2-1-1 square wave
vector<double> t_sw, dA_sw, dE_sw, dR_sw, dT_sw;// initializes arrays to store values from csv file. t -> time, dA-> aileron command, dE -> elevator command, dR-> rudder command
int length_sw = 0;// variable to keep track of the length of the square wave vectors
void load_squarewave(){
	ifstream doc;// Create variable to load .csv file
	doc.open("/home/nsl/sysID/src/sysid/src/InputCSVs/4axis_sqw_input_CZ150_3211.csv");// loads .csv file
	string line, word, temp;// creates string variables
	getline(doc,line);// reads entire row and stores it in variable line and the removes that row from the variable doc
	while (line.length() >0) {//This while loop extracts the data from the .csv file and stores it in its corresponding arrays. The while condition checks if the line actually has any values
		stringstream s(line);//s is used to break line up into its individual "words" (numbers in string format)
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		t_sw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dA_sw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (comma in this case). It then stores the string in word and removes comma
		dE_sw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(s, word, ','); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dR_sw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
	    getline(s, word, '\n'); // extracts string from s (line) until it hits the delimiter (end of line in this case). It then stores the string in word
		dT_sw.push_back(stod(word));// converts the string stored in word to a double and appends it to the end of its respective vector
		getline(doc,line);// reads the next row row and stores it in variable line and the removes that row from the variable doc
		length_sw++;}//increments the counter in preparation for reading the following line and storing its enclosed values in the next position along the arrays
} 

void multisine_prop(ros::Rate rate){
	mavros_msgs::ActuatorControl act_con;// Initializes variable used to actuate servos
	double t0 = 0;// Records initial time
	double dt = 0.01; // Length of time step of maneuver
	int phase = 0;// The phase of the maneuver. 0 when off and 1 when it is being executed
	double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0,  dT_com = 0.0;// Initializes the commanded actuator inputs
	counter = 0;
	
	//Main while loop of the script. This is where the script monitors pilot's input to channels 5 and 6. Decides to executes the maneuver if channel 5 is PTIed. Channel 6 determines amplitude
	while(ros::ok() && mode <= 1333){
		
		switch (phase) {
			case 0:// The "off" phase: the script merely passes pilot's manual inputs. If switch for channel 5 is flipped, it resets initial time, intiializes maneuver and transitions to phase 1
				dA_com = 0.0; //Sets demanded aileron deflection to zero
				dE_com = 0.0; //Sets demanded elevator deflection to zero
				dR_com = 0.0; //Sets demanded rudder deflection to zero
				dT_com = 0.0; //Sets demanded throttle zero by default
				if (PTI > 1500){ //Condition to check if the switch for channel 5 is flipped
					t0 = ros::Time::now().toSec(); //sets the initial time of the maneuver 
					counter = 0; //sets the counter of the sequence to zero (uses same counter used to extract data from the .csv file
					phase = 1; //Changes the phase to "on"
					}
				break;
			case 1://The "on" phase: the script sequentially executes the pre-scripted multisine maneuver. Flipping the switch of channel 5 terminates and resets the maneuver. Channel 6 controls the gain of the multisine
				while ( t_ms[counter] + dt + t0 < ros::Time::now().toSec() && counter <= length_ms) { //Assumes the code's refresh rate is significantly faster than the prescribed time step of maneuver. Checks if the time elapsed since the start of the maneuver has exceeded the next time step. If so it updates the counter until it reaches the position of the highest time step during the maneuver less than the elapsed time
					counter++;} // Updates counter
				if (counter > length_ms || PTI < 1500){ //Checks for either of the conditions which would terminate the maneuver: Whether the time elapsed exceeds the total time of the maneuver or whether the switch for channel 5 have been flipped to the "off" setting
					dA_com = 0.0; // resets the aileron deflection to zero
					dE_com = 0.0; // resets the elevator deflection to zero
					dR_com = 0.0;// resets the rudder deflection to zero
					dT_com = 0.0;// resets the throttle deflection to zero
					phase = 0;} // Resets the phase to "off"
				else {
					/*if (submode > 1667){//uniform multisine
			            dA_com = dA_ms_uni[counter] * amp;
			            dE_com = dE_ms_uni[counter] * amp;
			            dR_com = dR_ms_uni[counter] * amp;
			            dT_com = dT_ms_uni[counter] * amp_prop;
			            } */ 
			        if (submode > 1667){//aero multisine
			            dA_com = dA_ms[counter] * 0.8*amp;
			            // (extra 2* for dE, dR specific to CZ150)
			            dE_com = dE_ms[counter] * 2.0*amp;
			            dR_com = dR_ms[counter] * 2.0*amp;
			            dT_com = 0.0;
			            } 
		            else if (submode > 1333){//Multisine with throttle LBW
			            dA_com = dA_ms_lbw[counter] * 0.8*amp;
			            // (extra 2* for dE, dR specific to CZ150)
			            dE_com = dE_ms_lbw[counter] * 2.0*amp;
			            dR_com = dR_ms_lbw[counter] * 2.0*amp;
			            dT_com = dT_ms_lbw[counter] * amp_prop;
			            } 
		            else{ //3-2-1-1 sequence de-dr-da-dt
			            dA_com = dA_ms_sqwv[counter] * 0.8*amp;
			            // (extra 2* for dE, dR specific to CZ150)
			            dE_com = dE_ms_sqwv[counter] * 2.0*amp; 
			            dR_com = dR_ms_sqwv[counter] * 2.0*amp;
			            dT_com = dT_ms_sqwv[counter] * amp_prop;
			            } 
					}

				break;
		}

		act_con.controls[0] = max( min( dA_com + manual_input.y, 1.0), -1.0); // writes the desired aileron deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[1] = max( min( dE_com - manual_input.x, 1.0), -1.0); // writes the desired elevator deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[2] = max( min( dR_com + manual_input.r, 1.0), -1.0);// writes the desired rudder deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[3] = max( min( dT_com + manual_input.z, 1.0), 0.0); // writes the manual throttle setting and clips the signal to the range (0, 1)

		actuator_pub.publish(act_con); // Publishes the desired actuator control to the relevant ROS topic. MAVLINK and the Pixhawk take care of the rest from here on.
		ros::spinOnce(); // Nudges ROS to take wrap up any lingering threads or processes before the beginning of the next iteration of the while loop 
		rate.sleep(); // Pauses the while loop for enough time to ensure the prescribed rate is respected. Note that this may not be possible if the time required to process an iteration of the while loop exceeds the period corresponding to the rate.
	}
}

void freq_sweep(ros::Rate rate){
	mavros_msgs::ActuatorControl act_con;// Initializes variable used to actuate servos
	double t0 = 0;// Records initial time
	double dt = 0.01; // Length of time step of maneuver
	int phase = 0;// The phase of the maneuver. 0 when off and 1 when it is being executed
	double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0, dT_com = 0.0, com = 0.0;// Initializes the commanded actuator inputs
	counter = 0;
	
	//Main while loop of the script. This is where the script monitors pilot's input to channels 5 and 6. Decides to executes the maneuver if channel 5 is PTIed. Channel 6 determines amplitude
	while(ros::ok() && mode < 1667 && mode > 1333){
		dA_com = 0.0; //Sets demanded aileron deflection to zero by default
		dE_com = 0.0; //Sets demanded elevator deflection to zero by default
		dR_com = 0.0; //Sets demanded rudder deflection to zero by default
		dT_com = 0.0; //Sets demanded throttle zero by default
		
		switch (phase) {
			case 0:// The "off" phase: the script merely passes pilot's manual inputs. If switch for channel 5 is flipped, it resets initial time, intiializes maneuver and transitions to phase 1
				com = 0.0; //Sets demanded command deflection to zero
				if (PTI > 1500){ //Condition to check if the switch for channel 5 is flipped
					t0 = ros::Time::now().toSec(); //sets the initial time of the maneuver 
					counter = 0; //sets the counter of the sequence to zero (uses same counter used to extract data from the .csv file
					phase = 1; //Changes the phase to "on"
					com = dCS[0] * amp;} //assigns initial demanded deflection of the maneuver
				break;
			case 1://The "on" phase: the script sequentially executes the pre-scripted maneuver. Flipping the switch of channel 5 terminates and resets the maneuver. Channel 6 controls the gain of the multisine
				while ( t_fs[counter] + dt + t0 < ros::Time::now().toSec() && counter <= length_fs) { //Assumes the code's refresh rate is significantly faster than the prescribed time step of maneuver. Checks if the time elapsed since the start of the maneuver has exceeded the next time step. If so it updates the counter until it reaches the position of the highest time step during the maneuver less than the elapsed time
					counter++;} // Updates counter
				if (counter > length_fs || PTI < 1500){ //Checks for either of the conditions which would terminate the maneuver: Whether the time elapsed exceeds the total time of the maneuver or whether the switch for channel 5 have been flipped to the "off" setting
					com = 0.0; // resets the deflection to zero
					phase = 0;} // Resets the phase to "off"
				else {
					com = dCS[counter] * amp;} // Passes the desired deflection at the corresponding time step
				break;
		}
			
			//dT_com = com; //Use this line for throttle frequency sweep, otherwise, use lines below
		/**/  //Comment out this section for propulsion frequency sweep
		if (submode > 1667){
			dE_com = com;} // Passes command to the rudder if channel 6 is between the second threshold and maximum
		else if (submode > 1333){
			dR_com = com;} // Passes command to the elevators if channel 6 is between the two thresholds
		else{
			dA_com = com;} // Passes command to ailerons if channel 6 is between the minimum and the first threshold
         // end of commented secion for propulsion frequency sweep
        
		act_con.controls[0] = max( min( 0.8*dA_com + manual_input.y, 1.0), -1.0); // writes the desired aileron deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[1] = max( min( 2.0*dE_com - manual_input.x, 1.0), -1.0); // writes the desired elevator deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[2] = max( min( 2.0*dR_com + manual_input.r, 1.0), -1.0);// writes the desired rudder deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[3] = max( min( dT_com + manual_input.z, 1.0), 0.0); // writes the manual throttle setting and clips the signal to the range (0, 1)

		actuator_pub.publish(act_con); // Publishes the desired actuator control to the relevant ROS topic. MAVLINK and the Pixhawk take care of the rest from here on.
		ros::spinOnce(); // Nudges ROS to take wrap up any lingering threads or processes before the beginning of the next iteration of the while loop 
		rate.sleep(); // Pauses the while loop for enough time to ensure the prescribed rate is respected. Note that this may not be possible if the time required to process an iteration of the while loop exceeds the period corresponding to the rate.
	}
	
}

void doublet(ros::Rate rate)
{
    mavros_msgs::ActuatorControl act_con;//Creates the variable used to actuate servos
    double t0 = 0;//Records initial time
//    double t0 = ros::Time::now().toSec();//Records initial time
    double t = 0.5;
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
    double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0, dT_com = 0.0, com = 0.0;// Initializes the commanded actuator inputs
	double com_high = 1;
	double com_low = -1;
	double temp = 0;
	double k_ph = 1.1;
	double k_th = 1.0;
	double k_p = 0.08;
	double k_q = 0.08;
    
    double dt = 0.01; // Length time of step maneuver of
    counter = 0;
	
	while(ros::ok() && mode >= 1667){
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

        	if (PTI < 1500){//Resets maneuver if pilot desires
        	    phase = 0;}

        	if (submode > 1666){//Longitudinal doublet maneuver
                t = 0.7;//elevator doublet period - ie short period (specific to CZ150)
                t1 = t/2; t2 = t/2;
	
        	    switch (phase) {//This phase is the idle phase to capture the effects of the doublet. Flipping Channel 5 to low resets the process and sets the phase to stand by phase
        	        case 0:
        	            dE_com= 0.0;
        	            if (PTI < 1500){
        	                phase = 1;}
        	            break;
        	        case 1://This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet
        	            dE_com= 0.0;
        	            if (PTI > 1500){
        	                t0 = ros::Time::now().toSec();
        	                temp = com_high;
        	                com_high = com_low;
        	                com_low = temp;
        	                dE_com = com_high;
        	                phase = 2;}
        	            break;
        	        case 2://This is the high phase of the doublet where the elevator is set to the high setting for the desired duration
        	            dE_com = com_high;
        	            if (ros::Time::now().toSec()-t0 >= t1){
        	                dE_com = com_low;
        	                phase = 3;}
        	            break;
        	        case 3://This is the low phase of the doublet where the elevator is set to its low setting for the desired duration
        	            dE_com = com_low;
        	            if (ros::Time::now().toSec()-t0 >= t1 + t2){
        	                dE_com = 0.0;
        	                phase = 0;}
        	            break;}
        	        dR_com = 0.0;
        	        dA_com = 0.0;
        	        act_con.controls[0] = std::max( std::min( 0.8*amp*dA_com + manual_input.y, 1.0), -1.0 );//Aileron servo (-1.0,1.0)
        	        //act_con.controls[0] = std::max( std::min( -k_ph*(sin_phi - 0.70710678 * manual_input.y) -k_p*p, 1.0), -1.0 );//Aileron servo (-1.0,1.0) If desired, this feedback keeps aircraft level
	                // (extra 2* for dE, dR specific to CZ150)
                    act_con.controls[1] = std::max( std::min( 2.0*amp*dE_com - manual_input.x, 1.0), -1.0 );//Elevator servo (-1.0,1.0) 
        	        act_con.controls[2] = std::max( std::min( 2.0*amp*dR_com + manual_input.r, 1.0), -1.0 );//Rudder servo (-1.0,1.0)
                    act_con.controls[3] = std::max( std::min( amp_prop*dT_com + manual_input.z, 1.0), -1.0 );//Propeller throttle (0.0,1.0)
                            	        
        	        actuator_pub.publish(act_con);//Writes actuator commands based on variable value
                	//ros::spinOnce();//Calls all callbacks waiting to becalled at that point in time
                	//rate.sleep();//Sleeps for a period of time based on the frequency or rate defined for ROS to run
        	    }//End of Long Doublet
        	    
        	else if (submode > 1333){//LatDir doublet maneuver
                t = 1.25;//rudder doublet period - ie dutch roll period (specific to CZ150)
                t1 = t/2; t2 = t/2; t3 = 4.0;
                
        	    switch (phase) {//This phase is the idle phase to capture the effects of the doublet. Flipping Channel 5 to low resets the process and sets the phase to stand by phase
        	        case 0:
        	            dR_com= 0.0;
        	            dA_com= 0.0;
        	            if (PTI < 1500){
        	                phase = 1;}
        	            break;
        	        case 1://This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet
        	            dR_com= 0.0;
        	            dA_com = 0.0;
        	            if (PTI > 1500){
        	                t0 = ros::Time::now().toSec();
        	                temp = com_high;
        	                com_high = com_low;
        	                com_low = temp;
        	                dR_com = com_high;
        	                phase = 2;}
        	            break;
        	        case 2://This is the high phase of the doublet where the elevator is set to the high setting for the desired duration
        	            dR_com = com_high;
        	            dA_com = 0.0;
        	            if (ros::Time::now().toSec()-t0 >= t1){
        	                dR_com = com_low;
        	                phase = 3;}
        	            break;
        	        case 3://This is the low phase of the doublet where the elevator is set to its low setting for the desired duration
        	            dR_com = com_low;
        	            dA_com = 0.0;
        	            if (ros::Time::now().toSec()-t0 >= t1 + t2){
        	                dR_com = 0.0;
        	                phase = 4;}//if aileron 1-2-1 is not needed change the value from 4 to 0
        	            break;
        	        case 4://Interim phase to capture effects of rudder double
        	            dR_com= 0.0;
        	            dA_com= 0.0;
        	            if (ros::Time::now().toSec()-t0 >= t1 + t2 + t3){
        	                dA_com = com_low;
        	                phase = 5;}
        	            break;
        	        case 5://first part of 1-2-1 aileron maneuver
        	            dR_com= 0.0;
        	            dA_com= com_low;
        	            if (sin_phi * com_low >= 0.707){
        	                dA_com = com_high;
        	                phase = 6;}
        	            break;
        	        case 6://second part of 1-2-1 aileron maneuver
        	            dR_com= 0.0;
        	            dA_com= com_high;
        	            if (sin_phi * com_high >= 0.707){
        	                dA_com = com_low;
        	                phase = 7;}
        	            break;
        	        case 7://third and last part of 1-2-1 aileron maneuver
        	            dR_com= 0.0;
        	            dA_com= com_low;
        	            if (sin_phi * com_low >= -0.0871557427){
        	                dA_com = 0.0;
        	                phase = 0;}
        	            break;}
        	        dE_com = 0.0;
	                act_con.controls[0] = std::max( std::min( 0.8*amp*dA_com + manual_input.y, 1.0), -1.0 );//Aileron servo (-1.0,1.0)
	                // (extra 2* for dE, dR specific to CZ150)
                    act_con.controls[1] = std::max( std::min( 2.0*amp*dE_com - manual_input.x, 1.0), -1.0 );//Elevator servo (-1.0,1.0) 
        	        act_con.controls[2] = std::max( std::min( 2.0*amp*dR_com + manual_input.r, 1.0), -1.0 );//Rudder servo (-1.0,1.0)
        	        act_con.controls[3] = std::max( std::min( amp_prop*dT_com + manual_input.z, 1.0), -1.0 );//Propeller throttle (0.0,1.0)
        	        
        	        actuator_pub.publish(act_con);//Writes actuator commands based on variable value
                	//ros::spinOnce();//Calls all callbacks waiting to becalled at that point in time
                	//rate.sleep();//Sleeps for a period of time based on the frequency or rate defined for ROS to run
	            }//End of Lat-Dir Doublet
	            
	            /* //Thottle doublet commented out unless propulsion testing
            else {//Throttle doublet 
                t = 20;//throttle doublet period [sec]
                t1 = t/2; t2 = t/2;
	
        	    switch (phase) {//This phase is the idle phase to capture the effects of the doublet. Flipping Channel 5 to low resets the process and sets the phase to stand by phase
        	        case 0:
        	            dT_com= 0.0;
        	            if (PTI < 1500){
        	                phase = 1;}
        	            break;
        	        case 1://This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet
        	            dT_com= 0.0;
        	            if (PTI > 1500){
        	                t0 = ros::Time::now().toSec();
        	                temp = com_high;
        	                com_high = com_low;
        	                com_low = temp;
        	                dT_com = com_high;
        	                phase = 2;}
        	            break;
        	        case 2://This is the high phase of the doublet where the elevator is set to the high setting for the desired duration
        	            dT_com = com_high;
        	            if (ros::Time::now().toSec()-t0 >= t1){
        	                dT_com = com_low;
        	                phase = 3;}
        	            break;
        	        case 3://This is the low phase of the doublet where the elevator is set to its low setting for the desired duration
        	            dT_com = com_low;
        	            if (ros::Time::now().toSec()-t0 >= t1 + t2){
        	                dT_com = 0.0;
        	                phase = 0;}
        	            break;}
        	        dE_com = 0.0;
        	        dR_com = 0.0;
        	        dA_com = 0.0;
        	        act_con.controls[0] = std::max( std::min( amp*dA_com + manual_input.y, 1.0), -1.0 );//Aileron servo (-1.0,1.0)
        	        //act_con.controls[0] = std::max( std::min( -k_ph*(sin_phi - 0.70710678 * manual_input.y) -k_p*p, 1.0), -1.0 );//Aileron servo (-1.0,1.0) If desired, this feedback keeps wings level
	                // (extra 2* for dE, dR specific to CZ150)
                    act_con.controls[1] = std::max( std::min( 2.0*amp*dE_com - manual_input.x, 1.0), -1.0 );//Elevator servo (-1.0,1.0) 
        	        act_con.controls[2] = std::max( std::min( 2.0*amp*dR_com + manual_input.r, 1.0), -1.0 );//Rudder servo (-1.0,1.0)
                    act_con.controls[3] = std::max( std::min( amp*dT_com + manual_input.z, 1.0), -1.0 );//Propeller throttle (0.0,1.0)
                            	        
        	        actuator_pub.publish(act_con);//Writes actuator commands based on variable value
        	        }  */ //end of throttle doublet

    // 3-2-1-1 commented out for propulsion testing
            else {//3-2-1-1 combined square wave maneuver
	//This is where the script monitors pilot's input to channels 5 and 6. Decides to executes the maneuver if channel 5 is PTIed. Channel 6 determines amplitude
		switch (phase) {
			case 0:// The "off" phase: the script merely passes pilot's manual inputs. If switch for channel 5 is flipped, it resets initial time, intiializes maneuver and transitions to phase 1
				dA_com = 0.0; //Sets demanded aileron deflection to zero
				dE_com = 0.0; //Sets demanded elevator deflection to zero
				dR_com = 0.0; //Sets demanded rudder deflection to zero
				dT_com = 0.0;// Sets demanded throttle deflection to zero
				if (PTI > 1500){ //Condition to check if the switch for channel 5 is flipped
					t0 = ros::Time::now().toSec(); //sets the initial time of the maneuver 
					counter = 0; //sets the counter of the sequence to zero (uses same counter used to extract data from the .csv file
					phase = 1; //Changes the phase to "on"
					dA_com = dA_sw[0] * 0.8*amp; //assigns initial demanded aileron deflection of the maneuver 
					dE_com = dE_sw[0] * 2.0*amp; //assigns initial demanded elevator deflection of the maneuver
					dR_com = dR_sw[0] * 2.0*amp; //assigns initial demanded rudder deflection of the maneuver
					dT_com = dT_sw[0] * amp_prop;}
				break;
			case 1://The "on" phase: the script sequentially executes the pre-scripted multisine maneuver. Flipping the switch of channel 5 terminates and resets the maneuver. Channel 6 controls the gain of the multisine
				while ( t_sw[counter] + dt + t0 < ros::Time::now().toSec() && counter <= length_sw) { //Assumes the code's refresh rate is significantly faster than the prescribed time step of maneuver. Checks if the time elapsed since the start of the maneuver has exceeded the next time step. If so it updates the counter until it reaches the position of the highest time step during the maneuver less than the elapsed time
					counter++;} // Updates counter
				if (counter > length_sw - 1 || PTI < 1500){ //Checks for either of the conditions which would terminate the maneuver: Whether the time elapsed exceeds the total time of the maneuver or whether the switch for channel 5 have been flipped to the "off" setting
					dA_com = 0.0; // resets the aileron deflection to zero
					dE_com = 0.0; // resets the elevator deflection to zero
					dR_com = 0.0;// resets the rudder deflection to zero
					dT_com = 0.0;// resets the throttle deflection to zero
					phase = 0;} // Resets the phase to "off"
				else {
					dA_com = dA_sw[counter] * 0.8*amp; // Passes the desired aileron deflection at the corresponding time step
					dE_com = dE_sw[counter] * 2.0*amp; // Passes the desired elevator deflection at the corresponding time step
					dR_com = dR_sw[counter] * 2.0*amp; // Passes the desired rudder deflection at the corresponding time step
					dT_com = dT_sw[counter] * amp_prop;}
				break;
		}
        //ROS_INFO_STREAM(to_string(counter)); //for troubleshooting
        ROS_INFO_STREAM(to_string(length_sw - counter) + " " + to_string(t_sw[counter]) + " " + to_string(dA_sw[counter]) + " " + to_string(dE_sw[counter]) + " " + to_string(dR_sw[counter]));
		act_con.controls[0] = max( min( dA_com + manual_input.y, 1.0), -1.0); // writes the desired aileron deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[1] = max( min( dE_com - manual_input.x, 1.0), -1.0); // writes the desired elevator deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[2] = max( min( dR_com + manual_input.r, 1.0), -1.0);// writes the desired rudder deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[3] = max( min( dT_com + manual_input.z, 1.0), 0.0); // writes the manual throttle setting and clips the signal to the range (0, 1)

		actuator_pub.publish(act_con); // Publishes the desired actuator control to the relevant ROS topic. MAVLINK and the Pixhawk take care of the rest from here on.
            }    //end of 3-2-1-1
            
            ros::spinOnce(); // Nudges ROS to take wrap up any lingering threads or processes before the beginning of the next iteration of the while loop 
            rate.sleep(); // Pauses the while loop for enough time to ensure the prescribed rate is respected. Note that this may not be possible if the time required to process an iteration of the while loop exceeds the period corresponding to the rate.
                     	       
        }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "off_node");//Initializes ROS
	ros::NodeHandle nh;//Creates Node handle
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, rcin_cb);
	ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>("mavros/manual_control/control", 1, manual_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
	actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
	ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1, imu_cb);
    // wait for FCU connection
    	ros::Rate rate(200.0);//Defines the rate at which your code will run. Note that the code will not run faster than this rate but may run slower if its computationally taxing
    while(ros::ok() && !current_state.connected){
        ROS_INFO_STREAM("Waiting!");
        ros::spinOnce();
        rate.sleep();
    }
    load_multisine();
	//load_multisine_uni();
	load_multisine_lbw();
	load_multisine_sqwv();
	load_freq_sweep();
	load_squarewave();

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        ros::spinOnce();
        rate.sleep();
    }
    


    while(ros::ok()){
		if(mode < 1333)
			multisine_prop(rate);
		else if (mode < 1666)
			freq_sweep(rate);
		else
			doublet(rate);
    }

    return 0;
}

