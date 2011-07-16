/*
 *	wiic_twist.cpp
 *
 * This file was converted from the WiiC example for ROS, written by:
 *    Kevin Walchko
 *    Email: kevin dot walchko at gmail dot com
 *
 *	WiiC Example file is part of WiiC, written by:
 *		Gabriele Randelli
 *		Email: randelli@dis.uniroma1.it
 *
 *	Copyright 2010
 *
 *	Original file is based on WiiuseCpp, written By:
 *		James Thomas
 *		Email: jt@missioncognition.net
 *
 *	Copyright 2009
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// C++
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h>        
#include <sensor_msgs/Imu.h>     
#include <std_msgs/UInt8.h>                 

// Wii
#include <wiicpp.h>
#include <wiic_twist/LEDControl.h>
#include <wiic_twist/IrSourceInfo.h>
#include <wiic_twist/RumbleControl.h>
#include <wiic_twist/State.h>
#include <wiic_twist/TimedSwitch.h>

using namespace std;

CWii wii; // Defaults to 4 remotes
CWiimote wm;

bool motor_on = false;

int LED_MAP[4] = {CWiimote::LED_1, CWiimote::LED_2, CWiimote::LED_3, CWiimote::LED_4};


void HandleStatus(CWiimote &wm)
{
	ROS_INFO("\n");
	ROS_INFO("--- CONTROLLER STATUS [wiimote id %i] ---\n", wm.GetID());
	
	ROS_INFO("attachment: %i", wm.ExpansionDevice.GetType());
	ROS_INFO("motion plus: %d", wm.isUsingMotionPlus());
	ROS_INFO("external devices: %d", wm.isUsingEXP());
	ROS_INFO("speaker: %i", wm.isUsingSpeaker());
	ROS_INFO("ir: %i", wm.isUsingIR());
	ROS_INFO("leds: %i %i %i %i", wm.isLEDSet(1), wm.isLEDSet(2), wm.isLEDSet(3), wm.isLEDSet(4));
	ROS_INFO("battery: %f %%", wm.GetBatteryLevel());
	
	CExpansionDevice::ExpTypes ex = wm.ExpansionDevice.GetType();
	switch(ex){
		case CExpansionDevice::TYPE_NONE:
			ROS_INFO("expansion NONE");
			break;
		case CExpansionDevice::TYPE_NUNCHUK:
			ROS_INFO("expansion TYPE_NUNCHUK");
			break;
		case CExpansionDevice::TYPE_MOTION_PLUS:
			ROS_INFO("expansion TYPE_MOTION_PLUS");
			break;
		default:
			ROS_INFO("expansion UNKNOWN");
	}
}


// change to byte: <xxxx bit3 bit2 bit1 bit0>
void ledsReceived(const wiic_twist::LEDControl::ConstPtr& leds)
{
	;
}

void rumbleReceived(const wiic_twist::RumbleControl::ConstPtr& rumble)
{
	for(int i=0;i<3;i++){
		usleep(200000);
	wm.SetRumbleMode(CWiimote::ON);
	usleep(200000);
	wm.SetRumbleMode(CWiimote::OFF);
	}
}

bool sameTwist(const geometry_msgs::Twist& a, const geometry_msgs::Twist& b){
	
	return ((a.linear.x == b.linear.x) && (a.linear.y == b.linear.y) && (a.linear.z == b.linear.z) &&
			  (a.angular.x == b.angular.x) && (a.angular.y == b.angular.y) && (a.angular.z == b.angular.z));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "wiic_twist");
	ros::NodeHandle n;
	ros::Rate r(100.0); // run at 100 Hz to keep up with wiimote
	
	// Publish
	ros::Publisher cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	//ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
	ros::Publisher state_pub = n.advertise<wiic_twist::State>("/state", 50);
	
	// Subscribe
	ros::Subscriber leds_sub = n.subscribe<wiic_twist::LEDControl>("/leds", 1, ledsReceived);
	ros::Subscriber rumble_sub = n.subscribe<wiic_twist::RumbleControl>("/rumble", 1, rumbleReceived);
	
	std::vector<CWiimote>::iterator i;
	int reloadWiimotes = 0;
	int numFound;
	bool keep_running = true;
	geometry_msgs::Twist t;
	geometry_msgs::Twist last_t;
	
	ROS_INFO("Searching for wiimotes... Turn them on!");
	
	//Find the wiimote, the number is the timeout (seconds)
	numFound = wii.Find(4);  
	
	// Connect to the wiimote
	std::vector<CWiimote>& wiimotes = wii.Connect();
	int numConnected = (int)wiimotes.size();
	
	ROS_INFO("Found %d wiimotes ... Connected to %d wiimotes", numFound, numConnected);
	
	if(numConnected != 1){ // only connecting to one controller
		ROS_ERROR("Too many controllers found");
		exit(-1);
	}
	
	// grab first (and only) wiimote
	wm = wiimotes[0];
	
	
	// Set Leds
	wm.SetLEDs(LED_MAP[0]);
	
	//Rumble for 0.2 seconds as a connection ack
	wm.SetRumbleMode(CWiimote::ON);
	usleep(200000);
	wm.SetRumbleMode(CWiimote::OFF);
	
	// setup
	wm.SetMotionSensingMode(CWiimote::ON);
	wm.EnableMotionPlus(CWiimote::OFF);
	wm.IR.SetMode(CIR::OFF);
	
	wm.UpdateStatus();
	
	HandleStatus(wm);
	
	do
	{
		// Something has changed (i.e., added or removed a nunchuck for example)
		// need to reload what the wiimotes are equiped with
		if(reloadWiimotes)
		{
			// Regenerate the list of wiimotes
			wiimotes = wii.GetWiimotes();
			reloadWiimotes = 0;
		}
		
		
		//Poll the wiimotes to get the status like pitch or roll
		if(wii.Poll()) // removed the wii polling func?
		{
			// disconnect works better with this function here
			int i = wm.GetEvent();
			switch(i)
			{
					
				case CWiimote::EVENT_EVENT:
					break;
					
				case CWiimote::EVENT_STATUS:
					HandleStatus(wm);
					break;
					
				case CWiimote::EVENT_DISCONNECT:
				case CWiimote::EVENT_UNEXPECTED_DISCONNECT:
					ROS_INFO("disconnecting"); // could use this to end loop
					keep_running = false;
					reloadWiimotes = 1;
					break;
				default:
					ROS_INFO("Other wm.GetEvent() has occurred: %d",i);
			}
			
			////////////////////////////////////////////////////////////////////
			wiic_twist::State state;
			state.header.stamp = ros::Time::now();
			state.header.frame_id = "wiimote";
			
			state.buttons[wiic_twist::State::MSG_BTN_1] = wm.Buttons.isPressed(CButtons::BUTTON_ONE);
			state.buttons[wiic_twist::State::MSG_BTN_2] = wm.Buttons.isPressed(CButtons::BUTTON_TWO);
			state.buttons[wiic_twist::State::MSG_BTN_A] = wm.Buttons.isPressed(CButtons::BUTTON_A);
			state.buttons[wiic_twist::State::MSG_BTN_B] = wm.Buttons.isPressed(CButtons::BUTTON_B);
			state.buttons[wiic_twist::State::MSG_BTN_UP] = wm.Buttons.isPressed(CButtons::BUTTON_UP);
			state.buttons[wiic_twist::State::MSG_BTN_DOWN] = wm.Buttons.isPressed(CButtons::BUTTON_DOWN);
			state.buttons[wiic_twist::State::MSG_BTN_LEFT] = wm.Buttons.isPressed(CButtons::BUTTON_LEFT);
			state.buttons[wiic_twist::State::MSG_BTN_RIGHT] = wm.Buttons.isPressed(CButtons::BUTTON_RIGHT);
			state.buttons[wiic_twist::State::MSG_BTN_PLUS] = wm.Buttons.isPressed(CButtons::BUTTON_PLUS);
			state.buttons[wiic_twist::State::MSG_BTN_MINUS] = wm.Buttons.isPressed(CButtons::BUTTON_MINUS);
			
			state.percent_battery = wm.GetBatteryLevel();
			
			// Acceleration
			geometry_msgs::Vector3 accel;
			float x,y,z;
			wm.Accelerometer.GetGravityVector(x,y,z);
			accel.x = x;
			accel.y = y;
			accel.z = z;
			state.linear_acceleration_raw = accel;
			//ROS_INFO("accell: %f %f %f",x,y,z);
			
			// Angular velocity
			geometry_msgs::Vector3 gyro;
			wm.Accelerometer.GetOrientation(x, y, z);
			gyro.x = x;
			gyro.y = y;
			gyro.z = z;
			state.angular_velocity_zeroed = gyro;
			wm.Accelerometer.GetRawOrientation(x, y);
			gyro.x = x;
			gyro.y = y;
			gyro.z = z;
			state.angular_velocity_raw = gyro;
			
			state_pub.publish(state);
			
			////////////////////////////////////////////////////////////////////
			geometry_msgs::Twist t;
			
			// forward/backwards
			if(wm.Buttons.isPressed(CButtons::BUTTON_UP)) t.linear.x = 1.0;
			else if(wm.Buttons.isPressed(CButtons::BUTTON_DOWN)) t.linear.x = -1.0;
			else t.linear.x = 0.0;
			
			// turn
			if(wm.Buttons.isPressed(CButtons::BUTTON_LEFT)) t.angular.z = 1.0;
			else if(wm.Buttons.isPressed(CButtons::BUTTON_RIGHT)) t.angular.z = -1.0;
			else t.angular.z = 0.0;
			
			// publish to robot
			if( sameTwist(t,last_t) ); // don't publish repeats
			else{
				cmdVel_pub.publish(t);
				last_t = t;
			}
			/////////////////////////////////////////////////////////////////////
			//std_msgs::UInt8 motors;
			
			
			//if(wm.Buttons.isPressed(CButtons::BUTTON_B)) HandleStatus(wm);
			//if(wm.Buttons.isPressed(CButtons::BUTTON_A)) wm.Disconnect();
			
			/////////////////////////////////////////////////////////////////////
			//sensor_msgs::Imu imu;
			//imu_pub.publish(imu);
			
		}
		
		ros::spinOnce();
		r.sleep();
		
		// Go so long as there are wiimotes left to poll and ROS is ok
	} while(keep_running && n.ok()); 
	
	wm.Disconnect();
	
	return 0;
}
