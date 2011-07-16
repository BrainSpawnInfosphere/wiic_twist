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

bool motor_on = false;

int LED_MAP[4] = {CWiimote::LED_1, CWiimote::LED_2, CWiimote::LED_3, CWiimote::LED_4};

void HandleEvent(CWiimote &wm)
{
    char prefixString[64];

	/*
    if(wm.Buttons.isJustPressed(CButtons::BUTTON_MINUS))
    {
        wm.SetMotionSensingMode(CWiimote::OFF);
    }

    if(wm.Buttons.isJustPressed(CButtons::BUTTON_PLUS))
    {
        wm.SetMotionSensingMode(CWiimote::ON);
    }

    if(wm.Buttons.isJustPressed(CButtons::BUTTON_DOWN))
    {
        //wm.IR.SetMode(CIR::OFF);
    }

    if(wm.Buttons.isJustPressed(CButtons::BUTTON_UP))
    {
        //wm.IR.SetMode(CIR::ON);
    }

    if(wm.Buttons.isJustPressed(CButtons::BUTTON_RIGHT))
    {
        //wm.EnableMotionPlus(CWiimote::ON);
		  //cout << "Motion Plus is: "<< (wm.isUsingMotionPlus() ? "ON" : "OFF") << endl;
    }

    if(wm.Buttons.isJustPressed(CButtons::BUTTON_LEFT))
    {
     	 //wm.EnableMotionPlus(CWiimote::OFF);
		 //cout << "Motion Plus is: "<< (wm.isUsingMotionPlus() ? "ON" : "OFF") << endl;
    }
*//*
    if(wm.Buttons.isJustPressed(CButtons::BUTTON_B))
    {
        //wm.ToggleRumble();
		 motor_on = true;
    }
	 else motor_on = false;
	*/
	
	if(wm.Buttons.isPressed(CButtons::BUTTON_B))
	{
		//wm.ToggleRumble();
		motor_on = true;
	}
	else if(wm.Buttons.isReleased(CButtons::BUTTON_B)) motor_on = false;
	
	/*
    sprintf(prefixString, "Wiimote[%i]: ", wm.GetID());

    if(wm.Buttons.isPressed(CButtons::BUTTON_A))
    {
        printf("%s A pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_B))
    {
        printf("%s B pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_UP))
    {
        printf("%s Up pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_DOWN))
    {
        printf("%s Down pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_LEFT))
    {
        printf("%s Left pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_RIGHT))
    {
        printf("%s Right pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_MINUS))
    {
        printf("%s Minus pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_PLUS))
    {
        printf("%s Plus pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_ONE))
    {
        printf("%s One pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_TWO))
    {
        printf("%s Two pressed\n", prefixString);
    }

    if(wm.Buttons.isPressed(CButtons::BUTTON_HOME))
    {
        printf("%s Home pressed\n", prefixString);
    }
*/
	/*
    // if the accelerometer is turned on then print angles
    if(wm.isUsingACC())
    {
        float pitch, roll, yaw, a_pitch, a_roll;
        wm.Accelerometer.GetOrientation(pitch, roll, yaw);
        wm.Accelerometer.GetRawOrientation(a_pitch, a_roll);
        printf("%s wiimote roll = %f [%f]\n", prefixString, roll, a_roll);
        printf("%s wiimote pitch = %f [%f]\n", prefixString, pitch, a_pitch);
        printf("%s wiimote yaw = %f\n", prefixString, yaw);
    }

	// if the Motion Plus is turned on then print angles
    if(wm.isUsingMotionPlus()) {
    	float roll_rate, pitch_rate, yaw_rate;
    	wm.ExpansionDevice.MotionPlus.Gyroscope.GetRates(roll_rate,pitch_rate,yaw_rate);

        printf("%s motion plus roll rate = %f\n", prefixString,roll_rate);
    	printf("%s motion plus pitch rate = %f\n", prefixString,pitch_rate);
    	printf("%s motion plus yaw rate = %f\n", prefixString,yaw_rate);
    }
	 */
/*
    // if(IR tracking is on then print the coordinates
    if(wm.isUsingIR())
    {
        std::vector<CIRDot>::iterator i;
        int x, y;
        int index;

        printf("%s Num IR Dots: %i\n", prefixString, wm.IR.GetNumDots());
        printf("%s IR State: %u\n", prefixString, wm.IR.GetState());

        std::vector<CIRDot>& dots = wm.IR.GetDots();

        for(index = 0, i = dots.begin(); i != dots.end(); ++index, ++i)
        {
            if((*i).isVisible())
            {
                (*i).GetCoordinate(x, y);
                printf("%s IR source %i: (%i, %i)\n", prefixString, index, x, y);

                wm.IR.GetCursorPosition(x, y);
                printf("%s IR cursor: (%i, %i)\n", prefixString, x, y);
                printf("%s IR z distance: %f\n", prefixString, wm.IR.GetDistance());
            }
        }
    }
*/
    int exType = wm.ExpansionDevice.GetType();
	
	//printf("expansion device: %d\n",exType);
	
    if(exType == wm.ExpansionDevice.TYPE_NUNCHUK)
    {
        float pitch, roll, yaw, a_pitch, a_roll;
        float angle, magnitude;

        CNunchuk &nc = wm.ExpansionDevice.Nunchuk;

        sprintf(prefixString, "Nunchuk [%i]: ", wm.GetID());

        if(nc.Buttons.isPressed(CNunchukButtons::BUTTON_C))
        {
            printf("%s C pressed\n", prefixString);
        }

        if(nc.Buttons.isPressed(CNunchukButtons::BUTTON_Z))
        {
            printf("%s Z pressed\n", prefixString);
        }

        nc.Accelerometer.GetOrientation(pitch, roll, yaw);
        nc.Accelerometer.GetRawOrientation(a_pitch, a_roll);
        printf("%s roll = %f [%f]\n", prefixString, roll, a_roll);
        printf("%s pitch = %f [%f]\n", prefixString, pitch, a_pitch);
        printf("%s yaw = %f\n", prefixString, yaw);

        nc.Joystick.GetPosition(angle, magnitude);
        printf("%s joystick angle = %f\n", prefixString, angle);
        printf("%s joystick magnitude = %f\n", prefixString, magnitude);
    }
	/*
    else if(exType == wm.ExpansionDevice.TYPE_CLASSIC)
    {
        float angle, magnitude;

        CClassic &cc = wm.ExpansionDevice.Classic;

        sprintf(prefixString, "Classic [%i]: ", wm.GetID());

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_A))
        {
            printf("%s A pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_B))
        {
            printf("%s B pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_X))
        {
            printf("%s X pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_Y))
        {
            printf("%s Y pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_LEFT))
        {
            printf("%s Left pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_UP))
        {
            printf("%s Up pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_RIGHT))
        {
            printf("%s Right pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_DOWN))
        {
            printf("%s Down pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_PLUS))
        {
            printf("%s Plus pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_MINUS))
        {
            printf("%s Minus pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_HOME))
        {
            printf("%s Home pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_ZL))
        {
            printf("%s ZL pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_FULL_L))
        {
            printf("%s ZR pressed\n", prefixString);
        }

        if(cc.Buttons.isPressed(CClassicButtons::BUTTON_FULL_R))
        {
            printf("%s ZR pressed\n", prefixString);
        }

        printf("%s L button pressed = %f\n", prefixString, cc.GetLShoulderButton());
        printf("%s R button pressed = %f\n", prefixString, cc.GetRShoulderButton());

        cc.LeftJoystick.GetPosition(angle, magnitude);
        printf("%s left joystick angle = %f\n", prefixString, angle);
        printf("%s left joystick magnitude = %f\n", prefixString, magnitude);

        cc.RightJoystick.GetPosition(angle, magnitude);
        printf("%s right joystick angle = %f\n", prefixString, angle);
        printf("%s right joystick magnitude = %f\n", prefixString, magnitude);
    }
	 */
}

void HandleStatus(CWiimote &wm)
{
    printf("\n");
    printf("--- CONTROLLER STATUS [wiimote id %i] ---\n\n", wm.GetID());

    printf("attachment: %i\n", wm.ExpansionDevice.GetType());
    printf("speaker: %i\n", wm.isUsingSpeaker());
    printf("ir: %i\n", wm.isUsingIR());
    printf("leds: %i %i %i %i\n", wm.isLEDSet(1), wm.isLEDSet(2), wm.isLEDSet(3), wm.isLEDSet(4));
    printf("battery: %f %%\n", wm.GetBatteryLevel());
}

void HandleDisconnect(CWiimote &wm)
{
    printf("\n");
    printf("--- DISCONNECTED [wiimote id %i] ---\n", wm.GetID());
    printf("\n");
}

void HandleReadData(CWiimote &wm)
{
    printf("\n");
    printf("--- DATA READ [wiimote id %i] ---\n", wm.GetID());
    printf("\n");
}

void HandleNunchukInserted(CWiimote &wm)
{
    printf("Nunchuk inserted on controller %i.\n", wm.GetID());
}
/*
void HandleClassicInserted(CWiimote &wm)
{
    printf("Classic controler inserted on controller %i.\n", wm.GetID());
}
*/
void ledsReceived(const wiic_twist::LEDControl::ConstPtr& leds)
{
	;
}

void rumbleReceived(const wiic_twist::RumbleControl::ConstPtr& rumble)
{
	;
	//wm.ToggleRumble(); // how do I get
}

class cState {
public:
	cState(){
		state = false;
	}
	
	bool test(bool b){
		bool changed = (state == b ? false : true); // has state changed?
		if(changed) state = !state;
		return changed;
	}
private:
	bool state;
};
	

typedef struct {
	cState motor;
} wii_state_t;
	

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wiic_twist");
	ros::NodeHandle n;
	ros::Rate r(100.0); // run at 100 Hz to keep up with wiimote
	
	// Publish
	ros::Publisher cmdVel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
	ros::Publisher brushes_pub = n.advertise<std_msgs::UInt8>("/brushes", 50);
	
	// Subscribe
	ros::Subscriber leds_sub = n.subscribe<wiic_twist::LEDControl>("/leds", 1, ledsReceived);
	ros::Subscriber rumble_sub = n.subscribe<wiic_twist::RumbleControl>("/rumble", 1, rumbleReceived);

	
	//CWii wii; // Defaults to 4 remotes
    std::vector<CWiimote>::iterator i;
    int reloadWiimotes = 0;
    int numFound;
    int index;

    ROS_INFO("Searching for wiimotes... Turn them on!");

    //Find the wiimote
	// only looking for 1 wiimote, if you want 5 wiimotes, then put 5 here
	numFound = wii.Find(5);  

    // Search for up to five seconds;

    ROS_INFO("Found %d wiimotes", numFound);
    //cout << "Connecting to wiimotes..." << endl;

    // Connect to the wiimote
    std::vector<CWiimote>& wiimotes = wii.Connect();

	ROS_INFO("Connected to %d wiimotes", (int)wiimotes.size());

    // Setup the wiimotes
    for(index = 0, i = wiimotes.begin(); i != wiimotes.end(); ++i, ++index)
    {
        // Use a reference to make working with the iterator handy.
        CWiimote & wiimote = *i;

        //Set Leds
        wiimote.SetLEDs(LED_MAP[index]);

        //Rumble for 0.2 seconds as a connection ack
        wiimote.SetRumbleMode(CWiimote::ON);
        usleep(200000);
        wiimote.SetRumbleMode(CWiimote::OFF);
		 
		 // Enable Accels
		 wiimote.SetMotionSensingMode(CWiimote::ON);
		 ROS_INFO("Wiimote[%d] accellerometers are: %s",index,(wiimote.isUsingACC() ? "ON" : "OFF"));
		 
		 
		 // enable Motion Plus if present
		 wiimote.EnableMotionPlus(CWiimote::ON);
		 ROS_INFO("Wiimote[%d] Motion Plus is: %s",index, (wiimote.isUsingMotionPlus() ? "ON" : "OFF"));
    }
	
	bool keep_running = false;
	
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
			  // don't need to handle multiple controllers ... change to just one
            for(i = wiimotes.begin(); i != wiimotes.end(); ++i)
            {
                // Use a reference to make working with the iterator handy.
                CWiimote & wiimote = *i;
#if 0
                switch(wiimote.GetEvent())
                {

                    case CWiimote::EVENT_EVENT:
                        HandleEvent(wiimote);
                        break;

                    case CWiimote::EVENT_STATUS:
                        HandleStatus(wiimote);
                        break;

                    case CWiimote::EVENT_DISCONNECT:
                    case CWiimote::EVENT_UNEXPECTED_DISCONNECT:
                        HandleDisconnect(wiimote);
                        reloadWiimotes = 1;
                        break;

                    case CWiimote::EVENT_READ_DATA:
                        HandleReadData(wiimote);
                        break;

                    case CWiimote::EVENT_NUNCHUK_INSERTED:
                        HandleNunchukInserted(wiimote);
                        reloadWiimotes = 1;
                        break;
/*
                    case CWiimote::EVENT_CLASSIC_CTRL_INSERTED:
                        HandleClassicInserted(wiimote);
                        reloadWiimotes = 1;
                        break;
*/
						  case CWiimote::EVENT_MOTION_PLUS_INSERTED:
								ROS_INFO("Motion Plus inserted.");
								break;
						
                    case CWiimote::EVENT_NUNCHUK_REMOVED:
                    //case CWiimote::EVENT_CLASSIC_CTRL_REMOVED:
                    //case CWiimote::EVENT_GUITAR_HERO_3_CTRL_REMOVED:
						  case CWiimote::EVENT_MOTION_PLUS_REMOVED:
                        ROS_INFO("An expansion was removed.");
                        HandleStatus(wiimote);
                        reloadWiimotes = 1;
                        break;

                    default:
							 ROS_ERROR("unknown message");
                        break;
                }
#endif
					
					//char prefixString[64];
					
					//sprintf(prefixString, "Wiimote[%i]: ", wiimote.GetID());
					/*
					if(wiimote.isUsingACC())
					{
						float pitch, roll, yaw, a_pitch, a_roll;
						wiimote.Accelerometer.GetOrientation(pitch, roll, yaw);
						wiimote.Accelerometer.GetRawOrientation(a_pitch, a_roll);
						//printf("%s wiimote roll = %f [%f]\n", prefixString, roll, a_roll);
						//printf("%s wiimote pitch = %f [%f]\n", prefixString, pitch, a_pitch);
						//printf("%s wiimote yaw = %f\n", prefixString, yaw);
					}
					
					// if the Motion Plus is turned on then print angles
					if(wiimote.isUsingMotionPlus()) {
						float roll_rate, pitch_rate, yaw_rate;
						wiimote.ExpansionDevice.MotionPlus.Gyroscope.GetRates(roll_rate,pitch_rate,yaw_rate);
						
						//printf("%s motion plus roll rate = %f\n", prefixString,roll_rate);
						//printf("%s motion plus pitch rate = %f\n", prefixString,pitch_rate);
						//printf("%s motion plus yaw rate = %f\n", prefixString,yaw_rate);
					}
					*/
					////////////////////////////////////////////////////////////////////
					geometry_msgs::Twist t;
					/*
					if(wiimote.isUsingACC())
					{
						float pitch, roll, yaw, a_pitch, a_roll;
						wiimote.Accelerometer.GetOrientation(pitch, roll, yaw);
						wiimote.Accelerometer.GetRawOrientation(a_pitch, a_roll);
						//printf("wiimote roll = %f [%f]\n", roll, a_roll);
						//printf("wiimote pitch = %f [%f]\n", pitch, a_pitch);
						//printf("wiimote yaw = %f\n", yaw);
						//printf("wiimote (roll,pitch): %f %f\n", roll, pitch);
						
						t.linear.x = pitch;
						t.linear.y = roll;
					}
					 */
					
					// forward/backwards
					if(wiimote.Buttons.isPressed(CButtons::BUTTON_UP)) t.linear.x = .2;
					else if(wiimote.Buttons.isPressed(CButtons::BUTTON_DOWN)) t.linear.x = -.2;
					else t.linear.x = 0.0;
					
					// turn
					if(wiimote.Buttons.isPressed(CButtons::BUTTON_LEFT)) t.angular.z = .3;
					else if(wiimote.Buttons.isPressed(CButtons::BUTTON_RIGHT)) t.angular.z = -.3;
					else t.angular.z = 0.0;
					
					// publish to robot
					cmdVel_pub.publish(t);
					
					/////////////////////////////////////////////////////////////////////
					std_msgs::UInt8 motors;
					
					
					if(motor_on)
					//if(wiimote.Buttons.isPressed(CButtons::BUTTON_B))
					{
						motors.data = 1;
					}
					else motors.data = 0;
					
					//if(motors > 0){
						brushes_pub.publish(motors);
					//}
									
					/////////////////////////////////////////////////////////////////////
					sensor_msgs::Imu imu;
					imu_pub.publish(imu);
            }
        }
		 
		 // Go so long as there are wiimotes left to poll and ROS is ok
		 keep_running = (wiimotes.size() && n.ok());
		 
		 //motor_on = false;
		 
		 ros::spinOnce();
		 r.sleep();

    } while(keep_running); 

    return 0;
}
