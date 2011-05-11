/*
 * controller.h
 *
 *  Created on: Feb 9, 2010
 *      Author: joseph
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

////////////////////////////////////////////////////////////////////////////
// IMPORTANT NOTICE
////////////////////////////////////////////////////////////////////////////
// STAUBLI_IP should be available in your environment.
// You may assign STAUBLI_IP to the IP of your physical controller(i.e CS8)
// or the computer in which CS8 emulator is running.
////////////////////////////////////////////////////////////////////////////

//#define STAUBLI_IP "http://192.168.1.254:5653/" //physical CS8
#define STAUBLI_IP "http://128.59.20.63:5653/"	//ANTIGUA
//#define STAUBLI_IP "http://128.59.19.61:5653/"	//Thomas' computer


//Declare functions
void print(double e);
int GetPosition(staubli::command::Response *res);
int GetJoints(staubli::command::Response *res);
int MoveJoints(staubli::command::Request  *req,
			   staubli::command::Response *res);
int MoveStraightLine(staubli::command::Request  *req,
					 staubli::command::Response *res);
int ForwardKinematics(staubli::command::Request  *req,
					 staubli::command::Response *res);
int InverseKinematics(staubli::command::Request  *req,
					 staubli::command::Response *res);
bool commandhandler(staubli::command::Request  &req,
					staubli::command::Response &res );

#endif /* CONTROLLER_H_ */
