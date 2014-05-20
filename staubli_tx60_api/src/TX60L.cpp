#include "TX60L.h"
#include "soapCS8ServerV0Proxy.h"
#include "soapCS8ServerV1Proxy.h"
#include "soapCS8ServerV3Proxy.h"
#include "CS8ServerV0.nsmap"
#include <math.h>
#include <iostream>
using std::string;
using std::cout;
using std::endl;
#include <boost/thread.hpp>

boost::mutex RobotLock;


//---------------------------------------Server V0---------------------------------//
bool TX60L::Login(const string& url, const string& userName, const string& password){
  boost::lock_guard<boost::mutex> lock(RobotLock);
	if(mLogin)
		delete mLogin;
	if(mLoginResponse)
		delete mLoginResponse;
	if(mCS8ServerV0){
		Logoff();
		delete mCS8ServerV0;
	}
	if(mCS8ServerV1){
		delete mCS8ServerV1;
	}
	mEndpoint = url;
	mEndpointV1 = url + "CS8ServerV1";
	mEndpointV3 = url + "CS8ServerV3";
	mCS8ServerV0 = new CS8ServerV0Proxy();
	mCS8ServerV0->soap_endpoint = mEndpoint.c_str();
	mLogin = new _ns1__login();
	mLoginResponse = new _ns1__loginResponse();
	mLogin->user = &const_cast<string&>(userName);
	mLogin->pwd = &const_cast<string&>(password);
	mCS8ServerV0->login(mLogin, mLoginResponse);
	mIsLoggedIn = (mLoginResponse->sid != 0);



	mCS8ServerV1 = new CS8ServerV1Proxy();
	mCS8ServerV1->soap_header(&(mLoginResponse->sid));
	mCS8ServerV1->soap_endpoint = mEndpointV1.c_str();

	mCS8ServerV3 = new CS8ServerV3Proxy();
	mCS8ServerV3->soap_header(&(mLoginResponse->sid));
	mCS8ServerV3->soap_endpoint = mEndpointV3.c_str();
    lastRequestSucceeded = mIsLoggedIn;
	return mIsLoggedIn;
}

void TX60L::Logoff(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
   return;

	_ns1__logout *logout = new _ns1__logout();
	_ns1__logoutResponse *logoutResponse = new _ns1__logoutResponse();
    lastRequestSucceeded = mCS8ServerV0->logout(logout, logoutResponse) == SOAP_OK;
    mIsLoggedIn = false;
	delete logout;
	delete logoutResponse;
}

bool TX60L::SetJoints(const std::vector<double>& joints){
  boost::lock_guard<boost::mutex> lock(RobotLock);
	if(joints.size() != 6)
    {
        lastRequestSucceeded = false;
        return false;
    }
    if(!mIsLoggedIn)
    {
        lastRequestSucceeded = false;
        return false;
    }
	_ns1__setRobotJointPos *setJoint = new _ns1__setRobotJointPos();
	_ns1__setRobotPosResponse *setJointResponse = new _ns1__setRobotPosResponse();
	ns1__JointPos *jp = new ns1__JointPos();
	jp->item.clear();
	for(size_t i = 0; i < joints.size(); ++i){
		jp->item.push_back(joints[i]);
	}
	setJoint->pos = jp;
    lastRequestSucceeded = mCS8ServerV0->setRobotJointPos(setJoint, setJointResponse) == SOAP_OK;
	delete setJoint;
	delete setJointResponse;
	delete jp;
	return true;
}

bool TX60L::GetRobotJoints(std::vector<double>& joints){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns1__getRobotJointPos* robotJoints = new _ns1__getRobotJointPos();
	_ns1__getRobotJointPosResponse* robotJointsResponse = new _ns1__getRobotJointPosResponse();
    lastRequestSucceeded = mCS8ServerV0->getRobotJointPos(robotJoints, robotJointsResponse) == SOAP_OK;

	joints = robotJointsResponse->pos->item;
	delete robotJoints;
	delete robotJointsResponse;
	if(joints.size() != 6){
		joints.clear();
        lastRequestSucceeded = false;
		return false;
	}

    return lastRequestSucceeded;
}

bool TX60L::GetRobotCartesianPosition(std::vector<double>& position){
  boost::lock_guard<boost::mutex> lock(RobotLock);
	if(!mIsLoggedIn)
    {
        lastRequestSucceeded = false;
		return false;
    }
	_ns1__getRobotJntCartPos* cartesianPos = new _ns1__getRobotJntCartPos();
	_ns1__getRobotJntCartPosResponse* cartesianPosResponse = new _ns1__getRobotJntCartPosResponse();
    if(mCS8ServerV0->getRobotJntCartPos(cartesianPos, cartesianPosResponse))
    {
        lastRequestSucceeded = false;
        return false;
    }
	position.clear();

	position.push_back(cartesianPosResponse->cartPos->x);
	position.push_back(cartesianPosResponse->cartPos->y);
	position.push_back(cartesianPosResponse->cartPos->z);
	position.push_back(cartesianPosResponse->cartPos->rx);
	position.push_back(cartesianPosResponse->cartPos->ry);
	position.push_back(cartesianPosResponse->cartPos->rz);
	delete cartesianPos;
	delete cartesianPosResponse;
    lastRequestSucceeded = true;
	return true;
}

bool TX60L::GetRobots(std::vector<int>& robots){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns1__getRobots * robotsInput = new _ns1__getRobots();
	_ns1__getRobotsResponse * robotsResponse = new _ns1__getRobotsResponse();
    if(mCS8ServerV0->getRobots(robotsInput, robotsResponse))
    {
        lastRequestSucceeded = false;
        return false;
    }
	delete robotsInput;
	delete robotsResponse;
	return true;
}

//----------------------------------------Server V1----------------------------------------//
bool TX60L::GetApplications(std::vector<std::string>& appNames){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns2__getApplications * applications = new _ns2__getApplications();
	_ns2__getApplicationsResponse * applicationsResponse = new _ns2__getApplicationsResponse();
    lastRequestSucceeded = mCS8ServerV1->getApplications(applications, applicationsResponse) == SOAP_OK;
	for(size_t i = 0; i < applicationsResponse->applications->application.size(); ++i){
		appNames.push_back(* (applicationsResponse->applications->application[i]->name) );
	}
	delete applications;
	delete applicationsResponse;
    return lastRequestSucceeded;
}

bool TX60L::GetJointRange(std::vector<double>& min, std::vector<double>& max){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns2__getJointRange * jointRange = new _ns2__getJointRange();
	_ns2__getJointRangeResponse * jointRangeResponse = new _ns2__getJointRangeResponse();
    lastRequestSucceeded = mCS8ServerV1->getJointRange(jointRange, jointRangeResponse) == SOAP_OK;
	min = jointRangeResponse->range->min_;
	max = jointRangeResponse->range->max_;
	delete jointRange;
	delete jointRangeResponse;
    return lastRequestSucceeded;
}

//------------------------------Server V3------------------------------//
bool TX60L::ResetMotion(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  std::cout<< "Motion Reset\n";
	if(!mIsLoggedIn)
		return false;
	_ns6__resetMotion * resetMotion = new _ns6__resetMotion();
	_ns6__motionResponse * resetMotionResponse = new _ns6__motionResponse();
    lastRequestSucceeded = mCS8ServerV3->resetMotion(resetMotion, resetMotionResponse) == SOAP_OK;

	delete resetMotion;
	delete resetMotionResponse;
    return lastRequestSucceeded;
}

bool TX60L::MoveJoints(std::vector<double> jnts, int btype, double jointvel, 
		       double acc, double dec, double tooltipvel, double rotvel, double blendPrev, double blendNext){
  boost::lock_guard<boost::mutex> lock(RobotLock);
	if(!mIsLoggedIn)
    {
        lastRequestSucceeded = false;
        return false;
    }
	_ns6__moveJJ * joints = new _ns6__moveJJ();
	_ns6__moveResponse * jointsResponse = new _ns6__moveResponse();

	// initialize the joint positions
	ns1__JointPos *jointPos = new ns1__JointPos();
	jointPos->item = jnts;
	joints->joint = jointPos;

	//initialize the motion desctiptor
    ns6__MotionDesc * md = new ns6__MotionDesc(ns6__BlendType(btype), blendPrev, blendNext,
						   jointvel, acc, dec,
						   tooltipvel, rotvel);
	InitializeMotionDesc(md);
	joints->mdesc = md;

    lastRequestSucceeded = mCS8ServerV3->moveJJ(joints, jointsResponse) == SOAP_OK;
	//cout << "ret good? " << (ret==SOAP_OK) << endl;


	delete joints;
	delete jointsResponse;

    return lastRequestSucceeded;
}

bool TX60L::IsJointQueueEmpty(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  _ns6__schedulerRefresh  sr_req;
  _ns6__MotionAndRobotsPos sr_resp;
  if(mCS8ServerV3->schedulerRefresh(&sr_req, &sr_resp) != SOAP_OK)
    std::cout<< "TX60L::IsJointQueueEmpty::SOAP ERROR \n";
  return sr_resp.motionEmpty;   
}

bool TX60L::IsRobotSettled(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
  _ns6__schedulerRefresh  sr_req;
  _ns6__MotionAndRobotsPos sr_resp;
  if(mCS8ServerV3->schedulerRefresh(&sr_req, &sr_resp) != SOAP_OK)
  {
    std::cout<< "TX60L::IsJointQueueEmpty::SOAP ERROR \n";
    lastRequestSucceeded = false;
  }
  return sr_resp.allRobotsPos->RobotsPos[0]->settled;   

}

bool TX60L::InverseKinematics(std::vector<double> pos, std::vector<double> start, std::vector<double> &jnts){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns6__reverseKin *inverseKin = new _ns6__reverseKin();
	_ns6__reverseKinResponse *inverseKinResponse = new _ns6__reverseKinResponse();

	// initialize the joint positions
	// initial joint angles
	ns1__JointPos *jointPos = new ns1__JointPos();
	jointPos->item.push_back(start[0]);
	jointPos->item.push_back(start[1]);
	jointPos->item.push_back(start[2]);
	jointPos->item.push_back(start[3]);
	jointPos->item.push_back(start[4]);
	jointPos->item.push_back(start[5]);
	inverseKin->jointIn = jointPos;

	//Initialize Frame
	inverseKin->target = new ns6__Frame();
	inverseKin->target->px=pos[0];
	inverseKin->target->py=pos[1];
    inverseKin->target->pz=pos[2];
	SetFrameFromRxRyRz(inverseKin->target, pos[3], pos[4], pos[5]);

	// Initialize the config
	inverseKin->config = new ns6__Config();
	InitializeConfig(inverseKin->config);

	// Initalize jointRange
	ns2__JointRange *jRange = new ns2__JointRange();
	inverseKin->jointRange = jRange;

    lastRequestSucceeded = mCS8ServerV3->reverseKin(inverseKin, inverseKinResponse) == SOAP_OK;
	jnts=inverseKinResponse->jointOut->item;

	delete inverseKin;
	delete inverseKinResponse;
    return lastRequestSucceeded;
}

bool TX60L::ForwardKinematics(std::vector<double> jnts, std::vector<double> &pos){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	double Rx=0,Ry=0,Rz=0;
	_ns6__forwardKin *forwardKin = new _ns6__forwardKin();
	_ns6__forwardKinResponse *forwardKinResponse = new _ns6__forwardKinResponse();

	//int robot;	/* required element of type xsd:int */
	//ns1__JointPos *joint;	/* required element of type ns1:JointPos */

	// initialize the joint positions
	ns1__JointPos *jointPos = new ns1__JointPos();
	jointPos->item = jnts;
	forwardKin->joint = jointPos;

    lastRequestSucceeded = mCS8ServerV3->forwardKin(forwardKin, forwardKinResponse) == SOAP_OK;
	ns6__Frame *responseFrame= new ns6__Frame();
	responseFrame=forwardKinResponse->position;
	
    GetRxRyRzCoord(responseFrame, &Rx,&Ry,&Rz);

	pos.push_back(forwardKinResponse->position->px);
	pos.push_back(forwardKinResponse->position->py);
	pos.push_back(forwardKinResponse->position->pz);
	pos.push_back(RAIDAN_2_DEGREE(Rx));
	pos.push_back(RAIDAN_2_DEGREE(Ry));
	pos.push_back(RAIDAN_2_DEGREE(Rz));

	ns6__Frame *tmp = new ns6__Frame();
	SetFrameFromRxRyRz(tmp, Rx, Ry, Rz);

	delete forwardKin;
	delete forwardKinResponse;
    return lastRequestSucceeded;
}
bool TX60L::Power(bool on){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns6__setPower * power = new _ns6__setPower();
	power->power = on;
	_ns6__setPowerResponse * powerResponse = new _ns6__setPowerResponse();
    lastRequestSucceeded = mCS8ServerV3->setPower(power, powerResponse) == SOAP_OK;
	delete power;
	delete powerResponse;
    return lastRequestSucceeded;
}

bool TX60L::Stop(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns6__stopMotion * motion = new _ns6__stopMotion();
	_ns6__motionResponse * motionResponse = new _ns6__motionResponse();
    lastRequestSucceeded = mCS8ServerV3->stopMotion(motion, motionResponse) == SOAP_OK;
	delete motion;
	delete motionResponse;
    return lastRequestSucceeded;
}

bool TX60L::Restart(){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns6__restartMotion * motion = new _ns6__restartMotion();
	_ns6__motionResponse * motionResponse = new _ns6__motionResponse();
    lastRequestSucceeded = mCS8ServerV3->restartMotion(motion, motionResponse) == SOAP_OK;
	delete motion;
	delete motionResponse;
    return lastRequestSucceeded;
}

bool TX60L::MoveLine(std::vector<double> pos, double jointvel, double acc, double dec){
  boost::lock_guard<boost::mutex> lock(RobotLock);
  if(!mIsLoggedIn)
  {
      lastRequestSucceeded = false;
      return false;
  }
	_ns6__moveL * moveL = new _ns6__moveL();
	
	//initialize the frame
	moveL->frame = new ns6__Frame();
	moveL->frame->px=pos[0];
	moveL->frame->py=pos[1];
	moveL->frame->pz=pos[2];
	SetFrameFromRxRyRz(moveL->frame, pos[3], pos[4], pos[5]);

	//initialize the motion desctiptor
	ns6__MotionDesc * md = new ns6__MotionDesc();
	md->vel = jointvel;
	md->acc = acc;
	md->dec = dec;

	InitializeMotionDesc(md);
	moveL->mdesc = md;
	_ns6__moveResponse * moveResponse = new _ns6__moveResponse();

    lastRequestSucceeded = mCS8ServerV3->moveL(moveL, moveResponse) == SOAP_OK;
	//cout << "ret good? " << (ret==SOAP_OK) << endl;

	delete moveL;
	delete moveResponse;
    return lastRequestSucceeded;
}

void TX60L::GetRxRyRzCoord(ns6__Frame *x_fr, double *x_Rx, double *x_Ry, double *x_Rz)
{

	double SMALL_FLOAT = pow(10.0, -6.0);
    double l_sinRy;

    l_sinRy = x_fr->ax;
    // ATTENTION : it may be possible that sinRy > 1.0 or < -1.0 (numerical pbm)
    if (l_sinRy < (-1.0 + SMALL_FLOAT * SMALL_FLOAT / 2.0)) // -1.0
    {
        *x_Rx = 0;
        *x_Ry = -PI / 2.0;
        *x_Rz = atan2(x_fr->ny, x_fr->nz);
    }
    else if (l_sinRy > (1.0 - SMALL_FLOAT * SMALL_FLOAT / 2.0))  //1.0
    {
        *x_Rx = 0;
        *x_Ry = PI / 2.0;
        *x_Rz = atan2(x_fr->ny, -x_fr->nz);
    }
    else
    {
        double l_sign = 1.0;
        *x_Ry = asin(l_sinRy);
        if ((x_fr->az < 0.0) && (x_fr->nx < 0.0)
            && ((abs(l_sinRy) > SMALL_FLOAT * SMALL_FLOAT) || (abs(x_fr->ay) > SMALL_FLOAT * SMALL_FLOAT)))
        {	// if cosRy > 0, and both cosRx and cosRz are < 0, choose Ry so that
            // cosRx and cosRz are positive
            if (*x_Ry >= 0.0)
                *x_Ry = PI - *x_Ry;
            else
                *x_Ry = -PI - *x_Ry;
            l_sign = -1.0;
        }
        *x_Rx = atan2(-l_sign * x_fr->ay, l_sign * x_fr->az);
        *x_Rz = atan2(-l_sign * x_fr->ox, l_sign * x_fr->nx);
    }
}

void 
TX60L::SetFrameFromRxRyRz(ns6__Frame *x_fr, double x_Rx, double x_Ry, double x_Rz)
{

	double l_sinRx, l_sinRy, l_sinRz;
	double l_cosRx, l_cosRy, l_cosRz;

	l_sinRx = sin(x_Rx);
	l_sinRy = sin(x_Ry);
	l_sinRz = sin(x_Rz);

	l_cosRx = cos(x_Rx);
	l_cosRy = cos(x_Ry);
	l_cosRz = cos(x_Rz);

	x_fr->nx = l_cosRz * l_cosRy;
	x_fr->ny = l_cosRz * l_sinRy * l_sinRx + l_sinRz * l_cosRx;
	x_fr->nz = -l_cosRz * l_sinRy * l_cosRx + l_sinRz * l_sinRx;

	x_fr->ox = -l_sinRz * l_cosRy;
	x_fr->oy = l_cosRz * l_cosRx - l_sinRz * l_sinRy * l_sinRx;
	x_fr->oz = l_cosRz * l_sinRx + l_sinRz * l_sinRy * l_cosRx;

	x_fr->ax = l_sinRy;
	x_fr->ay = -l_cosRy * l_sinRx;
	x_fr->az = l_cosRy * l_cosRx;
}

void TX60L::InitializeMotionDesc(ns6__MotionDesc* md){
	md->tool = new ns6__Frame();
	md->frame = new ns6__Frame();
	md->config = new ns6__Config();
	InitializeConfig(md->config);
}

void TX60L::InitializeConfig(ns6__Config * conf){
	conf->__union_Config = 1;
	union _ns6__union_Config uConfig;
	uConfig.anthroConfig=new ns6__AnthroConfig();
	uConfig.anthroConfig->shoulder = ((enum ns6__ShoulderConfig)3);
	uConfig.anthroConfig->elbow = ((enum ns6__PositiveNegativeConfig)3);
	uConfig.anthroConfig->wrist = ((enum ns6__PositiveNegativeConfig)3);
	conf->union_Config =uConfig;
}

