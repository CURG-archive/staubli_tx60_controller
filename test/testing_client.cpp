#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "staubli_tx60/GetCartesian.h"
#include "staubli_tx60/GetJoints.h"
#include "staubli_tx60/SetJointsAction.h"
#include "staubli_tx60/SetCartesianAction.h"
#include "staubli_tx60/FwdKinematics.h"
#include "staubli_tx60/InvKinematics.h"

int test1(){
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<staubli_tx60::GetCartesian>("getCartesian");

   staubli_tx60::GetCartesian srv1;

   if (client.call(srv1)) {
      ROS_INFO("Service called");
      ROS_INFO("Got robot's Cartesian positoin:\nx: %lf\ny: %lf\nz: %lf\nrx: %lf\nry: %lf\nrz: %lf",
	    (double)srv1.response.x,
	    (double)srv1.response.y,
	    (double)srv1.response.z,
	    (double)srv1.response.rx*180/M_PI,
	    (double)srv1.response.ry*180/M_PI,
	    (double)srv1.response.rz*180/M_PI  );
   }
   else {
      ROS_ERROR("Failed to call service");
      return 1;
   }
   return 0;
}

int test2(){
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<staubli_tx60::GetJoints>("getJoints");

   staubli_tx60::GetJoints srv1;

   if (client.call(srv1)) {
      ROS_INFO("Service called");
      ROS_INFO("Got robot's Joints:\n%lf\n%lf\n%lf\n%lf\n%lf\n%lf",
	    (double)srv1.response.j[0]*180/M_PI,
	    (double)srv1.response.j[1]*180/M_PI,
	    (double)srv1.response.j[2]*180/M_PI,
	    (double)srv1.response.j[3]*180/M_PI,
	    (double)srv1.response.j[4]*180/M_PI,
	    (double)srv1.response.j[5]*180/M_PI );
   }
   else {
      ROS_ERROR("Failed to call service");
      return 1;
   }
   return 0;
}

void test3(){
   std::vector<double> joints;
   joints.resize(6);
   scanf( "%lf%lf%lf%lf%lf%lf",
   & joints[0], 
   & joints[1],
   & joints[2],
   & joints[3],
   & joints[4],
   & joints[5] );

   actionlib::SimpleActionClient<staubli_tx60::SetJointsAction> ac("setJoints", true);
     ROS_INFO("Waiting for action server to start.");
     ac.waitForServer(); //will wait for infinite time
     staubli_tx60::SetJointsGoal goal;
     goal.j = joints;
     ac.sendGoal(goal);

     double toWait;
     scanf( "%lf", &toWait);
     bool finished_before_timeout = ac.waitForResult(ros::Duration(toWait));
     if( !finished_before_timeout ) ac.cancelGoal();
}

void test4(){
   double x,y,z,rx,ry,rz;
   int lineCtrl;
   scanf( "%lf%lf%lf%lf%lf%lf%d",
	 &x, &y, &z, &rx, &ry, &rz, &lineCtrl);

   actionlib::SimpleActionClient<staubli_tx60::SetCartesianAction> ac("setCartesian", true);
     ROS_INFO("Waiting for action server to start.");
     ac.waitForServer(); //will wait for infinite time
     staubli_tx60::SetCartesianGoal goal;
     goal.x = x;
     goal.y = y;
     goal.z = z;
     goal.rx = rx;
     goal.ry = ry;
     goal.rz = rz;
     goal.lineCtrl = lineCtrl;
     ac.sendGoal(goal);

     double toWait;
     bool finished_before_timeout = ac.waitForResult(ros::Duration(0));
     if( !finished_before_timeout ) ac.cancelGoal();
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "testing_client");
   test4();


   return 0;
}
