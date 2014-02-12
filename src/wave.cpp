#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "staubli_tx60/GetCartesian.h"
#include "staubli_tx60/GetJoints.h"
#include "staubli_tx60/SetJointsAction.h"
#include "staubli_tx60/SetCartesianAction.h"
#include "staubli_tx60/FwdKinematics.h"
#include "staubli_tx60/InvKinematics.h"
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

void wave() {
   double x,y,z,rx,ry,rz;
   int lineCtrl;

   actionlib::SimpleActionClient<staubli_tx60::SetCartesianAction> ac("setCartesian", true);
   ROS_INFO("Waiting for action server to start.");
   ac.waitForServer(); //will wait for infinite time
   staubli_tx60::SetCartesianGoal goal;

   int tem = fcntl(0, F_GETFL, 0);
   fcntl (0, F_SETFL, (tem | O_NDELAY));

   ROS_INFO("Press ENTER to quit" );

   while(1) {
      goal.x = .10;
      goal.y = -.4;
      goal.z = .4;
      goal.rx = 1.57;
      goal.ry = 0;
      goal.rz = 3.54;
      goal.lineCtrl = 1;
      ac.sendGoal(goal);

      bool finished_before_timeout = ac.waitForResult(ros::Duration(0));
      if( !finished_before_timeout ) ac.cancelGoal();

      char c;
      int  n;
      n = read(0, &c, 1);
      if( n>0 ) break;

      goal.x = .25;
      goal.y = -.4;
      goal.z = .4;
      goal.rx = 1.57;
      goal.ry = 0;
      goal.rz = 2.84;
      goal.lineCtrl = 1;
      ac.sendGoal(goal);

      finished_before_timeout = ac.waitForResult(ros::Duration(0));
      if( !finished_before_timeout ) ac.cancelGoal();

      n = read(0, &c, 1);
      if( n>0 ) break;

   }
   fcntl(0, F_SETFL, tem);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "wave");
   wave();
   return 0;
}
