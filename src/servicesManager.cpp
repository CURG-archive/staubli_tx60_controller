#include "servicesManager.h"
#include "CS8ServerV0.nsmap"

#define ERROR_EPSILON 1E-4

ServicesManager::ServicesManager(ros::NodeHandle node_handle, TX60L staubli)
{
    services.push_back(node_handle.advertiseService("getCartesian", &ServicesManager::getCartesianCB, this));
    services.push_back(node_handle.advertiseService("getJoints",    &ServicesManager::getJointsCB,    this));
    services.push_back(node_handle.advertiseService("getRotMat",    &ServicesManager::getRotMatCB,    this));
    services.push_back(node_handle.advertiseService("fwdKinematics",&ServicesManager::fwdKinematicsCB,this));
    services.push_back(node_handle.advertiseService("invKinematics",&ServicesManager::invKinematicsCB,this));
    services.push_back(node_handle.advertiseService("cancelMotion", &ServicesManager::cancelMotionCB, this));
}

bool ServicesManager::cancelMotionCB(staubli_tx60::ResetMotion::Request & req,
          staubli_tx60::ResetMotion::Response & res)
{
  staubli.ResetMotion();
  res.succeeded = true;
  return true;


}

bool ServicesManager::getCartesianCB(staubli_tx60::GetCartesian::Request  &req,
      staubli_tx60::GetCartesian::Response &res ) 
{

   std::vector<double> position;
   position.resize(6);
   if(staubli.GetRobotCartesianPosition(position)){
      res.x  = position[0];
      res.y  = position[1];
      res.z  = position[2];
      res.rx = position[3];
      res.ry = position[4];
      res.rz = position[5];
      ROS_INFO("Robot's Cartesian position sent.");
      return true;
   }else {
      ROS_ERROR("Cannot get Cartesian position.");
      return false;
   }
}

bool ServicesManager::getRotMatCB(staubli_tx60::GetRotMat::Request  &req,
      staubli_tx60::GetRotMat::Response &res ) 
{
   std::vector<double> position;
   position.resize(6);
   if(staubli.GetRobotCartesianPosition(position)){
      ns6__Frame fr;
      staubli.SetFrameFromRxRyRz( &fr, position[3], position[4], position[5] );
      res.m.clear();

      res.m.push_back(fr.nx); res.m.push_back(fr.ox); res.m.push_back(fr.ax);
      res.m.push_back(fr.ny); res.m.push_back(fr.oy); res.m.push_back(fr.ay);
      res.m.push_back(fr.nz); res.m.push_back(fr.oz); res.m.push_back(fr.az);

      return true;
   } else {
      ROS_ERROR("Cannot Query Arm's Rotation.");
      return false;
   }
}

bool ServicesManager::getJointsCB(staubli_tx60::GetJoints::Request  &req,
      staubli_tx60::GetJoints::Response &res ) 
{
   std::vector<double> joints;
   joints.resize(6);
   if(staubli.GetRobotJoints(joints)){
      res.j = joints;
      ROS_INFO("Robot's joints' configuratoin sent.");
      return true;
   }else {
      ROS_ERROR("Cannot get joints' configuration.");
      return false;
   }
}

bool ServicesManager::fwdKinematicsCB(staubli_tx60::FwdKinematics::Request  &req,
      staubli_tx60::FwdKinematics::Response &res ) 
{
   std::vector<double> pos;
   if(staubli.ForwardKinematics( req.j, pos ))
   {
      res.x  =  (double) pos[0];
      res.y  =  (double) pos[1];
      res.z  =  (double) pos[2];
      res.rx =  (double) pos[3] / 180.0 * M_PI;
      res.ry =  (double) pos[4] / 180.0 * M_PI;
      res.rz =  (double) pos[5] / 180.0 * M_PI;
      ROS_INFO("Robot's forward kinematics sent." );
      return true;
   }
   else 
   {
      ROS_ERROR("Cannot get forward kinematics.");
      return false;
   }
}

bool ServicesManager::invKinematicsCB(staubli_tx60::InvKinematics::Request  &req,
      staubli_tx60::InvKinematics::Response &res ) 
{
   std::vector<double> targetPos, j;
   targetPos.push_back( (double) req.x  );
   targetPos.push_back( (double) req.y  );
   targetPos.push_back( (double) req.z  );
   targetPos.push_back( (double) req.rx );
   targetPos.push_back( (double) req.ry );
   targetPos.push_back( (double) req.rz );
   if( invKinematics2( targetPos, j ) )
   {
      res.j = j;
      ROS_INFO("Robot's inverse kinematics sent." );
      return true;
   } 
   else {
      ROS_ERROR("Cannot get inverse kinematics.");
      return false;
   }
}


bool ServicesManager::invKinematics2( std::vector<double> &targetPos,
      std::vector<double> & j) 
{
  std::vector<double> curJoints;
  j.clear();
  if(staubli.GetRobotJoints( curJoints ))
  {
    if(staubli.InverseKinematics( targetPos, curJoints, j ))
    {
      if(fabs(j[0])+fabs(j[1])+fabs(j[2])+
      fabs(j[3])+fabs(j[4])+fabs(j[5]) < ERROR_EPSILON )
      {
        ROS_ERROR("Inv kinematics returned all zeros, PROBABLY error.");
        return false;
      }
      return true;
    }
  }
  return false;
}
