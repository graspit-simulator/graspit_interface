#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_ 

//GraspIt! includes
#include <graspit_source/include/plugin.h>

#include <ros/ros.h>

#include <graspit_interface/Robot.h>
#include <graspit_interface/GetRobot.h>
#include <graspit_interface/GetGraspableBody.h>
#include <graspit_interface/GetBody.h>
#include <graspit_interface/GetRobots.h>
#include <graspit_interface/GetGraspableBodies.h>
#include <graspit_interface/GetBodies.h>
#include <graspit_interface/SetDynamics.h>
#include <graspit_interface/GetDynamics.h>
#include <graspit_interface/SetGraspableBodyPose.h>
#include <graspit_interface/SetBodyPose.h>
#include <graspit_interface/SetRobotPose.h>
#include <graspit_interface/AutoGrasp.h>
#include <graspit_interface/AutoOpen.h>
#include <graspit_interface/SetRobotDesiredDOF.h>
#include <graspit_interface/ImportRobot.h>
#include <graspit_interface/ImportGraspableBody.h>
#include <graspit_interface/ImportObstacle.h>
#include <graspit_interface/LoadWorld.h>
#include <graspit_interface/ClearWorld.h>
#include <graspit_interface/SaveWorld.h>
#include <graspit_interface/SaveImage.h>
#include <graspit_interface/ToggleAllCollisions.h>

namespace GraspitInterface
{

class GraspitInterface : public Plugin
{

private:
  ros::NodeHandle *nh;

  // Service declarations
  ros::ServiceServer getRobot_srv;
  ros::ServiceServer getGraspableBody_srv;
  ros::ServiceServer getBody_srv;

  ros::ServiceServer getRobots_srv;
  ros::ServiceServer getGraspableBodies_srv;
  ros::ServiceServer getBodies_srv;

  ros::ServiceServer setRobotPose_srv;
  ros::ServiceServer setBodyPose_srv;
  ros::ServiceServer setGraspableBodyPose_srv;

  ros::ServiceServer getDynamics_srv;
  ros::ServiceServer setDynamics_srv;

  ros::ServiceServer autoGrasp_srv;
  ros::ServiceServer autoOpen_srv;
  ros::ServiceServer setRobotDesiredDOF_srv;

  ros::ServiceServer importRobot_srv;
  ros::ServiceServer importObstacle_srv;
  ros::ServiceServer importGraspableBody_srv;

  ros::ServiceServer clearWorld_srv;
  ros::ServiceServer loadWorld_srv;
  ros::ServiceServer saveWorld_srv;

  ros::ServiceServer saveImage_srv;
  ros::ServiceServer toggleAllCollisions_srv;

  // Service callbacks
  bool getRobotCB(graspit_interface::GetRobot::Request &request,
                  graspit_interface::GetRobot::Response &response);

  bool getGraspableBodyCB(graspit_interface::GetGraspableBody::Request &request,
                          graspit_interface::GetGraspableBody::Response &response);

  bool getBodyCB(graspit_interface::GetBody::Request &request,
                     graspit_interface::GetBody::Response &response);

  bool getRobotsCB(graspit_interface::GetRobots::Request &request,
                   graspit_interface::GetRobots::Response &response);

  bool getGraspableBodiesCB(graspit_interface::GetGraspableBodies::Request &request,
                            graspit_interface::GetGraspableBodies::Response &response);

  bool getBodiesCB(graspit_interface::GetBodies::Request &request,
                      graspit_interface::GetBodies::Response &response);

  bool setRobotPoseCB(graspit_interface::SetRobotPose::Request &request,
                      graspit_interface::SetRobotPose::Response &response);

  bool setGraspableBodyPoseCB(graspit_interface::SetGraspableBodyPose::Request &request,
                              graspit_interface::SetGraspableBodyPose::Response &response);

  bool setBodyPoseCB(graspit_interface::SetBodyPose::Request &request,
                         graspit_interface::SetBodyPose::Response &response);

  bool getDynamicsCB(graspit_interface::GetDynamics::Request &request,
                     graspit_interface::GetDynamics::Response &response);

  bool setDynamicsCB(graspit_interface::SetDynamics::Request &request,
                     graspit_interface::SetDynamics::Response &response);

  bool autoGraspCB(graspit_interface::AutoGrasp::Request &request,
                         graspit_interface::AutoGrasp::Response &response);

  bool autoOpenCB(graspit_interface::AutoOpen::Request &request,
                     graspit_interface::AutoOpen::Response &response);

  bool setRobotDesiredDOFCB(graspit_interface::SetRobotDesiredDOF::Request &request,
                     graspit_interface::SetRobotDesiredDOF::Response &response);

  bool importRobotCB(graspit_interface::ImportRobot::Request &request,
                         graspit_interface::ImportRobot::Response &response);

  bool importObstacleCB(graspit_interface::ImportObstacle::Request &request,
                     graspit_interface::ImportObstacle::Response &response);

  bool importGraspableBodyCB(graspit_interface::ImportGraspableBody::Request &request,
                     graspit_interface::ImportGraspableBody::Response &response);

  bool loadWorldCB(graspit_interface::LoadWorld::Request &request,
                         graspit_interface::LoadWorld::Response &response);

  bool saveWorldCB(graspit_interface::SaveWorld::Request &request,
                     graspit_interface::SaveWorld::Response &response);

  bool clearWorldCB(graspit_interface::ClearWorld::Request &request,
                     graspit_interface::ClearWorld::Response &response);

  bool saveImageCB(graspit_interface::SaveImage::Request &request,
                     graspit_interface::SaveImage::Response &response);

  bool toggleAllCollisionsCB(graspit_interface::ToggleAllCollisions::Request &request,
                     graspit_interface::ToggleAllCollisions::Response &response);

public: 
  GraspitInterface(){}
  ~GraspitInterface(){}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

};

}


#endif
