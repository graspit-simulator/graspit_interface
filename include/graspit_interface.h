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

public: 
  GraspitInterface(){}
  ~GraspitInterface(){}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

};

}


#endif
