#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_ 

//GraspIt! includes
#include <graspit_source/include/plugin.h>

#include <ros/ros.h>

#include <graspit_interface/Robot.h>
#include <graspit_interface/GetRobot.h>
#include <graspit_interface/GetGraspableBody.h>
#include <graspit_interface/GetObstacle.h>
#include <graspit_interface/GetRobots.h>
#include <graspit_interface/GetGraspableBodies.h>
#include <graspit_interface/GetObstacles.h>
#include <graspit_interface/SetDynamics.h>
#include <graspit_interface/GetDynamics.h>
#include <graspit_interface/SetGraspableBodyPose.h>
#include <graspit_interface/SetObstaclePose.h>
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
  ros::ServiceServer getObstacle_srv;

  ros::ServiceServer getRobots_srv;
  ros::ServiceServer getGraspableBodies_srv;
  ros::ServiceServer getObstacles_srv;

  ros::ServiceServer setRobotPose_srv;
  ros::ServiceServer setObstaclePose_srv;
  ros::ServiceServer setGraspableBodyPose_srv;

  ros::ServiceServer getDynamics_srv;
  ros::ServiceServer setDynamics_srv;

  // Service callbacks
  bool getRobotCB(graspit_interface::GetRobot::Request &request,
                  graspit_interface::GetRobot::Response &response);

  bool getGraspableBodyCB(graspit_interface::GetGraspableBody::Request &request,
                          graspit_interface::GetGraspableBody::Response &response);

  bool getObstacleCB(graspit_interface::GetObstacle::Request &request,
                     graspit_interface::GetObstacle::Response &response);

  bool getRobotsCB(graspit_interface::GetRobots::Request &request,
                   graspit_interface::GetRobots::Response &response);

  bool getGraspableBodiesCB(graspit_interface::GetGraspableBodies::Request &request,
                            graspit_interface::GetGraspableBodies::Response &response);

  bool getObstaclesCB(graspit_interface::GetObstacles::Request &request,
                      graspit_interface::GetObstacles::Response &response);

  bool setRobotPoseCB(graspit_interface::SetRobotPose::Request &request,
                      graspit_interface::SetRobotPose::Response &response);

  bool setGraspableBodyPoseCB(graspit_interface::SetGraspableBodyPose::Request &request,
                              graspit_interface::SetGraspableBodyPose::Response &response);

  bool setObstaclePoseCB(graspit_interface::SetObstaclePose::Request &request,
                         graspit_interface::SetObstaclePose::Response &response);

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
