#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_ 

//GraspIt! includes
#include <graspit_source/include/plugin.h>

#include <ros/ros.h>

#include <graspit_interface/Robot.h>
#include <graspit_interface/GetRobot.h>

namespace GraspitInterface
{

class GraspitInterface : public Plugin
{

private:
  ros::NodeHandle *nh;

  // Service declarations
  ros::ServiceServer getRobot_srv;

  // Service callbacks
  bool getRobotCB(graspit_interface::GetRobot::Request &request,
                  graspit_interface::GetRobot::Response &response);


public: 
  GraspitInterface(){};
  ~GraspitInterface(){};

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

};

}


#endif
