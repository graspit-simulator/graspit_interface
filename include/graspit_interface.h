#ifndef _GRASPIT_INTERFACE_H_
#define _GRASPIT_INTERFACE_H_ 

//ROS includes
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#endif

//GraspIt! includes
#include <graspit/plugin.h>
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/simAnnPlanner.h>

//Message includes
#include <graspit_interface/SearchSpace.h>

// Service includes
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
#include <graspit_interface/ForceRobotDOF.h>
#include <graspit_interface/MoveDOFToContacts.h>
#include <graspit_interface/SetRobotDesiredDOF.h>
#include <graspit_interface/ImportRobot.h>
#include <graspit_interface/ImportGraspableBody.h>
#include <graspit_interface/ImportObstacle.h>
#include <graspit_interface/LoadWorld.h>
#include <graspit_interface/ClearWorld.h>
#include <graspit_interface/SaveWorld.h>
#include <graspit_interface/SaveImage.h>
#include <graspit_interface/ToggleAllCollisions.h>
#include <graspit_interface/ComputeQuality.h>
#include <graspit_interface/ComputeEnergy.h>
#include <graspit_interface/ApproachToContact.h>
#include <graspit_interface/FindInitialContact.h>
#include <graspit_interface/DynamicAutoGraspComplete.h>

// ActionServer includes
#include <graspit_interface/PlanGraspsAction.h>

namespace GraspitInterface
{

class GraspitInterface : public QObject, public Plugin
{

    Q_OBJECT

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
  ros::ServiceServer forceRobotDOF_srv;
  ros::ServiceServer moveDOFToContacts_srv;
  ros::ServiceServer setRobotDesiredDOF_srv;

  ros::ServiceServer importRobot_srv;
  ros::ServiceServer importObstacle_srv;
  ros::ServiceServer importGraspableBody_srv;

  ros::ServiceServer clearWorld_srv;
  ros::ServiceServer loadWorld_srv;
  ros::ServiceServer saveWorld_srv;

  ros::ServiceServer saveImage_srv;
  ros::ServiceServer toggleAllCollisions_srv;

  ros::ServiceServer computeQuality_srv;
  ros::ServiceServer computeEnergy_srv;

  ros::ServiceServer approachToContact_srv;
  ros::ServiceServer findInitialContact_srv;

  ros::ServiceServer dynamicAutoGraspComplete_srv;

  // ActionServer declarations
  actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction> *plan_grasps_as;
  graspit_interface::PlanGraspsFeedback feedback_;
  graspit_interface::PlanGraspsResult result_;
  graspit_interface::PlanGraspsGoal goal;

  GraspPlanningState *mHandObjectState;
  SimAnnPlanner *mPlanner;

  bool firstTimeInMainLoop;

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

  bool forceRobotDOFCB(graspit_interface::ForceRobotDOF::Request &request,
                       graspit_interface::ForceRobotDOF::Response &response);

  bool moveDOFToContactsCB(graspit_interface::MoveDOFToContacts::Request &request,
                       graspit_interface::MoveDOFToContacts::Response &response);

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

  bool computeQualityCB(graspit_interface::ComputeQuality::Request &request,
                         graspit_interface::ComputeQuality::Response &response);

  bool computeEnergyCB(graspit_interface::ComputeEnergy::Request &request,
                         graspit_interface::ComputeEnergy::Response &response);

  bool approachToContactCB(graspit_interface::ApproachToContact::Request &request,
                           graspit_interface::ApproachToContact::Response &response);

  bool findInitialContactCB(graspit_interface::FindInitialContact::Request &request,
                            graspit_interface::FindInitialContact::Response &response);

  bool dynamicAutoGraspCompleteCB(graspit_interface::DynamicAutoGraspComplete::Request &request,
                                  graspit_interface::DynamicAutoGraspComplete::Response &response);

  //ActionServer callbacks
  void PlanGraspsCB(const graspit_interface::PlanGraspsGoalConstPtr &goal);

  void graspPlanningStateToROSMsg(const GraspPlanningState *gps, graspit_interface::Grasp &g, Hand *mHand);


  // Convenience functions for converting between pose types:
    inline geometry_msgs::Pose transfToRosMsg(transf pose) {
        geometry_msgs::Pose ret;
        ret.position.x = pose.translation().x() / 1000.0;
        ret.position.y = pose.translation().y() / 1000.0;;
        ret.position.z = pose.translation().z() / 1000.0;;
        ret.orientation.w = pose.rotation().w();
        ret.orientation.x = pose.rotation().x();
        ret.orientation.y = pose.rotation().y();
        ret.orientation.z = pose.rotation().z();
        return ret;
    }
    inline transf rosMsgToTransf(geometry_msgs::Pose pose) {
        Quaternion q(pose.orientation.w, pose.orientation.x, 
                pose.orientation.y, pose.orientation.z);
        vec3 p(pose.position.x * 1000.0, pose.position.y * 1000.0, 
                pose.position.z * 1000.0);
        transf ret(q, p);
        return ret;
}

public: 
  GraspitInterface(){}
  ~GraspitInterface(){}

  virtual int init(int argc, char **argv);

  virtual int mainLoop();

public Q_SLOTS:

    void runPlannerInMainThread();
    void processPlannerResultsInMainThread();
    void buildFeedbackInMainThread();

Q_SIGNALS:

    void emitRunPlannerInMainThread();
    void emitProcessPlannerResultsInMainThread();
    void emitBuildFeedbackInMainThread();


};

}


#endif
