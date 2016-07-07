#include "graspit_interface.h"

#include "graspit_source/include/graspitCore.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/world.h"


namespace GraspitInterface
{

int GraspitInterface::init(int argc, char** argv)
{
    ros::init(argc, argv, "graspit_interface_node");

    nh = new ros::NodeHandle("");

    getRobot_srv = nh->advertiseService("getRobot", &GraspitInterface::getRobotCB, this);
    getRobot_srv = nh->advertiseService("getGraspableBody", &GraspitInterface::getGraspableBodyCB, this);
    getRobot_srv = nh->advertiseService("getObstacle", &GraspitInterface::getObstacleCB, this);
    getRobot_srv = nh->advertiseService("getRobots", &GraspitInterface::getRobotsCB, this);
    getRobot_srv = nh->advertiseService("getGraspableBodies", &GraspitInterface::getGraspableBodiesCB, this);
    getRobot_srv = nh->advertiseService("getObstacles", &GraspitInterface::getObstaclesCB, this);
    getRobot_srv = nh->advertiseService("setRobotPose", &GraspitInterface::setRobotPoseCB, this);
    getRobot_srv = nh->advertiseService("setObstaclePose", &GraspitInterface::setObstaclePoseCB, this);
    getRobot_srv = nh->advertiseService("setGraspableBodyPose", &GraspitInterface::setGraspableBodyPoseCB, this);
    getRobot_srv = nh->advertiseService("getDynamics", &GraspitInterface::getDynamicsCB, this);
    getRobot_srv = nh->advertiseService("setDynamics", &GraspitInterface::setDynamicsCB, this);

    ROS_INFO("GraspIt interface successfully initialized!");

    return 0;
}

int GraspitInterface::mainLoop()
{
    ros::spinOnce();
    return 0;
} 

bool GraspitInterface::getRobotCB(graspit_interface::GetRobot::Request &request,
                                  graspit_interface::GetRobot::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {
        Robot *r = graspitCore->getWorld()->getRobot(request.id);
        transf t = r->getTran();

        geometry_msgs::Pose robot_pose = geometry_msgs::Pose();

        robot_pose.position.x = t.translation().x() / 1000.0;
        robot_pose.position.y = t.translation().y() / 1000.0;;
        robot_pose.position.z = t.translation().z() / 1000.0;;
        robot_pose.orientation.w = t.rotation().w;
        robot_pose.orientation.x = t.rotation().x;
        robot_pose.orientation.y = t.rotation().y;
        robot_pose.orientation.z = t.rotation().z;

        response.robot.pose = robot_pose;

        for (int i=0; i < r->getNumJoints(); i++) {
            sensor_msgs::JointState robot_joint_state = sensor_msgs::JointState();
            response.robot.joints.push_back(robot_joint_state);
        }

        for (int i=0; i< r->getNumDOF(); i++) {
            response.robot.dofs.push_back(r->getDOF(i)->getVal());
        }

        return true;
    }
    return true;
}

bool GraspitInterface::getRobotCB(graspit_interface::GetRobot::Request &request,
                graspit_interface::GetRobot::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getGraspableBodyCB(graspit_interface::GetGraspableBody::Request &request,
                graspit_interface::GetGraspableBody::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getObstacleCB(graspit_interface::GetObstacle::Request &request,
                graspit_interface::GetObstacle::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getRobotsCB(graspit_interface::GetRobot::Request &request,
                graspit_interface::GetRobot::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getGraspableBodiesCB(graspit_interface::GetGraspableBodies::Request &request,
                graspit_interface::GetGraspableBodies::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getObstaclesCB(graspit_interface::GetObstacles::Request &request,
                graspit_interface::GetObstacles::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getRobotPoseCB(graspit_interface::GetRobot::Request &request,
                graspit_interface::GetRobotPose::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getGraspableBodyPoseCB(graspit_interface::GetGraspableBody::Request &request,
                graspit_interface::GetGraspableBodyPose::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getObstaclePoseCB(graspit_interface::GetObstaclePose::Request &request,
                graspit_interface::GetObstacle::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::getDynamics(graspit_interface::GetDynamics::Request &request,
                graspit_interface::GetDynamics::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}

bool GraspitInterface::setDynamics(graspit_interface::SetDynamics::Request &request,
                graspit_interface::SetDynamics::Response &response)
{
    ROS_ERROR("NOT IMPLEMENTED!!");
}


}
