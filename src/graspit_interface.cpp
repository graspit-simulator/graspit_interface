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


}
