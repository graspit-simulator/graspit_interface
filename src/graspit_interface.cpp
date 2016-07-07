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
    getRobot_srv = nh->advertiseService("getBody", &GraspitInterface::getBodyCB, this);
    getRobot_srv = nh->advertiseService("getRobots", &GraspitInterface::getRobotsCB, this);
    getRobot_srv = nh->advertiseService("getGraspableBodies", &GraspitInterface::getGraspableBodiesCB, this);
    getRobot_srv = nh->advertiseService("getBodies", &GraspitInterface::getBodiesCB, this);
    getRobot_srv = nh->advertiseService("setRobotPose", &GraspitInterface::setRobotPoseCB, this);
    getRobot_srv = nh->advertiseService("setBodyPose", &GraspitInterface::setBodyPoseCB, this);
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

        geometry_msgs::Pose pose = geometry_msgs::Pose();

        pose.position.x = t.translation().x() / 1000.0;
        pose.position.y = t.translation().y() / 1000.0;;
        pose.position.z = t.translation().z() / 1000.0;;
        pose.orientation.w = t.rotation().w;
        pose.orientation.x = t.rotation().x;
        pose.orientation.y = t.rotation().y;
        pose.orientation.z = t.rotation().z;

        response.robot.pose = pose;

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

bool GraspitInterface::getGraspableBodyCB(graspit_interface::GetGraspableBody::Request &request,
                graspit_interface::GetGraspableBody::Response &response)
{
    if (graspitCore->getWorld()->getNumGB() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {
        GraspableBody *b = graspitCore->getWorld()->getGB(request.id);
        transf t = b->getTran();

        geometry_msgs::Pose pose = geometry_msgs::Pose();

        pose.position.x = t.translation().x() / 1000.0;
        pose.position.y = t.translation().y() / 1000.0;;
        pose.position.z = t.translation().z() / 1000.0;;
        pose.orientation.w = t.rotation().w;
        pose.orientation.x = t.rotation().x;
        pose.orientation.y = t.rotation().y;
        pose.orientation.z = t.rotation().z;

        response.graspable_body.pose = pose;
        return true;
    }
    return true;
}

bool GraspitInterface::getBodyCB(graspit_interface::GetBody::Request &request,
                graspit_interface::GetBody::Response &response)
{
    if (graspitCore->getWorld()->getNumBodies() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {
        Body *b = graspitCore->getWorld()->getBody(request.id);
        transf t = b->getTran();

        geometry_msgs::Pose pose = geometry_msgs::Pose();

        pose.position.x = t.translation().x() / 1000.0;
        pose.position.y = t.translation().y() / 1000.0;;
        pose.position.z = t.translation().z() / 1000.0;;
        pose.orientation.w = t.rotation().w;
        pose.orientation.x = t.rotation().x;
        pose.orientation.y = t.rotation().y;
        pose.orientation.z = t.rotation().z;

        response.body.pose = pose;

        return true;
    }
    return true;
}

bool GraspitInterface::getRobotsCB(graspit_interface::GetRobots::Request &request,
                graspit_interface::GetRobots::Response &response)
{
    for (int i=0; i < graspitCore->getWorld()->getNumRobots(); i++)
    {
        response.ids.push_back(i);
    }
    return true;
}

bool GraspitInterface::getGraspableBodiesCB(graspit_interface::GetGraspableBodies::Request &request,
                graspit_interface::GetGraspableBodies::Response &response)
{
    for (int i=0; i < graspitCore->getWorld()->getNumGB(); i++)
    {
        response.ids.push_back(i);
    }
    return true;
}

bool GraspitInterface::getBodiesCB(graspit_interface::GetBodies::Request &request,
                graspit_interface::GetBodies::Response &response)
{
    for (int i=0; i < graspitCore->getWorld()->getNumBodies(); i++)
    {
        response.ids.push_back(i);
    }
    return true;
}

bool GraspitInterface::setRobotPoseCB(graspit_interface::SetRobotPose::Request &request,
                graspit_interface::SetRobotPose::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {
        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getRobot(request.id)->setTran(newTransform);
        return true;
    }
}

bool GraspitInterface::setGraspableBodyPoseCB(graspit_interface::SetGraspableBodyPose::Request &request,
                graspit_interface::SetGraspableBodyPose::Response &response)
{
    if (graspitCore->getWorld()->getNumGB() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {

        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getGB(request.id)->setTran(newTransform);
        return true;
    }
}

bool GraspitInterface::setBodyPoseCB(graspit_interface::SetBodyPose::Request &request,
                graspit_interface::SetBodyPose::Response &response)
{
    if (graspitCore->getWorld()->getNumBodies() < request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else {

        vec3 newTranslation(request.pose.position.x * 1000.0,
                            request.pose.position.y * 1000.0,
                            request.pose.position.z * 1000.0);

        Quaternion newRotation(request.pose.orientation.w,
                               request.pose.orientation.x,
                               request.pose.orientation.y,
                               request.pose.orientation.z);

        transf newTransform(newRotation, newTranslation);

        graspitCore->getWorld()->getBody(request.id)->setTran(newTransform);
        return true;
    }
}

bool GraspitInterface::getDynamicsCB(graspit_interface::GetDynamics::Request &request,
                graspit_interface::GetDynamics::Response &response)
{
    response.dynamicsEnabled = graspitCore->getWorld()->dynamicsAreOn();
    return true;
}

bool GraspitInterface::setDynamicsCB(graspit_interface::SetDynamics::Request &request,
                graspit_interface::SetDynamics::Response &response)
{
    if(request.enableDynamics && (!graspitCore->getWorld()->dynamicsAreOn()))
    {
        graspitCore->getWorld()->turnOnDynamics();
    }
    else if((!request.enableDynamics) && graspitCore->getWorld()->dynamicsAreOn()){
        graspitCore->getWorld()->turnOffDynamics();
    }
    return true;
}


}
