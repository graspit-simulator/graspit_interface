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
    getGraspableBody_srv = nh->advertiseService("getGraspableBody", &GraspitInterface::getGraspableBodyCB, this);
    getBody_srv = nh->advertiseService("getBody", &GraspitInterface::getBodyCB, this);
    getRobots_srv = nh->advertiseService("getRobots", &GraspitInterface::getRobotsCB, this);
    getGraspableBodies_srv = nh->advertiseService("getGraspableBodies", &GraspitInterface::getGraspableBodiesCB, this);
    getBodies_srv = nh->advertiseService("getBodies", &GraspitInterface::getBodiesCB, this);
    setRobotPose_srv = nh->advertiseService("setRobotPose", &GraspitInterface::setRobotPoseCB, this);
    setBodyPose_srv = nh->advertiseService("setBodyPose", &GraspitInterface::setBodyPoseCB, this);
    setGraspableBodyPose_srv = nh->advertiseService("setGraspableBodyPose", &GraspitInterface::setGraspableBodyPoseCB, this);
    getDynamics_srv = nh->advertiseService("getDynamics", &GraspitInterface::getDynamicsCB, this);
    setDynamics_srv = nh->advertiseService("setDynamics", &GraspitInterface::setDynamicsCB, this);
    autoGrasp_srv = nh->advertiseService("autoGrasp", &GraspitInterface::autoGraspCB, this);
    autoOpen_srv = nh->advertiseService("autoOpen", &GraspitInterface::autoOpenCB, this);
    setRobotDesiredDOF_srv = nh->advertiseService("setRobotDesiredDOF", &GraspitInterface::setRobotDesiredDOFCB, this);

    importRobot_srv = nh->advertiseService("importRobot", &GraspitInterface::importRobotCB, this);
    importObstacle_srv = nh->advertiseService("importObstacle", &GraspitInterface::importObstacleCB, this);
    importGraspableBody_srv = nh->advertiseService("importGraspableBody", &GraspitInterface::importGraspableBodyCB, this);


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
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
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
    if (graspitCore->getWorld()->getNumGB() <= request.id) {
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
    if (graspitCore->getWorld()->getNumBodies() <= request.id) {
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
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
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
    if (graspitCore->getWorld()->getNumGB() <= request.id) {
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
    if (graspitCore->getWorld()->getNumBodies() <= request.id) {
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
        ROS_INFO("Turning Dynamics On");
    }
    else if((!request.enableDynamics) && graspitCore->getWorld()->dynamicsAreOn()){
        graspitCore->getWorld()->turnOffDynamics();
        ROS_INFO("Turning Dynamics Off");
    }
    return true;
}

bool GraspitInterface::autoGraspCB(graspit_interface::AutoGrasp::Request &request,
                       graspit_interface::AutoGrasp::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
    else{
        graspitCore->getWorld()->getHand(request.id)->autoGrasp(true, 1.0, false);
    }
    return true;
}

bool GraspitInterface::autoOpenCB(graspit_interface::AutoOpen::Request &request,
                   graspit_interface::AutoOpen::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
    else{
        graspitCore->getWorld()->getHand(request.id)->autoGrasp(true, -1.0, false);
    }
    return true;
}

bool GraspitInterface::setRobotDesiredDOFCB(graspit_interface::SetRobotDesiredDOF::Request &request,
                   graspit_interface::SetRobotDesiredDOF::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
    else{
        if(graspitCore->getWorld()->dynamicsAreOn())
        {
            graspitCore->getWorld()->getHand(request.id)->setDesiredDOFVals(request.dofs.data());
        }
        else
        {
            graspitCore->getWorld()->getHand(request.id)->forceDOFVals(request.dofs.data());
        }
    }
    return true;
}

bool GraspitInterface::importRobotCB(graspit_interface::ImportRobot::Request &request,
                       graspit_interface::ImportRobot::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/robots/") +
            QString(request.filename.data()) +
            QString("/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s",filename.toStdString().c_str());

    Robot * r = graspitCore->getWorld()->importRobot(filename);
    if(r == NULL){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::importObstacleCB(graspit_interface::ImportObstacle::Request &request,
                   graspit_interface::ImportObstacle::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/obstacles/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s", filename.toStdString().c_str());

    Body * b = graspitCore->getWorld()->importBody(QString("Body"),filename);
    if(b == NULL){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::importGraspableBodyCB(graspit_interface::ImportGraspableBody::Request &request,
                   graspit_interface::ImportGraspableBody::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/models/objects/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading %s",filename.toStdString().c_str());
    Body * b = graspitCore->getWorld()->importBody(QString("GraspableBody"),filename);
    if(b == NULL){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}


}
