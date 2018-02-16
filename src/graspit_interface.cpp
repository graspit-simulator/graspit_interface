#include "graspit_interface.h"

#include <string>
#include <graspit/graspitCore.h>
#include <graspit/robot.h>
#include <graspit/world.h>
#include <graspit/ivmgr.h>

#include <graspit/quality/quality.h>
#include <graspit/quality/qualVolume.h>
#include <graspit/quality/qualEpsilon.h>
#include <graspit/grasp.h>
#include <graspit/EGPlanner/searchState.h>
#include <graspit/EGPlanner/egPlanner.h>
#include <graspit/EGPlanner/simAnnPlanner.h>
#include <graspit/EGPlanner/simAnnParams.h>
#include <graspit/EGPlanner/guidedPlanner.h>
#include <graspit/EGPlanner/energy/searchEnergyFactory.h>
#include <graspit/EGPlanner/energy/searchEnergy.h>
#include <graspit/bodySensor.h>
#include <graspit/cmdline/cmdline.h>

namespace GraspitInterface
{

int GraspitInterface::init(int argc, char** argv)
{
    ros::init(argc, argv, "graspit_interface_node");


    const std::string node_name_help =
            "print Ros Node name\n";

    cmdline::parser* parser = new cmdline::parser();

    parser->add<std::string>("node_name", 'n', node_name_help,  false);
    parser->parse(argc, argv);

    std::string node_name = "";

    if(parser->exist("node_name")){
        node_name = parser->get<std::string>("node_name");
    }


    nh = new ros::NodeHandle(node_name);

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

    forceRobotDOF_srv = nh->advertiseService("forceRobotDof", &GraspitInterface::forceRobotDOFCB, this);
    moveDOFToContacts_srv = nh->advertiseService("moveDOFToContacts", &GraspitInterface::moveDOFToContactsCB, this);
    setRobotDesiredDOF_srv = nh->advertiseService("setRobotDesiredDOF", &GraspitInterface::setRobotDesiredDOFCB, this);

    importRobot_srv = nh->advertiseService("importRobot", &GraspitInterface::importRobotCB, this);
    importObstacle_srv = nh->advertiseService("importObstacle", &GraspitInterface::importObstacleCB, this);
    importGraspableBody_srv = nh->advertiseService("importGraspableBody", &GraspitInterface::importGraspableBodyCB, this);

    clearWorld_srv = nh->advertiseService("clearWorld", &GraspitInterface::clearWorldCB, this);
    loadWorld_srv = nh->advertiseService("loadWorld", &GraspitInterface::loadWorldCB, this);
    saveWorld_srv = nh->advertiseService("saveWorld", &GraspitInterface::saveWorldCB, this);

    saveImage_srv = nh->advertiseService("saveImage", &GraspitInterface::saveImageCB, this);
    toggleAllCollisions_srv = nh->advertiseService("toggleAllCollisions", &GraspitInterface::toggleAllCollisionsCB, this);

    computeQuality_srv = nh->advertiseService("computeQuality", &GraspitInterface::computeQualityCB, this);
    computeEnergy_srv = nh->advertiseService("computeEnergy", &GraspitInterface::computeEnergyCB, this);

    approachToContact_srv = nh->advertiseService("approachToContact", &GraspitInterface::approachToContactCB, this);
    findInitialContact_srv = nh->advertiseService("findInitialContact", &GraspitInterface::findInitialContactCB, this);
    dynamicAutoGraspComplete_srv= nh->advertiseService("dynamicAutoGraspComplete", &GraspitInterface::dynamicAutoGraspCompleteCB, this);

    plan_grasps_as = new actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction>(*nh, "planGrasps",
                                                                                            boost::bind(&GraspitInterface::PlanGraspsCB, this, _1), false);
    plan_grasps_as->start();

    firstTimeInMainLoop = true;

    mPlanner = NULL;
    mHandObjectState = NULL;

    ROS_INFO("GraspIt interface successfully initialized!");

    return 0;
}

int GraspitInterface::mainLoop()
{
    if(firstTimeInMainLoop)
    {
        //Planner Must be started by mainthread, so it cannot be
        //Started inside the callback for the action server.  I need to connect these here
        //So that when the signal is emitted, the slot function is executed by the correct thread.
        QObject::connect(this, SIGNAL(emitRunPlannerInMainThread()), this, SLOT(runPlannerInMainThread()), Qt::BlockingQueuedConnection);
        QObject::connect(this, SIGNAL(emitProcessPlannerResultsInMainThread()), this, SLOT(processPlannerResultsInMainThread()), Qt::BlockingQueuedConnection);
        QObject::connect(this, SIGNAL(emitBuildFeedbackInMainThread()), this, SLOT(buildFeedbackInMainThread()), Qt::BlockingQueuedConnection);
        firstTimeInMainLoop = false;
        ROS_INFO("Planner Signal/Slots connected");
    }

    if (ros::ok()){
      ros::spinOnce();
    }
    else{
        graspitCore->exitMainLoop();
    }
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
        pose.orientation.w = t.rotation().w();
        pose.orientation.x = t.rotation().x();
        pose.orientation.y = t.rotation().y();
	pose.orientation.z = t.rotation().z();

	response.robot.pose = pose;
	
	// Info for all contacts with this robot:
	std::list<Contact*> contacts = r->getContacts();
	for (std::list<Contact*>::iterator it = contacts.begin();
	     it != contacts.end(); it++) {
	  graspit_interface::Contact c;
	  c.body1 = (*it)->getBody1()->getName().toStdString();
	  c.body2 = (*it)->getBody2()->getName().toStdString();
            
          transf contactInWorldFrame = (*it)->getBody1Tran() % (*it)->getFrame();
          c.ps.header.frame_id = "world";
          c.ps.pose.position.x = contactInWorldFrame.translation().x() / 1000.0;
          c.ps.pose.position.y = contactInWorldFrame.translation().y() / 1000.0;;
          c.ps.pose.position.z = contactInWorldFrame.translation().z() / 1000.0;;
          c.ps.pose.orientation.w = contactInWorldFrame.rotation().w();
          c.ps.pose.orientation.x = contactInWorldFrame.rotation().x();
          c.ps.pose.orientation.y = contactInWorldFrame.rotation().y();
          c.ps.pose.orientation.z = contactInWorldFrame.rotation().z();
	  c.cof = (*it)->getCof();
	  response.robot.contacts.push_back(c);
        }

        double *jointVals = new double[r->getNumJoints()];
        r->getJointValues(jointVals);

        sensor_msgs::JointState robot_joint_state = sensor_msgs::JointState();
        for (int i=0; i < r->getNumJoints(); i++) {

            std::string joint_name;
            std::ostringstream convert;
            convert << i;
            joint_name =convert.str();

            robot_joint_state.name.push_back(joint_name);
            robot_joint_state.position.push_back(jointVals[i]);
            robot_joint_state.velocity.push_back(0);
            robot_joint_state.effort.push_back(0);

        }
        response.robot.joints.push_back(robot_joint_state);

        for (int i=0; i< r->getNumDOF(); i++) {
            response.robot.dofs.push_back(r->getDOF(i)->getVal());
        }

        for (int i=0; i< r->getNumSensors(); i++) {

            BodySensor *s = r->getSensor(i);
            transf t = s->getSensorTran();

            geometry_msgs::PoseStamped poseStamped = geometry_msgs::PoseStamped();

            poseStamped.header.frame_id = "world";
            poseStamped.pose.position.x = t.translation().x() / 1000.0;
            poseStamped.pose.position.y = t.translation().y() / 1000.0;;
            poseStamped.pose.position.z = t.translation().z() / 1000.0;;
            poseStamped.pose.orientation.w = t.rotation().w();
            poseStamped.pose.orientation.x = t.rotation().x();
            poseStamped.pose.orientation.y = t.rotation().y();
            poseStamped.pose.orientation.z = t.rotation().z();

            response.robot.tactile.sensor_poses.push_back(poseStamped);
            response.robot.tactile.sensor_forces.push_back(s->getNormalForce());
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
        pose.orientation.w = t.rotation().w();
        pose.orientation.x = t.rotation().x();
        pose.orientation.y = t.rotation().y();
        pose.orientation.z = t.rotation().z();

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
        pose.orientation.w = t.rotation().w();
        pose.orientation.x = t.rotation().x();
        pose.orientation.y = t.rotation().y();
        pose.orientation.z = t.rotation().z();

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
        graspitCore->getWorld()->updateGrasps();
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
        graspitCore->getWorld()->updateGrasps();
    }
    return true;
}

bool GraspitInterface::forceRobotDOFCB(graspit_interface::ForceRobotDOF::Request &request,
                                       graspit_interface::ForceRobotDOF::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
        return true;
    } else {
        graspitCore->getWorld()->getHand(request.id)->forceDOFVals(request.dofs.data());
        response.result = response.RESULT_SUCCESS;
        return true;
    }
}

bool GraspitInterface::moveDOFToContactsCB(graspit_interface::MoveDOFToContacts::Request &request,
                     graspit_interface::MoveDOFToContacts::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_ENABLED;
        return true;
    } else {
        graspitCore->getWorld()->getHand(request.id)->moveDOFToContacts(request.dofs.data(), request.desired_steps.data(), request.stopAtContact);
        response.result = response.RESULT_SUCCESS;
        return true;
    }
}

bool GraspitInterface::setRobotDesiredDOFCB(graspit_interface::SetRobotDesiredDOF::Request &request,
                                            graspit_interface::SetRobotDesiredDOF::Response &response)
{
    if (graspitCore->getWorld()->getNumRobots() <= request.id) {
        response.result = response.RESULT_INVALID_ID;
        return true;
    } else if (!graspitCore->getWorld()->dynamicsAreOn()) {
        response.result = response.RESULT_DYNAMICS_MODE_DISABLED;
        return true;
    } else {
        for(int i=0; i < graspitCore->getWorld()->getHand(request.id)->getNumDOF(); i++) {
            graspitCore->getWorld()->getHand(request.id)->getDOF(i)->setDesiredVelocity(request.dof_velocities.data()[i]);
        }
        graspitCore->getWorld()->getHand(request.id)->setDesiredDOFVals(request.dofs.data());
        response.result = response.RESULT_SUCCESS;
        return true;
    }
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
    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    r->setTran(newTransform);
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
        //Now try to load using unaltered filepath from request.
        b = graspitCore->getWorld()->importBody(QString("Body"),QString(request.filename.data()));
        if(b == NULL){
            response.result = response.RESULT_FAILURE;
            return true;
        }
    }

    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    b->setTran(newTransform);
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
    //First try to load from Graspit Directory
    Body * b = graspitCore->getWorld()->importBody(QString("GraspableBody"),filename);
    if(b == NULL){
        //Now try to load using unaltered filepath from request.
        b = graspitCore->getWorld()->importBody(QString("GraspableBody"),QString(request.filename.data()));
        if(b == NULL){
            response.result = response.RESULT_FAILURE;
            return true;
        }
    }

    vec3 newTranslation(request.pose.position.x * 1000.0,
                        request.pose.position.y * 1000.0,
                        request.pose.position.z * 1000.0);

    Quaternion newRotation(request.pose.orientation.w,
                           request.pose.orientation.x,
                           request.pose.orientation.y,
                           request.pose.orientation.z);

    transf newTransform(newRotation, newTranslation);
    b->setTran(newTransform);

    return true;
}


bool GraspitInterface::loadWorldCB(graspit_interface::LoadWorld::Request &request,
                       graspit_interface::LoadWorld::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/worlds/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Loading World: %s",filename.toStdString().c_str());
    int result = graspitCore->getWorld()->load(filename);
    if(result == FAILURE){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::saveWorldCB(graspit_interface::SaveWorld::Request &request,
                   graspit_interface::SaveWorld::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/worlds/") +
            QString(request.filename.data()) +
            QString(".xml");

    ROS_INFO("Saving World: %s",filename.toStdString().c_str());
    int result = graspitCore->getWorld()->save(filename);
    if(result == FAILURE){
        response.result = response.RESULT_FAILURE;
        return true;
    }
    return true;
}

bool GraspitInterface::clearWorldCB(graspit_interface::ClearWorld::Request &request,
                   graspit_interface::ClearWorld::Response &response)
{
    ROS_INFO("Emptying World");
    graspitCore->emptyWorld();

    return true;
}

bool GraspitInterface::saveImageCB(graspit_interface::SaveImage::Request &request,
                   graspit_interface::SaveImage::Response &response)
{
    QString filename = QString(getenv("GRASPIT"))+
            QString("/images/") +
            QString(request.filename.data()) +
            QString(".jpg");

    ROS_INFO("Saving Image: %s",filename.toStdString().c_str());
    graspitCore->getIVmgr()->saveImage(filename);
    return true;
}

bool GraspitInterface::toggleAllCollisionsCB(graspit_interface::ToggleAllCollisions::Request &request,
                   graspit_interface::ToggleAllCollisions::Response &response)
{
    graspitCore->getWorld()->toggleAllCollisions(request.enableCollisions);
    if(request.enableCollisions)
    {
        ROS_INFO("Collision Detection is On, objects cannot interpentrate");
    }
    else
    {
        ROS_INFO("Collision Detection is Off, objects can interpentrate");
    }
    return true;
}

bool GraspitInterface::computeQualityCB(graspit_interface::ComputeQuality::Request &request,
                                         graspit_interface::ComputeQuality::Response &response)
{
    CollisionReport colReport;
    // first test whether the hand is in collision now
    int numCols = graspitCore->getWorld()->getCollisionReport(&colReport);
    // if it is in collision, then there should be no reason to calculate the quality
    if(numCols>0){
        response.result = response.RESULT_COLLISION;
        response.epsilon = -1.0;
        response.volume = -1.0;
        return true;
    }
    Hand *mHand =graspitCore->getWorld()->getHand(request.id);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }

    // if there is no collision, then begin computation

    QualVolume mVolQual( mHand->getGrasp(), ("Volume"),"L1 Norm");
    QualEpsilon mEpsQual( mHand->getGrasp(), ("Epsilon"),"L1 Norm");

    graspitCore->getWorld()->findAllContacts();
    graspitCore->getWorld()->updateGrasps();

    response.epsilon = mEpsQual.evaluate();
    response.volume = mVolQual.evaluate();

    return true;
}

bool GraspitInterface::computeEnergyCB(graspit_interface::ComputeEnergy::Request &request,
                                         graspit_interface::ComputeEnergy::Response &response)
{
    Hand *mHand =graspitCore->getWorld()->getHand(request.handId);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_HAND_ID;
        ROS_INFO("Planning Hand is NULL");
        return true;
    }
    GraspableBody *mObject = graspitCore->getWorld()->getGB(request.graspableBodyId);
    if(mObject == NULL)
    {
        ROS_INFO("Planning Object is NULL");
        response.result = response.RESULT_INVALID_BODY_ID;
        return true;
    }

    graspitCore->getWorld()->findAllContacts();
    graspitCore->getWorld()->updateGrasps();

    std::vector<std::string> energyTypes = SearchEnergyFactory::getInstance()->getAllRegisteredEnergy();
    if(std::find(energyTypes.begin(),energyTypes.end(), request.energyType) == energyTypes.end())
      {
        ROS_INFO_STREAM("Invalid Energy Type " << request.energyType << std::endl);
        response.result = response.RESULT_INVALID_ENERGY_TYPE;
        return true;
      }

    SearchEnergy *se = SearchEnergyFactory::getInstance()->createEnergy(request.energyType);

    bool isLegal;
    double stateEnergy;
    se->analyzeCurrentPosture(mHand, mObject, isLegal, stateEnergy);
    response.isLegal = isLegal;
    response.energy= stateEnergy;

    return true;
}

bool GraspitInterface::approachToContactCB(graspit_interface::ApproachToContact::Request &request,
                                           graspit_interface::ApproachToContact::Response &response)
{
    Hand *mHand =graspitCore->getWorld()->getHand(request.id);
    if (mHand==NULL)
    {
        response.result = response.RESULT_INVALID_ID;
        return true;
    }
     mHand->approachToContact(request.moveDist, request.oneStep);
     return true;
}

bool GraspitInterface::findInitialContactCB(graspit_interface::FindInitialContact::Request &request,
                                            graspit_interface::FindInitialContact::Response &response)
{
     Hand *mHand =graspitCore->getWorld()->getHand(request.id);
     if (mHand==NULL)
     {
         response.result = response.RESULT_INVALID_ID;
         return true;
     }

     mHand->findInitialContact(request.moveDist);
     return true;
}

bool GraspitInterface::dynamicAutoGraspCompleteCB(graspit_interface::DynamicAutoGraspComplete::Request &request,
                                graspit_interface::DynamicAutoGraspComplete::Response &response)
{
     Hand *mHand = graspitCore->getWorld()->getCurrentHand();
     if (mHand==NULL)
     {
         response.result = response.RESULT_INVALID_ID;
         return true;
     }
     response.GraspComplete = mHand->dynamicAutograspComplete();
     return true;
}

void GraspitInterface::PlanGraspsCB(const graspit_interface::PlanGraspsGoalConstPtr &_goal)
{
    goal = *_goal;
    ROS_INFO("About to Call emit runPlannerInMainLoop();");
    emit emitRunPlannerInMainThread();

    ROS_INFO("Waiting For Planner to Finish");
    int last_feedback_step;
    while(mPlanner->isActive())
    {
        if (_goal->feedback_num_steps < 1){
            continue;
        }
        int current_feedback_step = mPlanner->getCurrentStep();
        if (current_feedback_step == mPlanner->getStartingStep()){
            continue;
        }
        if ((current_feedback_step % _goal->feedback_num_steps == 0) && (current_feedback_step != last_feedback_step)){
            ROS_INFO("Curret Planner Step: %d", mPlanner->getCurrentStep());
            ROS_INFO("Curret Num Grasps: %d", mPlanner->getListSize());

            // collect grasps
            emit emitBuildFeedbackInMainThread();
            plan_grasps_as->publishFeedback(feedback_);
            last_feedback_step = current_feedback_step;
        }
    }

    ROS_INFO("About to Call emit emitProcessPlannerResultsInMainThread();");
    emit emitProcessPlannerResultsInMainThread();

    plan_grasps_as->setSucceeded(result_);
    ROS_INFO("Action ServerCB Finished");
}


void GraspitInterface::buildFeedbackInMainThread()
{
    feedback_.current_step = mPlanner->getCurrentStep();

    Hand *mHand = graspitCore->getWorld()->getCurrentHand();

    feedback_.grasps.clear();
    feedback_.energies.clear();
    for(int i = 0; i < mPlanner->getListSize(); i++)
    {
        const GraspPlanningState *gps  = mPlanner->getGrasp(i);
        graspit_interface::Grasp g;
        graspPlanningStateToROSMsg(gps, g, mHand);

        feedback_.grasps.push_back(g);
        feedback_.energies.push_back(gps->getEnergy());
        feedback_.search_energy = goal.search_energy;
    }
}

void GraspitInterface::runPlannerInMainThread()
{
    ROS_INFO("Planner Starting in Mainloop");
    ROS_INFO("Getting Hand");
    Hand *mHand = graspitCore->getWorld()->getCurrentHand();
    if(mHand == NULL)
    {
        ROS_INFO("Planning Hand is NULL");
    }
    GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
    if(mObject == NULL)
    {
        ROS_INFO("Planning Object is NULL");
    }

    ROS_INFO("Initing mHandObjectState");
    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);

    switch(goal.search_space.type) {
        case graspit_interface::SearchSpace::SPACE_COMPLETE :
            {
                mHandObjectState->setPositionType(SPACE_COMPLETE);
                mHandObjectState->setRefTran( mObject->getTran() );
                break;
            }
        case graspit_interface::SearchSpace::SPACE_AXIS_ANGLE :
            {
                mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
                mHandObjectState->setRefTran( mObject->getTran() );
                break;
            }
        case graspit_interface::SearchSpace::SPACE_ELLIPSOID :
            {
                mHandObjectState->setPositionType(SPACE_ELLIPSOID);
                mHandObjectState->setRefTran( mObject->getTran() );
                break;
            }
        case graspit_interface::SearchSpace::SPACE_APPROACH :
            {
                mHandObjectState->setPositionType(SPACE_APPROACH);
                mHandObjectState->setRefTran( mHand->getTran() );
                break;
            }
        default:
            {
                ROS_INFO("Invalid Search Space Type");
                //return;
            }
    }

    ROS_INFO("Initing mHandObjectState");
    mHandObjectState->reset();

    ROS_INFO("Initing mPlanner");

    switch(goal.planner.type) {
        case graspit_interface::Planner::SIM_ANN :
            {
                mPlanner = new SimAnnPlanner(mHand);
                ROS_INFO("Using graspit_interface::Planner::SIM_ANN ");
                break;
            }
        case graspit_interface::Planner::MULTI_THREADED :
            {
                mPlanner = new GuidedPlanner(mHand);
                ROS_INFO("Using graspit_interface::Planner::MULTI_THREADED ");
                break;
            }
        default:
            {
                ROS_INFO("Invalid Planner Type");
                //return;
            }
    }


    mPlanner->setEnergyType(goal.search_energy);

    if (goal.sim_ann_params.set_custom_params)
    {
        ROS_INFO("Switching SimAnn Annealing parameters to your custom defined values!!! ");
        SimAnnParams simAnnParams;
        simAnnParams.YC = goal.sim_ann_params.YC;
        simAnnParams.HC = goal.sim_ann_params.HC;
        simAnnParams.YDIMS = goal.sim_ann_params.YDIMS;
        simAnnParams.NBR_ADJ = goal.sim_ann_params.NBR_ADJ;
        simAnnParams.ERR_ADJ = goal.sim_ann_params.ERR_ADJ;
        simAnnParams.DEF_T0 = goal.sim_ann_params.DEF_T0;
        simAnnParams.DEF_K0 = goal.sim_ann_params.DEF_K0;
        mPlanner->setAnnealingParameters(simAnnParams);
    }


    switch(goal.search_contact.type) {
        case graspit_interface::SearchContact::CONTACT_PRESET :
            {
                mPlanner->setContactType(CONTACT_PRESET);
                ROS_INFO("Using graspit_interface::SearchContact::CONTACT_PRESET ");
                break;
            }
        case graspit_interface::SearchContact::CONTACT_LIVE :
            {
                mPlanner->setContactType(CONTACT_LIVE);
                ROS_INFO("Using graspit_interface::SearchContact::CONTACT_LIVE ");
                break;
            }
        default:
            {
                ROS_INFO("Invalid Search Contact Type");
                //return;
            }
    }

    ROS_INFO("Setting Planner Model State");
    mPlanner->setModelState(mHandObjectState);
    int max_steps = goal.max_steps;
    if(max_steps ==0)
    {
        max_steps = 70000;
    }
    ROS_INFO("Setting Planner Max Steps %d", max_steps);
    mPlanner->setMaxSteps(max_steps);

    ROS_INFO("resetting Planner");
    mPlanner->resetPlanner();

    ROS_INFO("Starting Planner");
    mPlanner->startPlanner();

 }

void GraspitInterface::graspPlanningStateToROSMsg(const GraspPlanningState* gps, graspit_interface::Grasp &g, Hand *mHand){
    gps->execute(mHand);
    mHand->autoGrasp(false,1.0,false);

    geometry_msgs::Pose pose;
    transf t = mHand->getTran();
    pose.position.x = t.translation().x() / 1000.0;
    pose.position.y = t.translation().y() / 1000.0;;
    pose.position.z = t.translation().z() / 1000.0;;
    pose.orientation.w = t.rotation().w();
    pose.orientation.x = t.rotation().x();
    pose.orientation.y = t.rotation().y();
    pose.orientation.z = t.rotation().z();

    geometry_msgs::Vector3Stamped approach_direction;
    vec3 approachInHand = mHand->getApproachTran() *  vec3 (0, 0, 1);
    approachInHand.normalize();
    approach_direction.vector.x = approachInHand.x();
    approach_direction.vector.y = approachInHand.y();
    approach_direction.vector.z = approachInHand.z();
    approach_direction.header.frame_id = mHand->getName().toStdString();

    g.graspable_body_id = goal.graspable_body_id;

    double dof[mHand->getNumDOF()];
    mHand->getDOFVals(dof);
    for(int i = 0; i <mHand->getNumDOF(); ++i)
    {
        g.dofs.push_back(dof[i]);
    }

    g.pose = pose;
    mHand->getGrasp()->update();
    QualVolume mVolQual( mHand->getGrasp(), ("Volume"),"L1 Norm");
    QualEpsilon mEpsQual( mHand->getGrasp(), ("Epsilon"),"L1 Norm");

    graspitCore->getWorld()->findAllContacts();
    graspitCore->getWorld()->updateGrasps();

    g.epsilon_quality= mEpsQual.evaluate();
    g.volume_quality = mVolQual.evaluate();
    g.approach_direction = approach_direction;
}


 void GraspitInterface::processPlannerResultsInMainThread()
 {
     Hand *mHand = graspitCore->getWorld()->getCurrentHand();
     if(mHand == NULL)
     {
         ROS_INFO("Planning Hand is NULL");
     }
     GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
     if(mObject == NULL)
     {
         ROS_INFO("Planning Object is NULL");
     }

    result_.grasps.clear();
    result_.energies.clear();

    ROS_INFO("Publishing Result");
    for(int i = 0; i < mPlanner->getListSize(); i++)
    {
        const GraspPlanningState *gps  = mPlanner->getGrasp(i);
        graspit_interface::Grasp g;
        graspPlanningStateToROSMsg(gps, g, mHand);

        result_.grasps.push_back(g);
        result_.energies.push_back(gps->getEnergy());
        result_.search_energy = goal.search_energy;
    }

    if(mPlanner->getListSize() > 0)
    {
        ROS_INFO("Showing Grasp 0");
        mPlanner->showGrasp(0);
    }

    if(mHandObjectState != NULL)
    {
        delete mHandObjectState;
        mHandObjectState = NULL;
    }

    if(mPlanner != NULL)
    {
        delete mPlanner;
        mPlanner = NULL;
    }
}

}
