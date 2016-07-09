#include "graspit_interface.h"

#include "graspit_source/include/graspitCore.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/world.h"
#include "graspit_source/include/ivmgr.h"

#include "graspit_source/include/quality.h"
#include "graspit_source/include/grasp.h"
#include "graspit_source/include/EGPlanner/searchState.h"
#include "graspit_source/include/EGPlanner/simAnnPlanner.h"
#include "graspit_source/include/EGPlanner/egPlanner.h"

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

    clearWorld_srv = nh->advertiseService("clearWorld", &GraspitInterface::clearWorldCB, this);
    loadWorld_srv = nh->advertiseService("loadWorld", &GraspitInterface::loadWorldCB, this);
    saveWorld_srv = nh->advertiseService("saveWorld", &GraspitInterface::saveWorldCB, this);

    saveImage_srv = nh->advertiseService("saveImage", &GraspitInterface::saveImageCB, this);
    toggleAllCollisions_srv = nh->advertiseService("toggleAllCollisions", &GraspitInterface::toggleAllCollisionsCB, this);

    computeQuality_srv = nh->advertiseService("computeQuality", &GraspitInterface::computeQualityCB, this);

    approachToContact_srv = nh->advertiseService("approahToContact", &GraspitInterface::approachToContactCB, this);
    findInitialContact_srv = nh->advertiseService("findInitialContact", &GraspitInterface::findInitialContactCB, this);

    plan_grasps_as = new actionlib::SimpleActionServer<graspit_interface::PlanGraspsAction>(*nh, "planGrasps",
                                                                                            boost::bind(&GraspitInterface::PlanGraspsCB, this, _1), false);
    plan_grasps_as->start();

    startPlanner = false;
    plannerStarted = false;
    requestRender = false;
    mPlanner = NULL;
    mHandObjectState = NULL;

    ROS_INFO("GraspIt interface successfully initialized!");

    return 0;
}

int GraspitInterface::mainLoop()
{
    //Planner Must be started by mainthread, so it cannot be
    //Started inside the callback for the action server.
    if(startPlanner)
    {
        ROS_INFO("Planner Starting in Mainloop");
        mPlanner->startPlanner();
        startPlanner = false;
        plannerStarted = true;
    }

    //Rendering must be handled my the main thread
    //So if one of the action servers wants a render
    //They set requestRender to true, and it will be rendered here.
    if(requestRender)
    {
        requestRender = false;

        ROS_INFO("Rendering");
        if(graspitCore->getIVmgr())
        {
            graspitCore->getIVmgr()->getViewer()->render();
        }
    }

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

    Hand *mHand = graspitCore->getWorld()->getCurrentHand();
    if (mHand==NULL)
    {
        response.result = response.RESULT_NO_HAND;
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

bool GraspitInterface::approachToContactCB(graspit_interface::ApproachToContact::Request &request,
                                           graspit_interface::ApproachToContact::Response &response)
{
    Hand *mHand = graspitCore->getWorld()->getCurrentHand();
     if (mHand==NULL)
     {
         response.result = response.RESULT_NO_HAND;
         return true;
     }
     mHand->approachToContact(request.moveDist, request.oneStep);
     return true;
}

bool GraspitInterface::findInitialContactCB(graspit_interface::FindInitialContact::Request &request,
                                            graspit_interface::FindInitialContact::Response &response)
                  {
    Hand *mHand = graspitCore->getWorld()->getCurrentHand();
     if (mHand==NULL)
     {
         response.result = response.RESULT_NO_HAND;
         return true;
     }
     mHand->findInitialContact(request.moveDist);
     return true;
}

void GraspitInterface::PlanGraspsCB(const graspit_interface::PlanGraspsGoalConstPtr &goal)
{
    ROS_INFO("Getting Hand");
    Hand *mHand = graspitCore->getWorld()->getCurrentHand();
    if(mHand == NULL)
    {
        ROS_INFO("Planning Hand is NULL");
    }
    GraspableBody *mObject = graspitCore->getWorld()->getGB(0);
    if(mHand == NULL)
    {
        ROS_INFO("Planning Object is NULL");
    }

    ROS_INFO("Initing mHandObjectState");
    mHandObjectState = new GraspPlanningState(mHand);
//    if ( goal->search_space.Type == graspit_interface::SearchSpace.Type ) {
//     mHandObjectState.setPositionType(SPACE_COMPLETE);
//     mHandObjectState.setRefTran( mObject->getTran() );
//    }
//    else if ( s==QString("Axis-angle") ) {
//     mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
//     mHandObjectState->setRefTran( mObject->getTran() );
//    } else if ( s==QString("Ellipsoid") ) {
//     mHandObjectState->setPositionType(SPACE_ELLIPSOID);
//     mHandObjectState->setRefTran( mObject->getTran() );
//    } else if ( s==QString("Approach") ) {
//     mHandObjectState->setPositionType(SPACE_APPROACH);
//     mHandObjectState->setRefTran( mHand->getTran() );
//    } else {
//     fprintf(stderr,"WRONG SEARCH TYPE IN DROP BOX!\n");
//    }
    ROS_INFO("Initing mHandObjectState");
    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    mHandObjectState->reset();

    ROS_INFO("Initing mPlanner");
    mPlanner = new SimAnnPlanner(mHand);

    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
    mPlanner->setContactType(CONTACT_PRESET);
    mPlanner->setModelState(mHandObjectState);
    mPlanner->setMaxSteps(35000);

    mPlanner->resetPlanner();

//    QString s = energyBox->currentText();
//    if ( s == QString("Hand Contacts") ) {
//     mPlanner->setEnergyType(ENERGY_CONTACT);
//    } else if ( s == QString("Potential Quality") ) {
//     mPlanner->setEnergyType(ENERGY_POTENTIAL_QUALITY);
//    } else if ( s == QString("Autograsp Quality") ) {
//     mPlanner->setEnergyType(ENERGY_AUTOGRASP_QUALITY);
//    } else if ( s == QString("Contacts AND Quality") ) {
//     mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
//    } else if ( s == QString("Guided Autograsp") ) {
//     mPlanner->setEnergyType(ENERGY_GUIDED_AUTOGRASP);
//    } else {
//     fprintf(stderr,"WRONG ENERGY TYPE IN DROP BOX!\n");
//    }

//    //contact type
//    if ( setContactsBox->isChecked() ) {
//     mPlanner->setContactType(CONTACT_PRESET);
//    } else {
//     mPlanner->setContactType(CONTACT_LIVE);
//    }

    startPlanner = true;

    ROS_INFO("Waiting For Planner to Start");
    while(startPlanner)
    {
        sleep(0.1);
    }

    ROS_INFO("Waiting For Planner to Finish");
    while(mPlanner->isActive())
    {
        sleep(0.1);
//        for(int i=0; i < mPlanner->getListSize(); i++)
//        {
//            mPlanner->getGrasp(i);
//        }
//        plan_grasps_as->publishFeedback(feedback_);
    }

    ROS_INFO("Planner Is Finished");

    ROS_INFO("Showing Grasp 0");
    if(mPlanner->getListSize() > 0)
    {
        mPlanner->showGrasp(0);
    }

    requestRender = true;

    ROS_INFO("Cleaning Up");
    delete mPlanner;
    delete mHandObjectState;
    mPlanner = NULL;
    mHandObjectState = NULL;
    startPlanner = false;
    plannerStarted = false;

    ROS_INFO("Publishing Result");
    plan_grasps_as->setSucceeded(result_);

    ROS_INFO("Action ServerCB Finished");
}

}
