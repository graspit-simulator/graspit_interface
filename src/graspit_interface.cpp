#include "graspit_interface.h"

#include "graspit_source/include/graspitCore.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/world.h"
#include "graspit_source/include/ivmgr.h"

#include "graspit_source/include/quality.h"
#include "graspit_source/include/grasp.h"
#include "graspit_source/include/EGPlanner/searchState.h"
#include "graspit_source/include/EGPlanner/egPlanner.h"
#include "graspit_source/include/EGPlanner/simAnnPlanner.h"
#include "graspit_source/include/EGPlanner/guidedPlanner.h"


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

    forceRobotDOF_srv = nh->advertiseService("forceRobotDOF", &GraspitInterface::forceRobotDOFCB, this);
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
        firstTimeInMainLoop = false;
        ROS_INFO("Planner Signal/Slots connected");
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

        response.robot.pose = transfToRosMsg(r->getTran());
        response.robot.approach_direction = transfToRosMsg(r->getApproachTran());

        // Info for all contacts with this robot:
        std::list<Contact*> contacts = r->getContacts();
        for (std::list<Contact*>::iterator it = contacts.begin();
                it != contacts.end(); it++) {
            graspit_interface::Contact c;
            c.body1 = (*it)->getBody1()->getName().toStdString();
            c.body2 = (*it)->getBody2()->getName().toStdString();
            
            position p = (*it)->getPosition();
            c.position.x = p.x() * 0.001; 
            c.position.y = p.y() * 0.001; 
            c.position.z = p.z() * 0.001; 
            c.cof = (*it)->getCof();
            response.robot.contacts.push_back(c);
        }

        // Joint state and DOF information:
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

        response.graspable_body.pose = transfToRosMsg(b->getTran());

        response.graspable_body.element_name = b->getName().toStdString();
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
        // Get Body properties:
        response.body.pose = transfToRosMsg(b->getTran());
        response.body.element_name = b->getName().toStdString(); 
        response.body.material = b->getMaterial();
        response.body.transparency = b->getTransparency();
        response.body.youngs_modulus = b->getYoungs();
        response.body.is_dynamic = b->isDynamic();
        // Dynamic body properties, if applicable:
        if (b->isDynamic()) {
            DynamicBody * db = (DynamicBody*) b;
            response.body.max_radius = db->getMaxRadius();

            // ROS inertia msg includes mass, center mass, & inertia tensor:
            geometry_msgs::Inertia inert;
            // ros mass = kgs.  graspit mass = grams:
            inert.m = db->getMass() * 0.001; 
            position cog = db->getCoG();
            inert.com.x = cog.x(); inert.com.y = cog.y(); inert.com.z = cog.z(); 
            const double * in = db->getInertia();
            // TODO not sure if these line up right:
            inert.ixx = in[0]; inert.ixy = in[1]; inert.ixz = in[2];
            inert.iyy = in[4]; inert.iyz = in[5]; inert.izz = in[8];
            response.body.inertia = inert;

            // ROS accel msg:
            geometry_msgs::Accel accel; 
            geometry_msgs::Vector3 lin;
            geometry_msgs::Vector3 ang;
            const double * ac = db->getAccel();
            // TODO not sure if these line up right:
            lin.x = ac[0]; lin.y = ac[1]; lin.z = ac[2]; 
            ang.x = ac[3]; ang.y = ac[4]; ang.z = ac[5]; 
            accel.linear = lin; accel.angular = ang;
            response.body.accel = accel;

            // ROS vel msg:
            geometry_msgs::Twist vel; 
            const double * v = db->getVelocity();
            // TODO not sure if these line up right:
            lin.x = v[0]; lin.y = v[1]; lin.z = v[2]; 
            ang.x = v[3]; ang.y = v[4]; ang.z = v[5]; 
            vel.linear = lin; vel.angular = ang;
            response.body.velocity = vel;
        }
        else {
            response.body.inertia = geometry_msgs::Inertia();
            response.body.accel = geometry_msgs::Accel();
            response.body.velocity = geometry_msgs::Twist();
            response.body.max_radius = -1;
        }
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
        QString name = graspitCore->getWorld()->getBody(i)->getName();
        response.element_names.push_back(name.toStdString());
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
        transf newTransform = rosMsgToTransf(request.pose);

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
        transf newTransform = rosMsgToTransf(request.pose);

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
        transf newTransform = rosMsgToTransf(request.pose);

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
        // Check that desired values are within range.
        //  If outside valid range, cap at min/max.
        Hand *hand = graspitCore->getWorld()->getHand(request.id);
        double * dof = request.dofs.data();
        for (int d=0; d<hand->getNumDOF(); d++) {
            if (dof[d] < hand->getDOF(d)->getMin()) {
                ROS_WARN("Desired value %f is out of range for DOF %d.  (min: %f, max:%f).\n\tCapped value at min.",
                        dof[d], d, hand->getDOF(d)->getMin(),hand->getDOF(d)->getMax());
                dof[d] = hand->getDOF(d)->getMin();
            }  
            else if (dof[d] > hand->getDOF(d)->getMax()) {
                ROS_WARN("Desired value %f is out of range for DOF %d.  (min: %f, max:%f).\n\tCapped value at max.",
                        dof[d], d, hand->getDOF(d)->getMin(),hand->getDOF(d)->getMax());
                dof[d] = hand->getDOF(d)->getMax();
            }
        }
        hand->forceDOFVals(dof);
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
        Hand *hand = graspitCore->getWorld()->getHand(request.id);
        for(int i=0; i < hand->getNumDOF(); i++) {
            hand->getDOF(i)->setDesiredVelocity(request.dof_velocities.data()[i]);
        }
        // Check that desired values are within range.
        //  If outside valid range, cap at min/max.
        double * dof = request.dofs.data();
        for (int d=0; d<hand->getNumDOF(); d++) {
            if (dof[d] < hand->getDOF(d)->getMin()) {
                ROS_WARN("Desired value %f is out of range for DOF %d.  (min: %f, max:%f).\n\tCapped value at min.",
                        dof[d], d, hand->getDOF(d)->getMin(),hand->getDOF(d)->getMax());
                dof[d] = hand->getDOF(d)->getMin();
            }  
            else if (dof[d] > hand->getDOF(d)->getMax()) {
                ROS_WARN("Desired value %f is out of range for DOF %d.  (min: %f, max:%f).\n\tCapped value at max.",
                        dof[d], d, hand->getDOF(d)->getMin(),hand->getDOF(d)->getMax());
                dof[d] = hand->getDOF(d)->getMax();
            }
        }

        hand->setDesiredDOFVals(request.dofs.data());
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
    // Identifier for this body.
    response.element_name = r->getName().toStdString(); 
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
    // Identifier for this body.
    response.element_name = b->getName().toStdString(); 
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
        Body * b = graspitCore->getWorld()->importBody(QString("GraspableBody"),QString(request.filename.data()));
        if(b == NULL){
            response.result = response.RESULT_FAILURE;
            return true;
        }
    }

    // Identifier for this body.
    response.element_name = b->getName().toStdString(); 
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
    while(mPlanner->isActive())
    {
        sleep(1.0);
        ROS_INFO("Curret Planner Step: %d", mPlanner->getCurrentStep());
        ROS_INFO("Curret Num Grasps: %d", mPlanner->getListSize());

        feedback_.current_step = mPlanner->getCurrentStep();
        feedback_.current_num_grasps = mPlanner->getListSize();
        plan_grasps_as->publishFeedback(feedback_);
    }

    ROS_INFO("About to Call emit emitProcessPlannerResultsInMainThread();");
    emit emitProcessPlannerResultsInMainThread();

    plan_grasps_as->setSucceeded(result_);
    ROS_INFO("Action ServerCB Finished");
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

    switch(goal.search_energy.type) {
        case graspit_interface::SearchEnergy::ENERGY_CONTACT_QUALITY :
            {
                mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
                ROS_INFO("Using graspit_interface::SearchEnergy::ENERGY_CONTACT_QUALITY ");
                break;
            }
        case graspit_interface::SearchEnergy::ENERGY_POTENTIAL_QUALITY :
            {
                mPlanner->setEnergyType(ENERGY_POTENTIAL_QUALITY);
                ROS_INFO("Using graspit_interface::SearchEnergy::ENERGY_POTENTIAL_QUALITY ");
                break;
            }
        case graspit_interface::SearchEnergy::ENERGY_CONTACT :
            {
                mPlanner->setEnergyType(ENERGY_CONTACT);
                ROS_INFO("Using graspit_interface::SearchEnergy::ENERGY_CONTACT ");
                break;
            }
        case graspit_interface::SearchEnergy::ENERGY_AUTOGRASP_QUALITY :
            {
                mPlanner->setEnergyType(ENERGY_AUTOGRASP_QUALITY);
                ROS_INFO("Using graspit_interface::SearchEnergy::ENERGY_AUTOGRASP_QUALITY ");
                break;
            }
        case graspit_interface::SearchEnergy::ENERGY_GUIDED_AUTOGRASP :
            {
                mPlanner->setEnergyType(ENERGY_GUIDED_AUTOGRASP);
                ROS_INFO("Using graspit_interface::SearchEnergy::ENERGY_GUIDED_AUTOGRASP ");
                break;
            }
        default:
            {
                ROS_INFO("Invalid Search Energy Type");
                //return;
            }
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
        ROS_INFO("Loading Grasp");
        const GraspPlanningState *gps  = mPlanner->getGrasp(i);
        gps->execute(mHand);
        mHand->autoGrasp(false,1.0,false);

        ROS_INFO("Building Pose");

        graspit_interface::Grasp g;
        g.pose = transfToRosMsg(mHand->getTran());

        g.graspable_body_id = goal.graspable_body_id;

        double dof[mHand->getNumDOF()];
        mHand->getDOFVals(dof);
        for(int i = 0; i <mHand->getNumDOF(); ++i)
        {
            g.dofs.push_back(dof[i]);
        }

        mHand->getGrasp()->update();
        QualVolume mVolQual( mHand->getGrasp(), ("Volume"),"L1 Norm");
        QualEpsilon mEpsQual( mHand->getGrasp(), ("Epsilon"),"L1 Norm");

        graspitCore->getWorld()->findAllContacts();
        graspitCore->getWorld()->updateGrasps();

        g.epsilon_quality= mEpsQual.evaluate();
        g.volume_quality = mVolQual.evaluate();

        ROS_INFO("Pushing back grasp");
        result_.grasps.push_back(g);
        result_.energies.push_back(gps->getEnergy());
        result_.search_energy = goal.search_energy;
    }

    ROS_INFO("Showing Grasp 0");
    if(mPlanner->getListSize() > 0)
    {
        mPlanner->showGrasp(0);
    }

    ROS_INFO("Cleaning up mPlanner and mHandObjectState");
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
