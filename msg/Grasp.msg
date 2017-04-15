# id for the body the grasp was planned on.
int32 graspable_body_id

# pose of the hand with respect to the object 
# the grasp was planned on
geometry_msgs/Pose pose

float64[] dofs

float64 epsilon_quality
float64 volume_quality

# The approach direction to take before picking an object
geometry_msgs/Vector3Stamped approach_direction

#model_name of object to be grasped
string object_name

int32 grasp_id

float64[] secondary_qualities
int32 grasp_source
int32 grasp_group
int32 grasp_type


geometry_msgs/Pose pre_grasp_pose
geometry_msgs/Pose final_grasp_pose
float64[] pre_grasp_dof
float64[] final_grasp_dof
#geometry_msgs/Pose demonstration_pose

int32 TYPE_UNKNOWN = 1
int32 TYPE_FINGERTIP = 2
int32 TYPE_POWER_GRASP = 3        # free grasp
int32 TYPE_TABLECONTACT_GRASP = 4 # finger will make contact w/ table
int32 SPREAD_DOF=0
int32 FINGER_1_DOF=1
int32 FINGER_2_DOF=2
int32 FINGER_3_DOF=3
int32 SOURCE_EIGENGRASPS=1
int32 SOURCE_HUMAN=2
int32 SOURCE_HUMAN_REFINED=3
int32 SOURCE_TABLETOP_ALIGNED=7

