std_msgs/Header header

# id for the body the grasp was planned on.
int32 graspable_body_id

# pose of the hand with respect to the object 
# the grasp was planned on
geometry_msgs/Pose pose

sensor_msgs/JointState[] joints

float64[] dofs

float64 epsilon_quality
float64 volume_quality





