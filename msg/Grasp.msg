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




