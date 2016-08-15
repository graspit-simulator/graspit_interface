# Constants:
std_msgs/Header header
string element_name #UID for the body's world element.
geometry_msgs/Pose approach_direction

# Variables:
geometry_msgs/Pose pose
sensor_msgs/JointState[] joints
float64[] dofs
Contact[] contacts
