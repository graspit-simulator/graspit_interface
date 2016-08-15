std_msgs/Header header

string  element_name #UID for the body's world element.
geometry_msgs/Pose pose

# Other properties
int32     material
float64  transparency
float64  youngs_modulus
bool    is_dynamic
bool is_elastic

# Dynamic body properties. -1/None if body is not dynamic:
float64 mass
geometry_msgs/Inertia inertia
#this is DynamicBody()->getCoG and DynamicBody->getInertia()
geometry_msgs/Accel accel
geometry_msgs/Twist velocity
float64 max_radius
