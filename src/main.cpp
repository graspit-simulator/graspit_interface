#include "graspit_interface.h"

extern "C" Plugin* createPlugin() {
    return new GraspitInterface::GraspitInterface();
}

extern "C" std::string getType() {
    return "graspit_interface_plugin";
}
