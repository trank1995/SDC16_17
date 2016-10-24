#include "sdcIntersection.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcIntersection)

// SDC-defined constants


/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */
void sdcIntersection::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
     printf("load");
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcIntersection::Init()
{
    
    printf("init");
}

sdcIntersection::sdcIntersection(){
    printf("manager creating: \n");
    //printf(manager.id+3);
    manager::manager(3);
}
