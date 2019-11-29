/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEbraitenberg/include/TMEbraitenbergController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEbraitenbergController::TMEbraitenbergController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEbraitenbergController::~TMEbraitenbergController()
{
    // nothing to do.
}

void TMEbraitenbergController::reset()
{
    // nothing to do.
}

void TMEbraitenbergController::step()
{
    double dist_L = getDistanceAt(SENSOR_L);
    double dist_FL = getDistanceAt(SENSOR_FL);
    double dist_FFL = getDistanceAt(SENSOR_FFL);
    double dist_F = getDistanceAt(SENSOR_F);
    double dist_R = getDistanceAt(SENSOR_R);
    double dist_FR = getDistanceAt(SENSOR_FR);
    double dist_FFR = getDistanceAt(SENSOR_FFR);
    double rotation = 0;
    rotation = -1*dist_L+-1*dist_FL+-1*dist_FFL + dist_R+dist_FR+dist_FFR ;
    /*if(dist_FFR+dist_FR+dist_R > dist_L + dist_FFL + dist_FL){
    	rotation = 0.3;
    }else if(dist_FFR+dist_FR+dist_R < dist_L + dist_FFL + dist_FL){
    	rotation = -0.3;
    }*/
    setRotation(rotation);
    setTranslation(1);

}
