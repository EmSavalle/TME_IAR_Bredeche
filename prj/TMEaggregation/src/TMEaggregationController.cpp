/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEaggregation/include/TMEaggregationController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEaggregationController::TMEaggregationController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEaggregationController::~TMEaggregationController()
{
    // nothing to do.
}

void TMEaggregationController::reset()
{
    // nothing to do.
}

void TMEaggregationController::step()
{
	double dist_L = getWallAt(SENSOR_L);
    double dist_FL = getWallAt(SENSOR_FL);
    double dist_FFL = getWallAt(SENSOR_FFL);
    double dist_F = getWallAt(SENSOR_F);
    double dist_R = getWallAt(SENSOR_R);
    double dist_FR = getWallAt(SENSOR_FR);
    double dist_FFR = getWallAt(SENSOR_FFR);

    
    setTranslation(1);
	double rotation = 0;
    if(getRobotIdAt(SENSOR_L)!=-1){
     	rotation -= 0.1;
    }
    else if(getRobotIdAt(SENSOR_FL)!=-1){
     	rotation -= 0.2;
    }
    else if(getRobotIdAt(SENSOR_FFL)!=-1){
     	rotation -= 0.3;
    }
    else if(getRobotIdAt(SENSOR_R)!=-1){
     	rotation += 0.1;
    }
    else if(getRobotIdAt(SENSOR_FR)!=-1){
     	rotation += 0.2;
    }
    else if(getRobotIdAt(SENSOR_FFR)!=-1){
     	rotation += 0.3;
    }
    else if(dist_FFR+dist_FR+dist_R > dist_L + dist_FFL + dist_FL){
    	rotation -= 0.3;
    }else if(dist_FFR+dist_FR+dist_R < dist_L + dist_FFL + dist_FL){
    	rotation += 0.3;
    }
    setRotation(rotation);
    setTranslation(1);
}
