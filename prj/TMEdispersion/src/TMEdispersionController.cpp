/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEdispersion/include/TMEdispersionController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEdispersionController::TMEdispersionController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEdispersionController::~TMEdispersionController()
{
    // nothing to do.
}

void TMEdispersionController::reset()
{
    // nothing to do.
}

void TMEdispersionController::step()
{
	int sens[] = {SENSOR_L,SENSOR_FL,SENSOR_FFL,SENSOR_FLL,SENSOR_R,SENSOR_FR,SENSOR_FFR,SENSOR_FRR,SENSOR_F,SENSOR_BR,SENSOR_BL,SENSOR_B};
	int leftMin = 0, leftMax = 3, rightMin = 4, rightMax = 7 , front = 8 , behindMin = 9, behindMax = 11;

    double rotation = 0;
    int translation = 0;
	for (int i = 0 ; i < sizeof(sens) ; i ++){
		if(i >= leftMin && i <= leftMax && getRobotIdAt(sens[i])!=-1){
			rotation += 0.1;
			translation = -1;
		}
		else if(i >= rightMin && i <= rightMax && getRobotIdAt(sens[i])!=-1){
			rotation -= 0.1;
			translation = -1;
		}
		else if(i == front && getRobotIdAt(sens[i])!=-1){
			translation = -1;
		}
		else if(i >= behindMin && i <= behindMax && getRobotIdAt(sens[i])!=-1){
			translation = 1;
			if(sens[i] == SENSOR_BR){
				rotation +=0.1;
			}
			else if(sens[i] == SENSOR_BL){
				rotation -=0.1;
			}
		}
	}
    setRotation(rotation);
    setTranslation(translation);
}
