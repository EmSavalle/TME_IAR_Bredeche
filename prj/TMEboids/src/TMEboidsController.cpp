/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEboids/include/TMEboidsController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEboidsController::TMEboidsController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
    
}

TMEboidsController::~TMEboidsController()
{
    // nothing to do.
}

void TMEboidsController::reset()
{
    // nothing to do.
}


void TMEboidsController::step()
{
	// TODO marche bof, à optimiser....
	// (de nombreux robots se "collent" et se bloquent)
	typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;
	
	int vision_field[] = {SENSOR_L, SENSOR_FL, SENSOR_FFL, SENSOR_FLL, SENSOR_R, SENSOR_FR, SENSOR_FFR, SENSOR_FRR, SENSOR_F}; // the sensor belt without the ones in the back (blind spot)
	
	polygon_type neighbours; // making a polygon of the neighbours to get its centroid
	// These can be modified to get different behaviours
    double zor = 0.2; // range for the zone of repulsion (as a percentage of max range)
    double zoo = 0.5; // range for the zone of orientation (as a percentage of max range)
    //double zoa = 1.; // range for the zone of attraction (as a percentage of max range)
     
    /* Choosing a mode for the boids
     * 0 = repulsion, 1 = orientation
     * 2 = attraction, 3 = all of them
     * 4 = all of them + wallAvoidance
     */
    int mode = 4;
    
    double rotation = 0;
	int translation = 1;
    
    if (mode == 0 || mode >= 3) { // repulsion
    	std::cout << "Repulsion..." << std::endl;
		for(int i = 0; i < 9; i++) {
			std::cout << i << std::endl;
			if(getRobotIdAt(i) != -1 && getDistanceAt(i) <= zor) {
				Point2d ngbr = getRobotControllerAt(i)->getPosition(); //position of the neighbour
				boost::geometry::append(neighbours.outer(), point_type(ngbr.x, ngbr.y));
			}
		}
		point_type centr; //centroid of all neighbours
		try{
			boost::geometry::centroid(neighbours, centr);
			Point2d target(boost::geometry::get<0>(centr), boost::geometry::get<1>(centr));
			std::cout << "repulsion centroid: " << boost::geometry:: dsv(centr) << std::endl;
			rotation = -getAngleToTarget(getPosition(), getOrientation(), target); // we want to get away from target, hence the minus
			translation = 1 - abs(getEuclideanDistance(getPosition(), target)); // the closer the target, the farther away we want to get
		} catch(boost::geometry::centroid_exception e) { }
		
		if (mode == 0) {
			std:: cout << "Setting rotation to " << rotation << "and translation to " << translation << std::endl;
			setRotation(rotation);
			setTranslation(translation);
		}
	}
	
	if (mode == 1 || mode >= 3) { // orientation
		std::cout << "Orientation..." << std::endl;
		for(int i = 0; i != sizeof(vision_field); i++) {
			double dist = getDistanceAt(i);
			if(getRobotIdAt(i) != -1 && dist > zor && dist <= zoo) {
				rotation += getRobotRelativeOrientationAt(i);
			}
		}
		if (mode == 1) {
			std:: cout << "Setting rotation to " << rotation << "and translation to " << translation << std::endl;
			setRotation(rotation);
			setTranslation(translation);
		}
	}
	
	if (mode >= 2) { //attraction
		std::cout << "Attraction..." << std::endl;
		for(int i = 0; i < 9; i++) {
			if(getRobotIdAt(i) != -1 && getDistanceAt(i) > zoo) {
				Point2d ngbr = getRobotControllerAt(i)->getPosition(); //position of the neighbour
				boost::geometry::append(neighbours.outer(), point_type(ngbr.x, ngbr.y));
			}
		}
		point_type centr; //centroid of all neighbours
		try{
			boost::geometry::centroid(neighbours, centr);
			Point2d target(boost::geometry::get<0>(centr), boost::geometry::get<1>(centr));
			rotation += getAngleToTarget(getPosition(), getOrientation(), target);
			if (mode == 2)
				translation = abs(getEuclideanDistance(getPosition(), target)); // the farther the target, the faster we want to get there
			else
				translation = (translation + abs(getEuclideanDistance(getPosition(), target)))/2.;
		} catch(boost::geometry::centroid_exception e) {
			rotation += 0;
			translation = (translation + 1.)/2.;
		}
		
		if(mode != 4) {
			std:: cout << "Setting rotation to " << rotation << "and translation to " << translation << std::endl;
			setRotation(rotation);
			setTranslation(translation);
		}
	}	
	
	if (mode == 4) {
		std::cout << "Wall avoidance..." << std::endl;
		double dist_L = getWallAt(SENSOR_L);
		double dist_FL = getWallAt(SENSOR_FL);
		double dist_FFL = getWallAt(SENSOR_FFL);
		double dist_F = getWallAt(SENSOR_F);
		double dist_R = getWallAt(SENSOR_R);
		double dist_FR = getWallAt(SENSOR_FR);
		double dist_FFR = getWallAt(SENSOR_FFR);
		
		if(dist_FFR+dist_FR+dist_R > dist_L + dist_FFL + dist_FL){
    		rotation -= 0.3;
    	}else if(dist_FFR+dist_FR+dist_R < dist_L + dist_FFL + dist_FL){
    		rotation += 0.3;
    	}
    	std:: cout << "Setting rotation to " << rotation << "and translation to " << translation << std::endl;
    	setRotation(rotation);
    	setTranslation(translation);
	}
		
}
