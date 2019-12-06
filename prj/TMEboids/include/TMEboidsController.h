/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef TMEBOIDSCONTROLLER_H
#define TMEBOIDSCONTROLLER_H

#include "Controllers/Controller.h"
#include "RoboroboMain/common.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>


class RobotWorldModel;

class TMEboidsController : public Controller
{
	public:
		TMEboidsController( RobotWorldModel *__wm );
		~TMEboidsController();
    
		void reset();
		void step();
};


#endif

