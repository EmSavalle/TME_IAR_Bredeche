/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef TMEEVOLUTIONCONTROLLER_H
#define TMEEVOLUTIONCONTROLLER_H

#include "Controllers/Controller.h"
#include "RoboroboMain/common.h"

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

class RobotWorldModel;

class TMEevolutionController : public Controller
{
	public:
		TMEevolutionController( RobotWorldModel *__wm );
		~TMEevolutionController();
		
		double[] genome;
    
		void reset();
		void step();
};


#endif

