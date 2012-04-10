/*
 * MSCFModel_CACC.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: segata
 */

#include "MSCFModel_CACC.h"

#include <iostream>
using namespace std;

MSCFModel_CACC::MSCFModel_CACC(const MSVehicleType* vtype, SUMOReal accel, SUMOReal decel, SUMOReal desiredGap) :
		MSCFModel(vtype, accel, decel, 1.0),
		C1(0.5), engineTimeConst(.5),myDesiredGap(desiredGap),omega_n(0.2),xi(1.0) {

	//TODO: for testing: set first vehicle as leader

	if (DELTA_T != 100)
	{
		cerr << "FATAL: in order to properly work, the time step for using CACC must be set to 100ms\n";
		assert(false);
	}

	//set alpha and 1-alpha
	alpha = TS / (TS + engineTimeConst);
	oneMinusAlpha = 1 - alpha;

	//see Eq. 7.39 in rajamani book
	var1 = 1 - C1;
	var2 = C1;
	var3 = -(2*xi - C1 * (xi + sqrt(xi*xi - 1))) * omega_n;
	var4 = -(xi + sqrt(xi*xi - 1)) * omega_n * C1;
	var5 = -(omega_n * omega_n);

}

MSCFModel_CACC::~MSCFModel_CACC() {}

SUMOReal
MSCFModel_CACC::moveHelper(MSVehicle* const veh, SUMOReal vPos) const {
    const SUMOReal vNext = MSCFModel::moveHelper(veh, vPos);
    return vNext;
}

SUMOReal
MSCFModel_CACC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal /*predMaxDecel*/) const {
    return _v(veh, gap2pred, speed, predSpeed, desiredSpeed(veh));
}


SUMOReal
MSCFModel_CACC::stopSpeed(const MSVehicle* const veh, SUMOReal gap2pred) const {
    if (gap2pred<0.01) {
        return 0;
    }
    return _v(veh, gap2pred, veh->getSpeed(), 0, desiredSpeed(veh));
}


//TODO: continue from here

/// @todo update interactionGap logic to IDM
SUMOReal
MSCFModel_CACC::interactionGap(const MSVehicle* const veh, SUMOReal vL) const {
    // Resolve the IDM equation to gap. Assume predecessor has
    // speed != 0 and that vsafe will be the current speed plus acceleration,
    // i.e that with this gap there will be no interaction.
    /*SUMOReal acc = myAccel * (1. - pow(veh->getSpeed()/desiredSpeed(veh), myDelta));
    SUMOReal vNext = veh->getSpeed() + acc;
    SUMOReal gap = (vNext - vL) * (veh->getSpeed() + vL) / (2*myDecel) + vL;

    // Don't allow timeHeadWay < deltaT situations.
    return MAX2(gap, SPEED2DIST(vNext));*/

	//TODO: understand this. for now set maximum radar range
	return 250;

}


SUMOReal
MSCFModel_CACC::_v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal desSpeed) const {

	//TODO: test for leader!!
	string id = veh->getID();
	std::stringstream ss;

	if (id.substr(id.size() - 2, 2).compare(".0") == 0)
	{
		return 36.11;
	}

	if (gap2pred < 110 && egoSpeed > 36.11)
	{
		WRITE_MESSAGE("");
	}

	ss << "Vehicle " << id << " runs at " << egoSpeed << " at a distance of " << gap2pred;
	WRITE_MESSAGE(ss.str());

	//get state variables for CACC
	VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();

	//compute epsilon, i.e., the desired distance error
	double epsilon = -gap2pred + myDesiredGap; //+ error
	//compute epsilon_dot, i.e., the desired speed error
	double epsilon_dot = egoSpeed - predSpeed; //+ error
	//compute acceleration of previous vehicle, basing on its previous and current speed
	double predAcc = SPEED2ACCEL(predSpeed - vars->predPreviousSpeed);

	//MS TODO: testing phase
	size_t prefix_end = id.find_last_of('.');
	SUMOVehicle *leader = MSNet::getInstance()->getVehicleControl().getVehicle(id.substr(0, prefix_end) + ".0");

	vars->leaderAcc = SPEED2ACCEL(vars->leaderSpeed - leader->getSpeed());
	vars->leaderSpeed = leader->getSpeed();

	//now apply the CACC formula to compute the acceleration that should be applied
	double computedAcc = var1 * predAcc + var2 * vars->leaderAcc + var3 * epsilon_dot + var4 * (egoSpeed - vars->leaderSpeed) + var5 * epsilon;
	computedAcc = MIN2(myAccel, MAX2(-myDecel, computedAcc));
	//filter the computed acceleration through a low-pass filter, in order to simulate the delayed engine response
	double engineAcc = alpha * computedAcc + oneMinusAlpha * vars->prevEgoAcc;

	//save new variables into the CACC state
	vars->predPreviousSpeed = predSpeed;
	vars->prevEgoAcc = engineAcc;

	//now compute the new speed and return it
	return MAX2(SUMOReal(0), egoSpeed + ACCEL2SPEED(engineAcc));

}


MSCFModel*
MSCFModel_CACC::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_CACC(vtype, myAccel, myDecel, myDesiredGap);
}
