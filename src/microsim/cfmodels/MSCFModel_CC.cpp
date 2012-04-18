/****************************************************************************/
/// @file    MSCFModel_CC.cpp
/// @author  Michele Segata
/// @date    Wed, 18 Apr 2012
/// @version $Id: $
///
// A series of automatic Cruise Controllers (CC, ACC, CACC)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2011 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSCFModel_CC.h"
#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_CC::MSCFModel_CC(const MSVehicleType* vtype,
                           SUMOReal accel, SUMOReal decel,
                           SUMOReal ccDecel, SUMOReal headwayTime, SUMOReal constantSpacing,
                           SUMOReal kp, SUMOReal lambda, SUMOReal c1, SUMOReal xi,
                           SUMOReal omegaN, SUMOReal tau)
    : MSCFModel(vtype, accel, decel, headwayTime), myCcDecel(ccDecel), myConstantSpacing(constantSpacing)
    , myKp(kp), myLambda(lambda), myC1(c1), myXi(xi), myOmegaN(omegaN), myTau(tau), myAlpha1(1 - myC1), myAlpha2(myC1),
    myAlpha3(-(2 * myXi - myC1 *(myXi + sqrt(myXi* myXi - 1))) * myOmegaN), myAlpha4(-(myXi + sqrt(myXi* myXi - 1)) * myOmegaN* myC1),
    myAlpha5(-myOmegaN* myOmegaN), myAlpha(DELTA_T / (myTau + DELTA_T)), myOneMinusAlpha(1 - myAlpha) {
    if (DELTA_T != 100) {
        std::cerr << "Fatal error: in order to use automatic cruise control models, time step must be set to 100 ms\n";
        assert(false);
    }
}

MSCFModel_CC::~MSCFModel_CC() {}


SUMOReal
MSCFModel_CC::moveHelper(MSVehicle* const veh, SUMOReal vPos) const {
    const SUMOReal vNext = MSCFModel::moveHelper(veh, vPos);
    //TODO: understand and change this method
    return vNext;
}


SUMOReal
MSCFModel_CC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal /*predMaxDecel*/) const {
    return _v(veh, gap2pred, speed, predSpeed, desiredSpeed(veh));
}


SUMOReal
MSCFModel_CC::stopSpeed(const MSVehicle* const veh, SUMOReal gap2pred) const {
    if (gap2pred < 0.01) {
            return 0;
        }
    return _v(veh, gap2pred, veh->getSpeed(), 0, desiredSpeed(veh));
    VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
    switch (vars->activeController) {
        case VehicleVariables::ACC:
        case VehicleVariables::CACC:

            std::cerr << "Cruise controllers should not be used when a stop can happen, like intersections\n";
            assert(false);
            break;

        case VehicleVariables::DRIVER:

            std::cerr << "stopSpeed() correctly invoked for driver, but still not implemented\n";
            assert(false);
            break;

        default:

            std::cerr << "Unkown controller selected\n";
            assert(false);
            break;

    }
}
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/xml/SUMOXMLDefinitions.h>

/// @todo update interactionGap logic to IDM
SUMOReal
MSCFModel_CC::interactionGap(const MSVehicle* const veh, SUMOReal vL) const {

    //TODO: understand and change this. for now use maximum radar range
    return 250;

}


SUMOReal
MSCFModel_CC::_v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal desSpeed) const {

    std::stringstream debug;

    //acceleration computed by the controller
    double controllerAcceleration;

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    //TODO: only for testing
    //-----------------------------------------------------------
    std::string id = veh->getID();
    if (id.substr(id.size() - 2, 2).compare(".0") == 0) {
        vars->activeController = VehicleVariables::ACC;
        if (MSNet::getInstance()->getCurrentTimeStep() < 120000)
            vars->ccDesiredSpeed = 36;
        else
            vars->ccDesiredSpeed = 30;
    } else {
        vars->activeController = VehicleVariables::CACC;
    }
    //-----------------------------------------------------------

    //this function has already been invoked for this timestep
    if (vars->lastUpdate == MSNet::getInstance()->getCurrentTimeStep()) {
        return vars->egoSpeed;
    }


    //update variables, first of all
    vars->egoSpeed = egoSpeed;
    vars->frontAcceleration = SPEED2ACCEL(predSpeed - vars->frontSpeed);
    vars->frontDistance = gap2pred;
    vars->frontSpeed = predSpeed;
    vars->lastUpdate = MSNet::getInstance()->getCurrentTimeStep();

    debug << "V " << id;

    switch (vars->activeController) {

        case VehicleVariables::ACC:

            debug << " uses CC";

            //TODO: modify. for now uses only CC
            controllerAcceleration = _cc(veh);

            break;

        case VehicleVariables::CACC:

            debug << " uses CACC";

            //TODO: again modify probably range/range-rate controller is needed
            controllerAcceleration = _cacc(veh);

            break;

        case VehicleVariables::DRIVER:

            std::cerr << "Switching to normal driver behavior still not implemented in MSCFModel_CC\n";
            assert(false);
            break;

        default:

            std::cerr << "Invalid controller selected in MSCFModel_CC\n";
            assert(false);
            break;

    }

    debug << " accel=" << controllerAcceleration;

    //compute the actual acceleration applied by the engine
    vars->egoAcceleration = _actuator(veh, controllerAcceleration);

    debug << " actuators=" << vars->egoAcceleration;

    //compute the speed from the actual acceleration
    vars->egoSpeed += ACCEL2SPEED(vars->egoAcceleration);

    debug << " speed=" << vars->egoSpeed << "\n";

    WRITE_MESSAGE(debug.str());

    return MAX2(SUMOReal(0), vars->egoSpeed);
}

SUMOReal
MSCFModel_CC::_cc(const MSVehicle* const veh) const {

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    return fmin(myAccel, fmax(-myCcDecel, -myKp * (vars->egoSpeed - vars->ccDesiredSpeed)));

}

SUMOReal
MSCFModel_CC::_acc(const MSVehicle* const veh) const {

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    return fmin(myAccel, fmax(-myDecel, -1.0 / myHeadwayTime * (vars->egoSpeed - vars->frontSpeed + myLambda * (-vars->frontDistance + myHeadwayTime * vars->egoSpeed))));

}

SUMOReal
MSCFModel_CC::_cacc(const MSVehicle* const veh) const {

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    //compute epsilon, i.e., the desired distance error
    double epsilon = -vars->frontDistance + myConstantSpacing; //NOTICE: error (if any) should already be included in frontDistance
    //compute epsilon_dot, i.e., the desired speed error
    double epsilon_dot = vars->egoSpeed - vars->frontSpeed;

    return fmin(myAccel, fmax(-myDecel, myAlpha1 * vars->frontAcceleration + myAlpha2 * vars->leaderAcceleration +
                              myAlpha3 * epsilon_dot + myAlpha4 * (vars->egoSpeed - vars->leaderSpeed) + myAlpha5 * epsilon));

}

SUMOReal
MSCFModel_CC::_actuator(const MSVehicle* const veh, SUMOReal acceleration) const {

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    return myAlpha * acceleration + myOneMinusAlpha * vars->egoAcceleration;

}

void
MSCFModel_CC::setCCDesiredSpeed(const MSVehicle* veh, SUMOReal ccDesiredSpeed) {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->ccDesiredSpeed = ccDesiredSpeed;
}

void
MSCFModel_CC::setLeaderInformation(const MSVehicle* veh, SUMOReal speed, SUMOReal acceleration) {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->leaderAcceleration = acceleration;
    vars->leaderSpeed = speed;
}

void
MSCFModel_CC::getVehicleInformation(const MSVehicle* veh, SUMOReal &speed, SUMOReal &acceleration) {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    speed = vars->egoSpeed;
    acceleration = vars->egoAcceleration;
}


MSCFModel*
MSCFModel_CC::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_CC(vtype,
                            myAccel, myDecel,
                            myCcDecel, myHeadwayTime, myConstantSpacing,
                            myKp, myLambda, myC1, myXi,
                            myOmegaN, myTau);
}
