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
    myAlpha5(-myOmegaN* myOmegaN), myAlpha(TS / (myTau + TS)), myOneMinusAlpha(1 - myAlpha) {

    if (DELTA_T != 100) {
        std::cerr << "Fatal error: in order to use automatic cruise control models, time step must be set to 100 ms\n";
        assert(false);
    }

    //instantiate the driver model. For now, use Krauss as default, then needs to be parameterized
    myHumanDriver = new MSCFModel_Krauss(vtype, accel, decel, 0.5, 1.5);
}

MSCFModel_CC::~MSCFModel_CC() {}


SUMOReal
MSCFModel_CC::moveHelper(MSVehicle* const veh, SUMOReal vPos) const {
    SUMOReal vNext;
    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != VehicleVariables::DRIVER)
        //TODO: understand and change this method
        vNext = MSCFModel::moveHelper(veh, vPos);
    else
        vNext = myHumanDriver->moveHelper(veh, vPos);
    return vNext;
}


SUMOReal
MSCFModel_CC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel) const {
    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    vars->activeController = VehicleVariables::ACC;
    if (vars->activeController != VehicleVariables::DRIVER)
        return _v(veh, gap2pred, speed, predSpeed, desiredSpeed(veh), true);
    else
        return myHumanDriver->followSpeed(veh, speed, gap2pred, predSpeed, predMaxDecel);
}


SUMOReal
MSCFModel_CC::stopSpeed(const MSVehicle* const veh, SUMOReal gap2pred) const {
    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != VehicleVariables::DRIVER)
    {
        if (gap2pred<0.01) {
            return 0;
        }
        return _v(veh, gap2pred, veh->getSpeed(), 0, desiredSpeed(veh), false);
    }
    else
        return myHumanDriver->stopSpeed(veh, gap2pred);
}

/// @todo update interactionGap logic to IDM
SUMOReal
MSCFModel_CC::interactionGap(const MSVehicle* const veh, SUMOReal vL) const {

    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != VehicleVariables::DRIVER)
    {
        //TODO: understand this. for now set maximum radar range
        return 250;
    }
    else
        return myHumanDriver->interactionGap(veh, vL);

}

SUMOReal
MSCFModel_CC::_v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal desSpeed, bool invokedFromFollowSpeed) const {

    std::stringstream debugstr;
    debugstr.precision(2);

    //acceleration computed by the controller
    double controllerAcceleration;
    //acceleration actually actuated by the engine
    double engineAcceleration;
    //speed computed by the model
    double speed;

    bool debug = false;

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    //TODO: only for testing
    //-----------------------------------------------------------
    std::string id = veh->getID();
    if (id.substr(id.size() - 2, 2).compare(".0") == 0) {
        vars->activeController = VehicleVariables::ACC;
        if (MSNet::getInstance()->getCurrentTimeStep() < 120000) {
            vars->ccDesiredSpeed = 36;
        } else {
            vars->ccDesiredSpeed = 25;
        }
    } else {
        vars->activeController = VehicleVariables::CACC;
    }
    //-----------------------------------------------------------

    //has this function already been invoked for this timestep?
    if (vars->lastUpdate != MSNet::getInstance()->getCurrentTimeStep() && invokedFromFollowSpeed) {
        debug = true;
        //relative speed at time now - 1
        double dv_t0 = vars->egoPreviousSpeed - vars->frontSpeed;
        //relative speed at time now
        double dv_t1 = egoSpeed - predSpeed;
        vars->egoSpeed = egoSpeed;
        vars->egoAcceleration = SPEED2ACCEL(veh->getSpeed() - vars->egoPreviousSpeed);
        vars->egoPreviousSpeed = veh->getSpeed();
        vars->lastUpdate = MSNet::getInstance()->getCurrentTimeStep();
        vars->frontAcceleration = vars->egoAcceleration - SPEED2ACCEL(dv_t1 - dv_t0);
        vars->frontSpeed = predSpeed;
    }

    debugstr << "V " << id;

    switch (vars->activeController) {

        case VehicleVariables::ACC:

            debugstr << " uses CC";

            controllerAcceleration = fmin(_cc(egoSpeed, vars->ccDesiredSpeed), _acc(egoSpeed, predSpeed, gap2pred));

            break;

        case VehicleVariables::CACC:

            debugstr << " uses CACC";

            double predAcceleration, leaderAcceleration, leaderSpeed;

            if (invokedFromFollowSpeed) {
                predAcceleration = vars->frontAcceleration;
                leaderAcceleration = vars->leaderAcceleration;
                leaderSpeed = vars->leaderSpeed;
            }
            else {
                /* if the method has not been invoked from followSpeed() then it has been
                 * invoked from stopSpeed(). In such case we set all parameters of preceding
                 * vehicles as they were non-moving obstacles
                 */
                predAcceleration = 0;
                leaderAcceleration = 0;
                leaderSpeed = 0;
            }

            //TODO: again modify probably range/range-rate controller is needed
            controllerAcceleration = _cacc(egoSpeed, predSpeed, predAcceleration, gap2pred, leaderSpeed, leaderAcceleration);

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

    debugstr << " accel=" << controllerAcceleration;

    //compute the actual acceleration applied by the engine
    engineAcceleration = _actuator(controllerAcceleration, vars->egoAcceleration);

    debugstr << " actuators=" << engineAcceleration;

    //compute the speed from the actual acceleration
    speed = egoSpeed + ACCEL2SPEED(engineAcceleration);

    debugstr << " speed=" << speed << "";

    debugstr << " distance=" << gap2pred;

    if (debug)
        WRITE_MESSAGE(debugstr.str());

    return MAX2(SUMOReal(0), speed);
}

SUMOReal
MSCFModel_CC::_cc(SUMOReal egoSpeed, SUMOReal desSpeed) const {

    //Eq. 5.5 of the Rajamani book, with Ki = 0 and bounds on max and min acceleration
    return fmin(myAccel, fmax(-myCcDecel, -myKp * (egoSpeed - desSpeed)));

}

SUMOReal
MSCFModel_CC::_acc(SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal gap2pred) const {

    //Eq. 6.18 of the Rajamani book
    return fmin(myAccel, fmax(-myDecel, -1.0 / myHeadwayTime * (egoSpeed - predSpeed + myLambda * (-gap2pred + myHeadwayTime * egoSpeed))));

}

SUMOReal
MSCFModel_CC::_cacc(SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal predAcceleration, SUMOReal gap2pred, SUMOReal leaderSpeed, SUMOReal leaderAcceleration) const {

    //compute epsilon, i.e., the desired distance error
    double epsilon = -gap2pred + myConstantSpacing; //NOTICE: error (if any) should already be included in gap2pred
    //compute epsilon_dot, i.e., the desired speed error
    double epsilon_dot = egoSpeed - predSpeed;
    //Eq. 7.39 of the Rajamani book
    return fmin(myAccel, fmax(-myDecel, myAlpha1 * predAcceleration + myAlpha2 * leaderAcceleration +
                              myAlpha3 * epsilon_dot + myAlpha4 * (egoSpeed - leaderSpeed) + myAlpha5 * epsilon));

}

SUMOReal
MSCFModel_CC::_actuator(SUMOReal acceleration, SUMOReal currentAcceleration) const {

    //standard low-pass filter discrete implementation
    return myAlpha * acceleration + myOneMinusAlpha * currentAcceleration;

}

void
MSCFModel_CC::setCCDesiredSpeed(const MSVehicle* veh, SUMOReal ccDesiredSpeed) {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->ccDesiredSpeed = ccDesiredSpeed;
}

void
MSCFModel_CC::setLeaderInformation(const MSVehicle* veh, SUMOReal speed, SUMOReal acceleration) {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    std::stringstream debug;
    debug.precision(2);
    debug << veh->getID() << " updates leader info: v=" << speed << " a=" << acceleration;
    WRITE_ERROR(debug.str());
    vars->leaderAcceleration = acceleration;
    vars->leaderSpeed = speed;
}

void
MSCFModel_CC::getVehicleInformation(const MSVehicle* veh, SUMOReal& speed, SUMOReal& acceleration) {
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
