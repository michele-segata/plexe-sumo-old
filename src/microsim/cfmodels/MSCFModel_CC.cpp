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
// Copyright (C) 2012-2014 Michele Segata (segata@ccs-labs.org)
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
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
#include <string.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_CC::MSCFModel_CC(const MSVehicleType* vtype,
                           SUMOReal accel, SUMOReal decel,
                           SUMOReal ccDecel, SUMOReal headwayTime, SUMOReal constantSpacing,
                           SUMOReal kp, SUMOReal lambda, SUMOReal c1, SUMOReal xi,
                           SUMOReal omegaN, SUMOReal tau, int lanesCount, SUMOReal ccAccel)
    : MSCFModel(vtype, accel, decel, headwayTime), myCcDecel(ccDecel), myConstantSpacing(constantSpacing)
    , myKp(kp), myLambda(lambda), myC1(c1), myXi(xi), myOmegaN(omegaN), myTau(tau), myAlpha1(1 - myC1), myAlpha2(myC1),
    myAlpha3(-(2 * myXi - myC1 *(myXi + sqrt(myXi* myXi - 1))) * myOmegaN), myAlpha4(-(myXi + sqrt(myXi* myXi - 1)) * myOmegaN* myC1),
    myAlpha5(-myOmegaN* myOmegaN), myAlpha(TS / (myTau + TS)), myOneMinusAlpha(1 - myAlpha), myLanesCount(lanesCount), myCcAccel(ccAccel) {

    //if the lanes count has not been specified in the attributes of the model, lane changing cannot properly work
    if (lanesCount == -1) {
        std::cerr << "The number of lanes needs to be specified in the attributes of carFollowing-CC with the \"lanesCount\" attribute\n";
        WRITE_ERROR("The number of lanes needs to be specified in the attributes of carFollowing-CC with the \"lanesCount\" attribute");
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

    if (vars->activeController != Plexe::DRIVER) {
        //if during this simulation step the speed has been set by the followSpeed() method, then use such value
        if (vars->followSpeedSetTime == MSNet::getInstance()->getCurrentTimeStep()) {
            vNext = vars->controllerFollowSpeed;
            vars->controllerAcceleration = vars->followControllerAcceleration;
            vars->accAcceleration = vars->followAccAcceleration;
        }
        //otherwise use the value set by the freeSpeed() method
        else {
            vNext = vars->controllerFreeSpeed;
            vars->controllerAcceleration = vars->freeControllerAcceleration;
            vars->accAcceleration = vars->freeAccAcceleration;
        }
    }
    else
        vNext = myHumanDriver->moveHelper(veh, vPos);

    //update ego data

    //this method should be called only once per vehicle per simulation step. is it true?
    assert(vars->egoDataLastUpdate != MSNet::getInstance()->getCurrentTimeStep());

    vars->egoPreviousSpeed = vars->egoSpeed;
    vars->egoSpeed = vNext;
    //value for acceleration must be checked as it might create problems. for example, if in the .rou.xml
    //file an initial speed is set, since egoPreviousSpeed will be zero, the initial acceleration will
    //be huge. such acceleration, will be "remembered" by the low pass filtering mechanism implementing
    //the actuator, so the cars will be wrongly accelerating (might reach more than 300km/h) until the
    //low pass filter will let the acceleration go down to a normal value.
    //so here we check whether computed acceleration has a reasonable value. if not just set it to 0
    double potentialAcceleration = SPEED2ACCEL(vars->egoSpeed - vars->egoPreviousSpeed);
    vars->egoAcceleration = potentialAcceleration < 5 ? potentialAcceleration : 0;
    vars->egoDataLastUpdate = MSNet::getInstance()->getCurrentTimeStep();

    return vNext;
}


SUMOReal
MSCFModel_CC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel) const {

    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();

    //has this function already been invoked for this timestep?
    if (vars->radarLastUpdate != MSNet::getInstance()->getCurrentTimeStep() && !vars->ignoreModifications) {
//        //relative speed at time now - 1
//        double dv_t0 = vars->egoPreviousSpeed - vars->frontSpeed;
//        //relative speed at time now
//        double dv_t1 = speed - predSpeed;

        vars->radarLastUpdate = MSNet::getInstance()->getCurrentTimeStep();
//        vars->frontAcceleration = vars->egoAcceleration - SPEED2ACCEL(dv_t1 - dv_t0);
//        //if we receive the first update of the simulation about the vehicle in front, we might save a wrong acceleration
//        //value, because we had no previous speed value, so we might save an acceleration of more than 10g for example
//        //so we can just set it to 0. at the next time step (100ms) it will be correctly updated
//        if (vars->frontAcceleration > 10 || vars->frontAcceleration < -10)
//            vars->frontAcceleration = 0;
        vars->radarFrontSpeed = predSpeed;
        vars->radarFrontDistance = gap2pred;
    }

    if (vars->activeController != Plexe::DRIVER) {
        return _v(veh, gap2pred, speed, predSpeed, desiredSpeed(veh), MSCFModel_CC::FOLLOW_SPEED);
    }
    else
        return myHumanDriver->followSpeed(veh, speed, gap2pred, predSpeed, predMaxDecel);
}


SUMOReal
MSCFModel_CC::stopSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred) const {

    /**
     * This SUMO method is meant for asking the car what speed the driver would
     * apply if it would have to stop in gap2pred meters. This method is called
     * by the SUMO engine when a approaching a junction, asking something like
     * "what would you do if...?". So this method should return the current speed
     * if the model thinks that there is more than enough space to stop,
     * otherwise it should return a reduced speed. SUMO, then, will use the
     * reduced speed in two cases:
     * 1) there is the real need for using that: for example, we are approaching
     * an intersection where the traffic light is red
     * 2) we are very far from the intersection: in this case SUMO still does not
     * know whether the car will have to stop (e.g., it does not know if the
     * traffic light will be red or not), so in this case SUMO decides to use the
     * reduced speed, because IF the car will have to stop, by using this reduced
     * speed it will be able to do that.
     *
     * Point number 2) causes a problem with ACC: the reaction of the ACC to a
     * completely stopped obstacle in front (this is how a red traffic light is
     * actually modeled) is applying a huge deceleration which, combined with all
     * the really complicated logics behind SUMO, makes the car stop at every
     * junction, even if this is a simple connection between two edges.
     * For making this work there is an easy and logic workaround: if the CC
     * is switched on, then we just return the current speed. This is actually
     * correct: the radar is only able to detect real obstacles, and not
     * intersections, so in reality the ACC controller will just let the car
     * go. It is a duty of the driver to switch the CC off before an intersection.
     *
     * If the CC is switched off, then we just call the stopSpeed method of the
     * installed carFollowing model simulating the real driver.
     *
     * EVEN WORSE: by simply returning the current speed you might never accelerate
     * when needed. So when the CC is enabled we just return a really high value
     */

    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != Plexe::DRIVER)
    {
        return 1e6;
    }
    else {
        return myHumanDriver->stopSpeed(veh, speed, gap2pred);
    }
}

SUMOReal MSCFModel_CC::freeSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal seen, SUMOReal maxSpeed, const bool onInsertion) const {
    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != Plexe::DRIVER)
    {
        return _v(veh, seen, speed, maxSpeed, desiredSpeed(veh), MSCFModel_CC::FREE_SPEED);
    }
    else {
        return MSCFModel::freeSpeed(veh, speed, seen, maxSpeed, onInsertion);
    }
}

SUMOReal
MSCFModel_CC::interactionGap(const MSVehicle* const veh, SUMOReal vL) const {

    VehicleVariables *vars = (VehicleVariables *)veh->getCarFollowVariables();
    if (vars->activeController != Plexe::DRIVER)
    {
        //maximum radar range is CC is enabled
        return 250;
    }
    else {
        return myHumanDriver->interactionGap(veh, vL);
    }

}

SUMOReal
MSCFModel_CC::maxNextSpeed(SUMOReal speed) const {
    return speed + (SUMOReal) ACCEL2SPEED(getMaxAccel());
}

SUMOReal
MSCFModel_CC::_v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal desSpeed, enum CONTROLLER_INVOKER invoker) const {

    //acceleration computed by the controller
    double controllerAcceleration;
    //acceleration actually actuated by the engine
    double engineAcceleration;
    //speed computed by the model
    double speed;
    //acceleration computed by the Cruise Control
    double ccAcceleration;
    //acceleration computed by the Adaptive Cruise Control
    double accAcceleration;
    //acceleration computed by the Cooperative Adaptive Cruise Control
    double caccAcceleration;
    //variables needed by CACC
    double predAcceleration, leaderAcceleration, leaderSpeed;

    bool debug = true;

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    if (vars->activeController != Plexe::DRIVER && vars->useFixedAcceleration) {
        controllerAcceleration = vars->fixedAcceleration;
    }
    else {

        switch (vars->activeController) {

            case Plexe::ACC:

                ccAcceleration = _cc(veh, egoSpeed, vars->ccDesiredSpeed);
                accAcceleration = _acc(veh, egoSpeed, predSpeed, gap2pred, vars->accHeadwayTime);

                if (gap2pred > 250 || ccAcceleration < accAcceleration) {
                    controllerAcceleration = ccAcceleration;
                }
                else {
                    controllerAcceleration = accAcceleration;
                }


                break;

            case Plexe::CACC:

                if (invoker == MSCFModel_CC::FOLLOW_SPEED) {
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
                ccAcceleration = _cc(veh, egoSpeed, vars->ccDesiredSpeed);
                caccAcceleration = _cacc(veh, egoSpeed, predSpeed, predAcceleration, gap2pred, leaderSpeed, leaderAcceleration, vars->caccSpacing);
                //if CACC is enabled and we are closer than 20 meters, let it decide
                if (gap2pred < 20) {
                    controllerAcceleration = caccAcceleration;
                }
                else {
                    controllerAcceleration = fmin(ccAcceleration, caccAcceleration);
                }

                break;

            case Plexe::FAKED_CACC:

                if (invoker == MSCFModel_CC::FOLLOW_SPEED) {
                    //compute ACC acceleration that will be then used to check for vehicles in front
                    vars->followAccAcceleration = _acc(veh, egoSpeed, predSpeed, gap2pred, vars->accHeadwayTime);
                }
                else {
                    //compute ACC acceleration that will be then used to check for vehicles in front
                    vars->freeAccAcceleration = _acc(veh, egoSpeed, predSpeed, gap2pred, vars->accHeadwayTime);
                }

                ccAcceleration = _cc(veh, egoSpeed, vars->ccDesiredSpeed);
                caccAcceleration = _cacc(veh, egoSpeed, vars->fakeData.frontSpeed, vars->fakeData.frontAcceleration, vars->fakeData.frontDistance, vars->fakeData.leaderSpeed, vars->fakeData.leaderAcceleration, vars->caccSpacing);
                controllerAcceleration = fmin(ccAcceleration, caccAcceleration);

                break;

            case Plexe::MYCC:

                ccAcceleration = _cc(veh, egoSpeed, vars->ccDesiredSpeed);
                caccAcceleration = _mycc(veh, egoSpeed, vars->frontSpeed, gap2pred);
                controllerAcceleration = fmin(ccAcceleration, caccAcceleration);
                break;

            case Plexe::DRIVER:

                std::cerr << "Switching to normal driver behavior still not implemented in MSCFModel_CC\n";
                assert(false);
                break;

            default:

                std::cerr << "Invalid controller selected in MSCFModel_CC\n";
                assert(false);
                break;

        }

    }

    //compute the actual acceleration applied by the engine
    engineAcceleration = _actuator(veh, controllerAcceleration, vars->egoAcceleration);

    //compute the speed from the actual acceleration
    speed = MAX2(SUMOReal(0), egoSpeed + ACCEL2SPEED(engineAcceleration));

    //if we have to ignore modifications (e.g., when this method is invoked by the lane changing logic)
    //DO NOT change the state of the vehicle
    if (!vars->ignoreModifications) {

        if (invoker == MSCFModel_CC::FOLLOW_SPEED && vars->followSpeedSetTime != MSNet::getInstance()->getCurrentTimeStep()) {
            vars->controllerFollowSpeed = speed;
            vars->followSpeedSetTime = MSNet::getInstance()->getCurrentTimeStep();
            vars->followControllerAcceleration = controllerAcceleration;
        }

        if (invoker == MSCFModel_CC::FREE_SPEED) {
            vars->controllerFreeSpeed = speed;
            vars->freeControllerAcceleration = controllerAcceleration;
        }

    }

    return speed;
}

SUMOReal
MSCFModel_CC::_cc(const MSVehicle *veh, SUMOReal egoSpeed, SUMOReal desSpeed) const {

    //Eq. 5.5 of the Rajamani book, with Ki = 0 and bounds on max and min acceleration
    return fmin(myCcAccel, fmax(-myCcDecel, -myKp * (egoSpeed - desSpeed)));

}

SUMOReal
MSCFModel_CC::_acc(const MSVehicle *veh, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal gap2pred, SUMOReal headwayTime) const {

    //Eq. 6.18 of the Rajamani book
    return fmin(myAccel, fmax(-myDecel, -1.0 / headwayTime * (egoSpeed - predSpeed + myLambda * (-gap2pred + headwayTime * egoSpeed))));

}

SUMOReal
MSCFModel_CC::_cacc(const MSVehicle *veh, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal predAcceleration, SUMOReal gap2pred, SUMOReal leaderSpeed, SUMOReal leaderAcceleration, SUMOReal spacing) const {

    VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
    //compute epsilon, i.e., the desired distance error
    double epsilon = -gap2pred + spacing; //NOTICE: error (if any) should already be included in gap2pred
    //compute epsilon_dot, i.e., the desired speed error
    double epsilon_dot = egoSpeed - predSpeed;
    //Eq. 7.39 of the Rajamani book
    return fmin(myAccel, fmax(-myDecel, vars->caccAlpha1 * predAcceleration + vars->caccAlpha2 * leaderAcceleration +
                              vars->caccAlpha3 * epsilon_dot + vars->caccAlpha4 * (egoSpeed - leaderSpeed) + vars->caccAlpha5 * epsilon));

}

SUMOReal
MSCFModel_CC::_mycc(const MSVehicle *veh, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal gap2pred) const {
    VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
    return fmin(myAccel, fmax(-myDecel, vars->myccKd * (gap2pred - 25) + vars->myccKs * (predSpeed - egoSpeed)));
}


SUMOReal
MSCFModel_CC::_actuator(const MSVehicle *veh, SUMOReal acceleration, SUMOReal currentAcceleration) const {

    VehicleVariables* vars = (VehicleVariables*)veh->getCarFollowVariables();
    //standard low-pass filter discrete implementation
    return vars->engineAlpha * acceleration + vars->engineOneMinusAlpha * currentAcceleration;

}

void
MSCFModel_CC::setCCDesiredSpeed(const MSVehicle* veh, SUMOReal ccDesiredSpeed) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->ccDesiredSpeed = ccDesiredSpeed;
}

void
MSCFModel_CC::setCACCConstantSpacing(const MSVehicle * veh, SUMOReal spacing) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->caccSpacing = spacing;
}

SUMOReal
MSCFModel_CC::getCACCConstantSpacing(const MSVehicle * veh) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    return vars->caccSpacing;
}

void
MSCFModel_CC::setLeaderInformation(const MSVehicle* veh, SUMOReal speed, SUMOReal acceleration, Position position, SUMOReal time) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->leaderAcceleration = acceleration;
    vars->leaderSpeed = speed;
    vars->leaderPosition = position;
    vars->leaderDataReadTime = time;
}

void
MSCFModel_CC::setPrecedingInformation(const MSVehicle* const veh, SUMOReal speed, SUMOReal acceleration, Position position, SUMOReal time) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->frontAcceleration = acceleration;
    vars->frontSpeed = speed;
    vars->frontDataLastUpdate = MSNet::getInstance()->getCurrentTimeStep();
    vars->frontPosition = position;
    vars->frontDataReadTime = time;
}

void
MSCFModel_CC::getVehicleInformation(const MSVehicle* veh, SUMOReal& speed, SUMOReal& acceleration, SUMOReal& controllerAcceleration, Position &position, SUMOReal &time) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    speed = vars->egoSpeed;
    acceleration = vars->egoAcceleration;
    controllerAcceleration = vars->controllerAcceleration;
    position = veh->getPosition();
    time = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep());
}

void MSCFModel_CC::setGenericInformation(const MSVehicle* veh, const struct Plexe::CCDataHeader &header, const void *content) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    switch (header.type) {
    //TODO: extend
    case CC_SET_VEHICLE_DATA: {
        const struct Plexe::VEHICLE_DATA* vehicle = (const struct Plexe::VEHICLE_DATA*)content;
        vars->vehicles[vehicle->index] = *vehicle;
        break;
    }
    case CC_SET_VEHICLE_POSITION: {
        int *myPosition = (int *)content;
        vars->position = *myPosition;
        break;
    }
    case CC_SET_PLATOON_SIZE: {
        int *nCars = (int*)content;
        vars->nCars = *nCars;
        break;
    }
    case CC_SET_CACC_XI: {
        vars->caccXi = *(double*)content;
        recomputeParameters(veh);
        break;
    }
    case CC_SET_CACC_OMEGA_N: {
        vars->caccOmegaN = *(double*)content;
        recomputeParameters(veh);
        break;
    }
    case CC_SET_CACC_C1: {
        vars->caccC1 = *(double*)content;
        recomputeParameters(veh);
        break;
    }
    case CC_SET_ENGINE_TAU: {
        vars->engineTau = *(double*)content;
        recomputeParameters(veh);
        break;
    }
    case CC_SET_MYCC_KD: {
        vars->myccKd = *(double*)content;
        break;
    }
    case CC_SET_MYCC_KS: {
        vars->myccKs = *(double*)content;
        break;
    }
    default: {
        break;
    }
    }
}

int MSCFModel_CC::getGenericInformation(const MSVehicle *veh, struct Plexe::CCDataHeader request, const void *reqParams, void *content) const {

    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();

    int size;

    switch(request.type) {
    case CC_GET_VEHICLE_DATA: {
        int index = *(int *)reqParams;
        memcpy(content, &vars->vehicles[index], sizeof(struct Plexe::VEHICLE_DATA));
        size = sizeof(struct Plexe::VEHICLE_DATA);
        break;
    }
    default: {
        std::cerr << "Invalid request type in MSCFModel_CC::getGenericInformation()\n";
        assert(false);
        break;
    }
    }

    return size;

}

void MSCFModel_CC::recomputeParameters(const MSVehicle *veh) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->caccAlpha1 = 1 - vars->caccC1;
    vars->caccAlpha2 = vars->caccC1;
    vars->caccAlpha3 = -(2 * vars->caccXi - vars->caccC1 * (vars->caccXi + sqrt(vars->caccXi * vars->caccXi - 1))) * vars->caccOmegaN;
    vars->caccAlpha4 = -(vars->caccXi + sqrt(vars->caccXi* vars->caccXi - 1)) * vars->caccOmegaN * vars->caccC1;
    vars->caccAlpha5 = -vars->caccOmegaN * vars->caccOmegaN;
    vars->engineAlpha = TS / (vars->engineTau + TS);
    vars->engineOneMinusAlpha = 1 - vars->engineAlpha;
}

void MSCFModel_CC::switchOnACC(const MSVehicle *veh, double ccDesiredSpeed)  const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->ccDesiredSpeed = ccDesiredSpeed;
    vars->activeController = Plexe::ACC;
}

void MSCFModel_CC::setACCHeadwayTime(const MSVehicle *veh, double headwayTime) const {
    assert(headwayTime > 0);
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->accHeadwayTime = headwayTime;
}

void MSCFModel_CC::setFixedAcceleration(const MSVehicle *veh, int activate, double acceleration) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->useFixedAcceleration = activate;
    vars->fixedAcceleration = acceleration;
}

void MSCFModel_CC::setActiveController(const MSVehicle *veh, enum Plexe::ACTIVE_CONTROLLER activeController) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->activeController = activeController;
}

enum Plexe::ACTIVE_CONTROLLER MSCFModel_CC::getActiveController(const MSVehicle *veh) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    return vars->activeController;
}

enum Plexe::PLATOONING_LANE_CHANGE_ACTION MSCFModel_CC::getLaneChangeAction(const MSVehicle *veh) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    return vars->laneChangeAction;
}

void MSCFModel_CC::setLaneChangeAction(const MSVehicle *veh, enum Plexe::PLATOONING_LANE_CHANGE_ACTION action) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    vars->laneChangeAction = action;
}

void MSCFModel_CC::setFixedLane(const MSVehicle *veh, int lane) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    //set the lane change only if the index is valid
    if (lane < veh->getEdge()->getLanes().size()) {
        vars->fixedLane = lane;
        vars->laneChangeAction = Plexe::MOVE_TO_FIXED_LANE;
    }
}

void MSCFModel_CC::getRadarMeasurements(const MSVehicle * veh, double &distance, double &relativeSpeed) const {
    VehicleVariables* vars = (VehicleVariables*) veh->getCarFollowVariables();
    if (MSNet::getInstance()->getCurrentTimeStep() == vars->radarLastUpdate + DELTA_T) {
        distance = vars->radarFrontDistance;
        relativeSpeed = vars->radarFrontSpeed - vars->egoSpeed;
    }
    else {
        distance = -1;
        relativeSpeed = 0;
    }
}

void MSCFModel_CC::setControllerFakeData(const MSVehicle *veh, double frontDistance, double frontSpeed, double frontAcceleration,
            double leaderSpeed, double leaderAcceleration) const {

    VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();
    if (frontSpeed >= 0) {
        vars->fakeData.frontAcceleration = frontAcceleration;
        vars->fakeData.frontDistance = frontDistance;
        vars->fakeData.frontSpeed = frontSpeed;
    }
    if (leaderSpeed >= 0) {
        vars->fakeData.leaderAcceleration = leaderAcceleration;
        vars->fakeData.leaderSpeed = leaderSpeed;
    }
}

void MSCFModel_CC::setIgnoreModifications(const MSVehicle *veh, bool ignore) const {
    VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();
    vars->ignoreModifications = ignore;
}

void MSCFModel_CC::setCrashed(const MSVehicle *veh, bool crashed) const {
    VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();
    vars->crashed = crashed;
}

bool MSCFModel_CC::isCrashed(const MSVehicle *veh) const {
    VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();
    return vars->crashed;
}

double MSCFModel_CC::getACCAcceleration(const MSVehicle *veh) const {
    VehicleVariables *vars = (VehicleVariables *) veh->getCarFollowVariables();
    return vars->accAcceleration;
}

int MSCFModel_CC::getMyLanesCount() const {
    return myLanesCount;
}

MSCFModel*
MSCFModel_CC::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_CC(vtype,
                            myAccel, myDecel,
                            myCcDecel, myHeadwayTime, myConstantSpacing,
                            myKp, myLambda, myC1, myXi,
                            myOmegaN, myTau, myLanesCount, myCcAccel);
}
