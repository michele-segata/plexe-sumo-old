/****************************************************************************/
/// @file    MSCACCLaneChanger.h
/// @author  Michele Segata
/// @date    Fri, 04 Mar 2012
/// @version $Id: MSCACCLaneChanger.h $
///
// Performs lane changing of vehicles, taking into account the presence of
// platooning vehicles and a dedicated lane for platooning
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

#include "MSCACCLaneChanger.h"
#include "MSVehicle.h"
#include "MSVehicleType.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include <cassert>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <microsim/MSAbstractLaneChangeModel.h>
#include <utils/common/MsgHandler.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

//#define DEBUG_VEHICLE_GUI_SELECTION 1
#ifdef DEBUG_VEHICLE_GUI_SELECTION
#include <utils/gui/div/GUIGlobalSelection.h>
#include <guisim/GUIVehicle.h>
#include <guisim/GUILane.h>
#endif

MSCACCLaneChanger::MSCACCLaneChanger(std::vector<MSLane*>* lanes, bool allowSwap) :
    MSLaneChanger(lanes, allowSwap) {
}

MSCACCLaneChanger::~MSCACCLaneChanger() {
}

enum Plexe::PLATOONING_LANE_CHANGE_ACTION MSCACCLaneChanger::getLaneChangeAction(MSVehicle* vehicle) {

    const MSCFModel_CC *model = dynamic_cast<const MSCFModel_CC *>(&vehicle->getCarFollowModel());

    //if the car is not CACC enabled, then we just let the default lane change model to take control
    if (!model)
        return Plexe::DRIVER_CHOICE;
    else
        return model->getLaneChangeAction(vehicle);

}

void MSCACCLaneChanger::setLaneChangeAction(MSVehicle* vehicle, enum Plexe::PLATOONING_LANE_CHANGE_ACTION action) {

    const MSCFModel_CC *model = dynamic_cast<const MSCFModel_CC *>(&vehicle->getCarFollowModel());

    assert(model);
    model->setLaneChangeAction(vehicle, action);

}

bool MSCACCLaneChanger::change() {

    //the code has been copied and adapted from MSLaneChanger::change()

    // Find change-candidate. If it is on an allowed lane, try to change
    // to the right (there is a rule in Germany that you have to change
    // to the right, unless you are overtaking). If change to the right
    // isn't possible, check if there is a possibility to overtake (on the
    // left.
    // If candidate isn't on an allowed lane, changing to an allowed has
    // priority.
    myCandi = findCandidate();
    MSVehicle* vehicle = veh(myCandi);
#ifdef DEBUG_VEHICLE_GUI_SELECTION
    if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(vehicle)->getGlID())) {
        int bla = 0;
    }
#endif
    if (vehicle->getLane() != (*myCandi).lane || vehicle->getLaneChangeModel().isChangingLanes()) {
        // vehicles shadows and changing vehicles are not eligible
        registerUnchanged(vehicle);
        return false;
    }
#ifndef NO_TRACI
    if (vehicle->hasInfluencer() && vehicle->getInfluencer().isVTDControlled()) {
        return false; // !!! temporary; just because it broke, here
    }
#endif
    const std::vector<MSVehicle::LaneQ> &preb = vehicle->getBestLanes();
    assert(preb.size() == myChanger.size());
    for (int i = 0; i < (int) myChanger.size(); ++i) {
        ((std::vector<MSVehicle::LaneQ>&) preb)[i].occupation = myChanger[i].dens + preb[i].nextOccupation;
    }

    vehicle->getLaneChangeModel().prepareStep();
    std::pair<MSVehicle* const, SUMOReal> leader = getRealThisLeader(myCandi);

    enum Plexe::PLATOONING_LANE_CHANGE_ACTION laneChangeAction = getLaneChangeAction(vehicle);

    MSCFModel_CC::VehicleVariables *vars = (MSCFModel_CC::VehicleVariables *)vehicle->getCarFollowVariables();

    //first of all: check for the requested action: if it has been fulfilled then change it
    switch (laneChangeAction) {

    case Plexe::MOVE_TO_FIXED_LANE: {

        //see below for an explanation of this offset variable
        int offset = vehicle->getEdge()->getLanes().size() - ((const MSCFModel_CC&)vehicle->getCarFollowModel()).getMyLanesCount();
        if (vehicle->getLaneIndex() == vars->fixedLane + offset) {
            setLaneChangeAction(vehicle, Plexe::STAY_IN_CURRENT_LANE);
        }

        break;

    }

    case Plexe::STAY_IN_CURRENT_LANE: {

        //if we are in the lane we want to stay in, then there is nothing the car wants to do
        vehicle->getLaneChangeModel().setOwnState(LCA_NONE);

        break;

    }

    default:
        break;

    }

    if (laneChangeAction == Plexe::MOVE_TO_FIXED_LANE) {

        int destination;
        int state = 0;

        /**
         * compute the lane index offset. If the highway in the scenario has 4 lanes, and the simulation requires the car
         * to move to lane index 1, we need to check whether we are in a part of the highway where there are really 4 lanes,
         * or if there are more due, for example, to an off-ramp. If there is a lane for the off-ramp, then the number of
         * lanes in such edge will be 5. We need to increment the index of the destination lane of (5-4)=1. So the car will
         * move to lane index 2, on a total number of lanes of 5. When changing edge and coming back to 4 lanes, then the car
         * will automatically be in the right lane (index 1)
         */
        int offset = vehicle->getEdge()->getLanes().size() - ((const MSCFModel_CC&)vehicle->getCarFollowModel()).getMyLanesCount();

        //compute where we want to go
        destination = vars->fixedLane + offset;
        /**
         * a negative destination could happen when the scenario changes the number of lane in the highway, decreasing it.
         * For example, if the highway has in general 4 lanes, and then switches to 3, and the simulator requires the car
         * to move to lane index=0 in the 4-lane edge, then the car should continue on lane index=-1 in the 3-lane edge,
         * which can't obviously be done
         */
        assert(destination < vehicle->getEdge()->getLanes().size() && destination >= 0);

        //compute the difference between where we are and where we are heading at
        int currentToDestination = vehicle->getLaneIndex() - destination;

        if (currentToDestination < 0) {

            //we need to move to the left

            // check whether the vehicle wants and is able to change to left lane
            int blockedCheck = 0;
            if ((myCandi + 1) != myChanger.end() && (myCandi + 1)->lane->allowsVehicleClass(veh(myCandi)->getVehicleType().getVehicleClass())) {

                std::pair<MSVehicle* const, SUMOReal> lLead = getRealLeader(myCandi + 1);
                std::pair<MSVehicle* const, SUMOReal> lFollow = getRealFollower(myCandi + 1);
                //TODO: debug this for make it work with different situations
                state = LCA_SPEEDGAIN | LCA_LEFT;

                //tell to the car following model to ignore the modifications. we are about to call the followSpeed() method
                //only for knowing whether we might get a benefit from changing lane
                const MSCFModel_CC *model = dynamic_cast<const MSCFModel_CC *>(&vehicle->getCarFollowModel());
                model->setIgnoreModifications(vehicle, true);

                //the change2left method tells us whether we have a vehicle on the left blocking the way so we can avoid collisions
                blockedCheck = change2left(leader, lLead, lFollow,preb);

                //disable ignore modification from now on. follow speed will not be called
                model->setIgnoreModifications(vehicle, false);

                bool changingAllowed = (blockedCheck & LCA_BLOCKED) == 0;

                //vehicle->getLaneChangeModel().setOwnState(state2|state1);
                // change if the vehicle wants to and is allowed to change
                if (changingAllowed) {
#ifndef NO_TRACI
                    // inform lane change model about this change
                    vehicle->getLaneChangeModel().fulfillChangeRequest(MSVehicle::REQUEST_LEFT);
#endif
                    startChange(vehicle, myCandi, 1);
                    return true;
                }
                if ((state & LCA_LEFT) != 0 && (state & LCA_URGENT) != 0) {
                    (myCandi + 1)->lastBlocked = vehicle;
                }
            }
            vehicle->getLaneChangeModel().setOwnState(state);

        }

        if (currentToDestination > 0) {

            //we need to move to the right

            // check whether the vehicle wants and is able to change to right lane
            int blockedCheck = 0;
            if ((myCandi - 1)->lane->allowsVehicleClass(veh(myCandi)->getVehicleType().getVehicleClass())) {

                std::pair<MSVehicle* const, SUMOReal> lLead = getRealLeader(myCandi - 1);
                std::pair<MSVehicle* const, SUMOReal> lFollow = getRealFollower(myCandi - 1);
                //TODO: debug this for make it work with different situations
                state = LCA_SPEEDGAIN | LCA_RIGHT;

                //tell to the car following model to ignore the modifications. we are about to call the followSpeed() method
                //only for knowing whether we might get a benefit from changing lane
                const MSCFModel_CC *model = dynamic_cast<const MSCFModel_CC *>(&vehicle->getCarFollowModel());
                model->setIgnoreModifications(vehicle, true);

                //the change2left method tells us whether we have a vehicle on the right blocking the way so we can avoid collisions
                blockedCheck = change2right(leader, lLead, lFollow,preb);
                //disable ignore modification from now on. follow speed will not be called
                model->setIgnoreModifications(vehicle, false);

                bool changingAllowed2 = (blockedCheck & LCA_BLOCKED) == 0;

                //vehicle->getLaneChangeModel().setOwnState(state2|state1);
                // change if the vehicle wants to and is allowed to change
                if (changingAllowed2) {
#ifndef NO_TRACI
                    // inform lane change model about this change
                    vehicle->getLaneChangeModel().fulfillChangeRequest(MSVehicle::REQUEST_RIGHT);
#endif
                    startChange(vehicle, myCandi, -1);
                    return true;
                }
                if ((state & LCA_RIGHT) != 0 && (state & LCA_URGENT) != 0) {
                    (myCandi - 1)->lastBlocked = vehicle;
                }
            }
            vehicle->getLaneChangeModel().setOwnState(state);

        }

        // check whether the vehicles should be swapped
        if (myAllowsSwap && (state & (LCA_URGENT)) != 0) {
            // get the direction ...
            ChangerIt target;
            int dir;
            if ((state & (LCA_URGENT)) != 0) {
                if (state & LCA_LEFT) {
                    // ... wants to go left
                    target = myCandi + 1;
                    dir = 1;
                }
                else {
                    // ... wants to go right
                    target = myCandi - 1;
                    dir = -1;
                }
            }
            MSVehicle* prohibitor = target->lead;
            if (target->hoppedVeh != 0) {
                SUMOReal hoppedPos = target->hoppedVeh->getPositionOnLane();
                if (prohibitor == 0 || (hoppedPos > vehicle->getPositionOnLane() && prohibitor->getPositionOnLane() > hoppedPos)) {
                    prohibitor = 0; // !!! vehicles should not jump over more than one lanetarget->hoppedVeh;
                }
            }
            if (prohibitor != 0 && ((prohibitor->getLaneChangeModel().getOwnState() & (LCA_URGENT/*|LCA_SPEEDGAIN*/)) != 0 && (prohibitor->getLaneChangeModel().getOwnState() & (LCA_LEFT | LCA_RIGHT)) != (vehicle->getLaneChangeModel().getOwnState() & (LCA_LEFT | LCA_RIGHT)))) {

                // check for position and speed
                if (prohibitor->getVehicleType().getLengthWithGap() - vehicle->getVehicleType().getLengthWithGap() == 0) {
                    // ok, may be swapped
                    // remove vehicle to swap with
                    MSLane::VehCont::iterator i = find(target->lane->myTmpVehicles.begin(), target->lane->myTmpVehicles.end(), prohibitor);
                    if (i != target->lane->myTmpVehicles.end()) {
                        MSVehicle* bla = *i;
                        assert(bla == prohibitor);
                        target->lane->myTmpVehicles.erase(i);
                        // set this vehicle
                        target->hoppedVeh = vehicle;
                        target->lane->myTmpVehicles.insert(target->lane->myTmpVehicles.begin(), vehicle);
                        myCandi->hoppedVeh = prohibitor;
                        myCandi->lane->myTmpVehicles.insert(target->lane->myTmpVehicles.begin(), prohibitor);

                        // leave lane and detectors
                        vehicle->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                        prohibitor->leaveLane(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
                        // patch position and speed
                        SUMOReal p1 = vehicle->getPositionOnLane();
                        vehicle->myState.myPos = prohibitor->myState.myPos;
                        prohibitor->myState.myPos = p1;
                        p1 = vehicle->getSpeed();
                        vehicle->myState.mySpeed = prohibitor->myState.mySpeed;
                        prohibitor->myState.mySpeed = p1;
                        // enter lane and detectors
                        vehicle->enterLaneAtLaneChange(target->lane);
                        prohibitor->enterLaneAtLaneChange(myCandi->lane);
                        // mark lane change
                        vehicle->getLaneChangeModel().changed();
                        prohibitor->getLaneChangeModel().changed();
                        (myCandi)->dens += prohibitor->getVehicleType().getLengthWithGap();
                        (target)->dens += vehicle->getVehicleType().getLengthWithGap();
                        return true;
                    }
                }
            }
        }

    }

    //if the action must not be chosen by the driver, and we arrive at this
    //point, then no lane change must be done
    if (laneChangeAction != Plexe::DRIVER_CHOICE) {
        registerUnchanged(vehicle);
        return false;
    }
    else {
        //if we get here, lane changing for platooning has not been requested. invoke normal lane changing
        return MSLaneChanger::change();
    }

}

int
MSCACCLaneChanger::change2left(const std::pair<MSVehicle* const, SUMOReal>& leader,
                               const std::pair<MSVehicle* const, SUMOReal>& rLead,
                               const std::pair<MSVehicle* const, SUMOReal>& rFollow,
                               const std::vector<MSVehicle::LaneQ>& preb) const {
    ChangerIt target = myCandi + 1;
    int blocked = overlapWithHopped(target)
                  ? target->hoppedVeh->getPositionOnLane() < veh(myCandi)->getPositionOnLane()
                  ? LCA_BLOCKED_BY_LEFT_FOLLOWER
                  : LCA_BLOCKED_BY_LEFT_LEADER
                  : 0;
    // overlap
    if (rFollow.first != 0 && rFollow.second < 0) {
        blocked |= (LCA_BLOCKED_BY_LEFT_FOLLOWER);
    }
    if (rLead.first != 0 && rLead.second < 0) {
        blocked |= (LCA_BLOCKED_BY_LEFT_LEADER);
    }
    // safe back gap
    if (rFollow.first != 0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        const MSCFModel& carFollowModel = rFollow.first->getCarFollowModel();
        if (carFollowModel.getModelID() == SUMO_TAG_CF_CC) {
            //the car is controlled by MSCFModel_CC
            const MSCFModel_CC& ccCarFollowModel = static_cast<const MSCFModel_CC &>(carFollowModel);
            Plexe::ACTIVE_CONTROLLER controller = ccCarFollowModel.getActiveController(rFollow.first);

            if (rFollow.second < rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), veh(myCandi)->getSpeed(), veh(myCandi)->getCarFollowModel().getMaxDecel())) {
                //if the gap is not enough to safely change lane
                if (controller == Plexe::DRIVER || controller == Plexe::ACC) {
                    //if the active controller is either a human, or an ACC, then the movement must be blocked
                    blocked |= LCA_BLOCKED_BY_LEFT_FOLLOWER;
                }
                else {
                    //otherwise the movement must be blocked only if the relative speed is greater than a certain threshold
                    if (rFollow.first->getSpeed() - veh(myCandi)->getSpeed() > 3) {
                        blocked |= LCA_BLOCKED_BY_LEFT_FOLLOWER;
                    }
                }
            }

        }
        else {
            if (rFollow.second < rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), veh(myCandi)->getSpeed(), veh(myCandi)->getCarFollowModel().getMaxDecel())) {
                blocked |= LCA_BLOCKED_BY_LEFT_FOLLOWER;
            }
        }
    }
    // safe front gap
    if (rLead.first != 0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rLead.second < veh(myCandi)->getCarFollowModel().getSecureGap(veh(myCandi)->getSpeed(), rLead.first->getSpeed(), rLead.first->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_LEFT_LEADER;
        }
    }
    MSAbstractLaneChangeModel::MSLCMessager msg(leader.first, rLead.first, rFollow.first);
    return blocked | veh(myCandi)->getLaneChangeModel().wantsChangeToLeft(
               msg, blocked, leader, rLead, rFollow, *(myCandi + 1)->lane, preb, &(myCandi->lastBlocked));
}

int
MSCACCLaneChanger::change2right(const std::pair<MSVehicle* const, SUMOReal>& leader,
                                const std::pair<MSVehicle* const, SUMOReal>& rLead,
                                const std::pair<MSVehicle* const, SUMOReal>& rFollow,
                                const std::vector<MSVehicle::LaneQ>& preb) const {
    ChangerIt target = myCandi - 1;
    int blocked = overlapWithHopped(target)
                  ? target->hoppedVeh->getPositionOnLane() < veh(myCandi)->getPositionOnLane()
                  ? LCA_BLOCKED_BY_RIGHT_FOLLOWER
                  : LCA_BLOCKED_BY_RIGHT_LEADER
                  : 0;
    // overlap
    if (rFollow.first != 0 && rFollow.second < 0) {
        blocked |= (LCA_BLOCKED_BY_RIGHT_FOLLOWER);
    }
    if (rLead.first != 0 && rLead.second < 0) {
        blocked |= (LCA_BLOCKED_BY_RIGHT_LEADER);
    }
    // safe back gap
    if (rFollow.first != 0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        const MSCFModel& carFollowModel = rFollow.first->getCarFollowModel();
        if (carFollowModel.getModelID() == SUMO_TAG_CF_CC) {
            //the car is controlled by MSCFModel_CC
            const MSCFModel_CC& ccCarFollowModel = static_cast<const MSCFModel_CC &>(carFollowModel);
            Plexe::ACTIVE_CONTROLLER controller = ccCarFollowModel.getActiveController(rFollow.first);

            if (rFollow.second < rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), veh(myCandi)->getSpeed(), veh(myCandi)->getCarFollowModel().getMaxDecel())) {
                //if the gap is not enough to safely change lane
                if (controller == Plexe::DRIVER || controller == Plexe::ACC) {
                    //if the active controller is either a human, or an ACC, then the movement must be blocked
                    blocked |= LCA_BLOCKED_BY_RIGHT_FOLLOWER;
                }
                else {
                    //otherwise the movement must be blocked only if the relative speed is greater than a certain threshold
                    if (rFollow.first->getSpeed() - veh(myCandi)->getSpeed() > 3) {
                        blocked |= LCA_BLOCKED_BY_RIGHT_FOLLOWER;
                    }
                }
            }

        }
        else {
            if (rFollow.second < rFollow.first->getCarFollowModel().getSecureGap(rFollow.first->getSpeed(), veh(myCandi)->getSpeed(), veh(myCandi)->getCarFollowModel().getMaxDecel())) {
                blocked |= LCA_BLOCKED_BY_RIGHT_FOLLOWER;
            }
        }
    }

    // safe front gap
    if (rLead.first != 0) {
        // !!! eigentlich: vsafe braucht die Max. Geschwindigkeit beider Spuren
        if (rLead.second < veh(myCandi)->getCarFollowModel().getSecureGap(veh(myCandi)->getSpeed(), rLead.first->getSpeed(), rLead.first->getCarFollowModel().getMaxDecel())) {
            blocked |= LCA_BLOCKED_BY_RIGHT_LEADER;
        }
    }

    MSAbstractLaneChangeModel::MSLCMessager msg(leader.first, rLead.first, rFollow.first);
    return blocked | veh(myCandi)->getLaneChangeModel().wantsChangeToRight(
               msg, blocked, leader, rLead, rFollow, *(myCandi - 1)->lane, preb, &(myCandi->lastBlocked));
}

void
MSCACCLaneChanger::laneChange(SUMOTime t) {

    //the code has been copied and adapted from MSLaneChanger::laneChange()

    // This is what happens in one timestep. After initialization of the
    // changer, each vehicle will try to change. After that the changer
    // nedds an update to prevent multiple changes of one vehicle.
    // Finally, the change-result has to be given back to the lanes.
    initChanger();
    while (vehInChanger()) {

        bool haveChanged = change();
        updateChanger(haveChanged);
    }
    updateLanes(t);
}
