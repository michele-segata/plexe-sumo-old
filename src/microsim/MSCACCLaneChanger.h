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

#ifndef MSCACCLANECHANGER_H_
#define MSCACCLANECHANGER_H_

// ===========================================================================
// included modules
// ===========================================================================
#include "MSLaneChanger.h"

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSLane.h"
#include "MSEdge.h"
#include <vector>
#include <utils/iodevices/OutputDevice.h>

#include <microsim/cfmodels/MSCFModel_CC.h>

// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCACCLaneChanger
 * @brief Implements a slightly modified lane changing mechanism which takes
 * into account the need for platooning. This mechanism reserves the leftmost
 * lane for automated platoons, while other lanes will be used by normal
 * drivers. This lane changer permits to performs actions such as "move left
 * to join platoon" or "stay in the platoon lane", by overriding the usual
 * lane changing behavior. For normal drivers, the lane changing mechanism
 * remains unchanged
 * @see MSLaneChanger
 */
class MSCACCLaneChanger : public MSLaneChanger  {
public:
    /// Constructor
    MSCACCLaneChanger(std::vector<MSLane*>* lanes, bool allowSwap);

    /// Destructor.
    ~MSCACCLaneChanger();

    /**
     * override original laneChange() method so that the MSCACCLaneChanger::change()
     * method is invoked, instead of the MSLaneChanger::change()
     *
     */
    void laneChange(SUMOTime t);

    /**
     * Override original change2left method. This way different conditions to check
     * whether lane change is blocked or not can be considered. For example, if the
     * car behind is controlled by CACC, then we want to let the car move to the left
     * even if there is a short gap, for example for joining. When the car behind is
     * controlled by and ACC, or by a human, then we want to be more careful before
     * moving in front of it
     */
    int change2left(const std::pair<MSVehicle* const, SUMOReal>& leader, const std::pair<MSVehicle* const, SUMOReal>& rLead, const std::pair<MSVehicle* const, SUMOReal>& rFollow, const std::vector<MSVehicle::LaneQ>& preb) const;

    /**
     * Override original change2right method, as for change2left
     */
    int change2right(const std::pair<MSVehicle* const, SUMOReal>& leader, const std::pair<MSVehicle* const, SUMOReal>& rLead, const std::pair<MSVehicle* const, SUMOReal>& rFollow, const std::vector<MSVehicle::LaneQ>& preb) const;

protected:

    /**
     * override original change() method so that first, request for lane
     * changing for platooning are made, and then, if no request are
     * present, or the car is a normal driver, the superclass change()
     * method is invoked.
     * TODO: test for correct behavior
     *
     */
    bool change();

    /**
     * returns the lane change action to be performed, as given by the CC car following
     * model
     */
    enum MSCFModel_CC::PLATOONING_LANE_CHANGE_ACTION getLaneChangeAction(MSVehicle* vehicle);

    /**
     * set the lane change action. for example, when the user has requested to move
     * to the platooning lane and then, when the car is there, this method can be used
     * to set the action to "STAY_THERE" automatically
     */
    void setLaneChangeAction(MSVehicle* vehicle, enum MSCFModel_CC::PLATOONING_LANE_CHANGE_ACTION action);
};

#endif /* MSCACCLANECHANGER_H_ */
