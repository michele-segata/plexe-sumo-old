/****************************************************************************/
/// @file    CC_VehicleVariables.h
/// @author  Michele Segata
/// @date    Mon, 7 Mar 2016
/// @version $Id: $
///
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2011 DLR (http://www.dlr.de/) and contributors
// Copyright (C) 2012-2016 Michele Segata (segata@ccs-labs.org)
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef CC_VEHICLEVARIABLES_H
#define CC_VEHICLEVARIABLES_H

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "CC_Const.h"
#include <microsim/cfmodels/MSCFModel.h>
#include <utils/geom/Position.h>
#include <string.h>

#include "GenericEngineModel.h"
#include "FirstOrderLagModel.h"
#include "RealisticEngineModel.h"

class CC_VehicleVariables : public MSCFModel::VehicleVariables {
public:

    /**
     * @struct FAKE_CONTROLLER_DATA
     * @brief represent the set of fake data which is sent to the controller in
     * order to automatically make the car move to a precise position before
     * joining the platoon.
     * we expect to get from the upper application the data that the CACC needs, i.e.:
     * - front distance, front speed and front vehicle acceleration: this information
     *   regards the car that the vehicle joining the platoon will have directly in
     *   front. this data might be real or might be fake: for example, if the platoon
     *   management algorithm decides to set the vehicle as the new leader, there won't
     *   be a car in front, and the fake data will be used only for positioning. in the
     *   case of fake data, acceleration must be set to 0
     * - leader front speed and acceleration: this information is the same as previously
     *   described for vehicle in front, but regards the leader. again, if the vehicle
     *   is being set as the new leader, this data might be fake data
     */
    struct FAKE_CONTROLLER_DATA {
        double frontDistance;
        double frontSpeed;
        double frontAcceleration;
        double leaderSpeed;
        double leaderAcceleration;
    };

    /**
     * Topology matrix L for the consensus controller
     */
    const static int defaultL[MAX_N_CARS][MAX_N_CARS];

    /**
     * Gains matrix K for the consensus controller
     */
    const static double defaultK[MAX_N_CARS][MAX_N_CARS];

    /**
     * Default damping ratios vector b for the consensus controller
     */
    const static double defaultB[];

    /**
     * Default time headways vector h for the consensus controller
     */
    const static double defaultH[];

    CC_VehicleVariables() : egoDataLastUpdate(0), egoSpeed(0), egoAcceleration(0), egoPreviousSpeed(0),
        frontDataLastUpdate(0), frontSpeed(0), radarFrontDistance(1000), frontAcceleration(0),
        radarFrontSpeed(0), leaderDataLastUpdate(0), leaderSpeed(0), leaderAcceleration(0),
        platoonId(""), isPlatoonLeader(false), ccDesiredSpeed(14), radarLastUpdate(0), activeController(Plexe::DRIVER),
        laneChangeAction(Plexe::DRIVER_CHOICE), followSpeedSetTime(0), controllerFollowSpeed(0), controllerFreeSpeed(0),
        ignoreModifications(false), fixedLane(-1), accHeadwayTime(1.5), useFixedAcceleration(0), fixedAcceleration(0),
        crashed(false), controllerAcceleration(0), followControllerAcceleration(0), freeControllerAcceleration(0),
        accAcceleration(0), followAccAcceleration(0), freeAccAcceleration(0), caccSpacing(5),
        leaderDataReadTime(0), frontDataReadTime(0), position(-1), nCars(8),
        caccXi(-1), caccOmegaN(-1), caccC1(-1), engineTau(-1), caccAlpha1(-1), caccAlpha2(-1),
        caccAlpha3(-1), caccAlpha4(-1), caccAlpha5(-1), engineAlpha(-1), engineOneMinusAlpha(-1),
        ploegH(0.5), ploegKp(0.2), ploegKd(0.7), nInitialized(0), engine(0),
        frontInitialized(false), leaderInitialized(false), caccInitialized(false) {
        fakeData.frontAcceleration = 0;
        fakeData.frontDistance = 0;
        fakeData.frontSpeed = 0;
        fakeData.leaderAcceleration = 0;
        fakeData.leaderSpeed = 0;
        leaderPosition.set(0, 0);
        frontPosition.set(0, 0);
        //init L, K, b, and h with default values
        memcpy(L, defaultL, sizeof(int)*MAX_N_CARS*MAX_N_CARS);
        memcpy(K, defaultK, sizeof(double)*MAX_N_CARS*MAX_N_CARS);
        memcpy(b, defaultB, sizeof(double)*MAX_N_CARS);
        memcpy(h, defaultH, sizeof(double)*MAX_N_CARS);
        //no data about any vehicle has been set
        for (int i = 0; i < MAX_N_CARS; i++)
            initialized[i] = false;
    }
    ~CC_VehicleVariables() {
        if (engine)
            delete engine;
    }

    /// @brief last time ego data has been updated
    SUMOTime egoDataLastUpdate;
    /// @brief current vehicle speed
    double egoSpeed;
    /// @brief current vehicle acceleration
    double egoAcceleration;
    /// @brief vehicle speed at previous timestep
    double egoPreviousSpeed;
    /// @brief acceleration as computed by the controller, to be sent to other vehicles
    double controllerAcceleration;
    /// @brief acceleration as computed by the controller when followSpeed is invoked
    double followControllerAcceleration;
    /// @brief acceleration as computed by the controller when freeSpeed is invoked
    double freeControllerAcceleration;

    /// @brief last time front vehicle data (speed and acceleration) has been updated
    SUMOTime frontDataLastUpdate;
    /// @brief current front vehicle speed
    double frontSpeed;
    /// @brief current front vehicle acceleration (used by CACC)
    double frontAcceleration;
    /// @brief current front vehicle position
    Position frontPosition;
    /// @brief when front vehicle data has been readed from GPS
    double frontDataReadTime;
    /// @did we receive at least one packet?
    bool frontInitialized;
    /// @brief last timestep at which front vehicle data (distance) has been updated
    SUMOTime radarLastUpdate;
    /// @brief current front vehicle distance as provided by the radar
    double radarFrontDistance;
    /// @brief current front vehicle speed as provided by the radar
    double radarFrontSpeed;

    /// @brief headway time for ACC
    double accHeadwayTime;
    /// @brief fixed spacing for CACC
    double caccSpacing;

    /// @brief last time leader vehicle data has been updated
    SUMOTime leaderDataLastUpdate;
    /// @brief platoon's leader speed (used by CACC)
    double leaderSpeed;
    /// @brief platoon's leader acceleration (used by CACC)
    double leaderAcceleration;
    /// @brief platoon's leader position
    Position leaderPosition;
    /// @brief when leader data has been readed from GPS
    double leaderDataReadTime;
    /// @did we receive at least one packet?
    bool leaderInitialized;
    bool caccInitialized;

    //time at which followSpeed has been invoked. In this way
    //we can tell moveHelper whether controllerFollowSpeed must
    //be used or not, by checking if this value has been set
    //in the current time step
    SUMOTime followSpeedSetTime;
    //speed computed by followSpeed
    double controllerFollowSpeed;
    //speed computed by freeSpeed
    double controllerFreeSpeed;

    //enable/disable the use of a constant, user defined acceleration instead of the one computed by the controller
    int useFixedAcceleration;
    //fixed acceleration to use
    double fixedAcceleration;

    /** tells the module to ignore modifications to the state state of the vehicle. a class is going to invoke
     *  methods like followSpeed, but not for changing actually moving the vehicle. for example the lane changer
     *  invokes followSpeed to determine whether the car can gain some advantages by moving to another lane. this
     *  advantage is computed by using the followSpeed method. in such case, the state of the vehicle should not
     *  be changed
     */
    bool ignoreModifications;

    //car collided in the last timestep
    bool crashed;

    /// @brief CC desired speed
    double ccDesiredSpeed;
    /// @brief currently active controller
    enum Plexe::ACTIVE_CONTROLLER activeController;

    /// @brief fake controller data. @see FAKE_CONTROLLER_DATA
    struct FAKE_CONTROLLER_DATA fakeData;
    /** acceleration computed by the ACC system. This can be used during the approach phase to understand whether
     *  a slow vehicles is in front
     */
    double accAcceleration, followAccAcceleration, freeAccAcceleration;

    /// @brief lane change action to be performed as given by the platoon management application
    enum Plexe::PLATOONING_LANE_CHANGE_ACTION laneChangeAction;

    /// @brief fixed lane selected
    int fixedLane;

    //TODO: most probably the following variables needs to be moved to the application logic (i.e., network protocol)
    /// @brief own platoon id
    std::string platoonId;
    /// @brief is ego vehicle the leader?
    bool isPlatoonLeader;

    /// @brief L matrix
    int L[MAX_N_CARS][MAX_N_CARS];
    /// @brief K matrix
    double K[MAX_N_CARS][MAX_N_CARS];
    /// @brief vector of damping ratios b
    double b[MAX_N_CARS];
    /// @brief vector of time headways h
    double h[MAX_N_CARS];

    /// @brief data about vehicles in the platoon
    struct Plexe::VEHICLE_DATA vehicles[MAX_N_CARS];
    /// @brief tells whether data about a certain vehicle has been initialized
    bool initialized[MAX_N_CARS];
    /// @brief count of initialized vehicles
    int nInitialized;
    /// @brief my position within the platoon (0 = first car)
    int position;
    /// @brief number of cars in the platoon
    int nCars;

    /// @brief controller related parameters
    double caccXi;
    double caccOmegaN;
    double caccC1;
    double caccAlpha1, caccAlpha2, caccAlpha3, caccAlpha4, caccAlpha5;
    double engineTau, engineAlpha, engineOneMinusAlpha;
    double ploegH;
    double ploegKp;
    double ploegKd;

    /// @brief engine model employed by this car
    GenericEngineModel *engine;
};

#endif
