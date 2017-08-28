/****************************************************************************/
/// @file    MSCFModel_CC.h
/// @author  Michele Segata
/// @date    Wed, 18 Apr 2012
/// @version $Id: $
///
// A series of automatic Cruise Controllers (CC, ACC, CACC)
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
#ifndef MSCFMODEL_CC_H
#define MSCFMODEL_CC_H

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
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <microsim/cfmodels/MSCFModel_Krauss.h>
#include <string.h>

#include "GenericEngineModel.h"
#include "FirstOrderLagModel.h"
#include "RealisticEngineModel.h"

#include "CC_VehicleVariables.h"


// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_CC
 * @brief A set of automatic Cruise Controllers, including classic Cruise
 * Control (CC), Adaptive Cruise Control (ACC) and Cooperative Adaptive Cruise
 * Control (CACC). Take as references the chapters 5, 6 and 7 of the Rajamani's
 * book "Vehicle dynamics and control" (2011).
 * This model is meant to be used for simulation of platooning systems in mixed
 * scenarios, so with automatic and driver controlled vehicles.
 * The platooning manager is a distributed application implemented for veins
 * (so for omnet++) supported by a 802.11p based communication protocol, which
 * will determine the actions to be performed (such as switching on the
 * automatic controller, or the lane to move to) and communicate them to this
 * car following models via TraCI
 * @see MSCFModel
 */
class MSCFModel_CC : public MSCFModel {
public:

    /** @brief Constructor
     * @param[in] accel The maximum acceleration that controllers can output (def. 1.5 m/s^2)
     * @param[in] decel The maximum deceleration that ACC and CACC controllers can output (def. 6 m/s^2)
     * @param[in] ccDecel The maximum deceleration that the CC can output (def. 1.5 m/s^2)
     * @param[in] headwayTime the headway gap for ACC (be aware of instabilities) (def. 1.5 s)
     * @param[in] constantSpacing the constant gap for CACC (def. 5 m)
     * @param[in] kp design constant for CC (def. 1)
     * @param[in] lambda design constant for ACC (def. 0.1)
     * @param[in] c1 design constant for CACC (def. 0.5)
     * @param[in] xi design constant for CACC (def. 1)
     * @param[in] omegaN design constant for CACC (def. 0.2)
     * @param[in] tau engine time constant used for actuation lag (def. 0.5 s)
     * @param[in] lanesCount number of lanes of the highway
     * @param[in] ccAccel the maximum acceleration the CC can apply
     */
    MSCFModel_CC(const MSVehicleType* vtype, double accel, double decel,
                 double ccDecel, double headwayTime, double constantSpacing,
                 double kp, double lambda, double c1, double xi,
                 double omegaN, double tau, int lanesCount, double ccAccel);

    /// @brief Destructor
    ~MSCFModel_CC();


    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Applies interaction with stops and lane changing model influences
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
     */
    double moveHelper(MSVehicle* const veh, double vPos) const;


    /** @brief Computes the vehicle's safe speed (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     * @see MSCFModel::ffeV
     */
    double followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel) const;

    /** @brief Overload base MSCFModel::insertionFollowSpeed method to inject
     * automated vehicles as soon as they are requested, without checking
     * for safe speed constraints
     *
     */
    virtual double insertionFollowSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel) const;


    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] gap2pred The (netto) distance to the the obstacle
     * @return EGO's safe speed for approaching a non-moving obstacle
     * @see MSCFModel::ffeS
     * @todo generic Interface, models can call for the values they need
     */
    double stopSpeed(const MSVehicle* const veh, double speed, double gap2pred) const;

    /** @brief Computes the vehicle's safe speed without a leader
     *
     * Returns the velocity of the vehicle in dependence to the length of the free street and the target
     *  velocity at the end of the free range. If onInsertion is true, the vehicle may still brake
     *  before the next movement.
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] seen The look ahead distance
     * @param[in] maxSpeed The maximum allowed speed
     * @param[in] onInsertion whether speed at insertion is asked for
     * @return EGO's safe speed
     */
    virtual double freeSpeed(const MSVehicle* const veh, double speed, double seen,
                               double maxSpeed, const bool onInsertion = false) const;

    double maxNextSpeed(double speed) const;


    /** @brief Returns the maximum gap at which an interaction between both vehicles occurs
     *
     * "interaction" means that the LEADER influences EGO's speed.
     * @param[in] veh The EGO vehicle
     * @param[in] vL LEADER's speed
     * @return The interaction gap
     * @todo evaluate signature
     * @see MSCFModel::interactionGap
     */
    double interactionGap(const MSVehicle* const , double vL) const;


    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    int getModelID() const {
        return SUMO_TAG_CF_CC;
    }
    /// @}



    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
     */
    MSCFModel* duplicate(const MSVehicleType* vtype) const;


    VehicleVariables* createVehicleVariables() const {
        CC_VehicleVariables *vars = new CC_VehicleVariables();
        vars->caccSpacing = myConstantSpacing;
        vars->caccC1 = myC1;
        vars->caccXi = myXi;
        vars->caccOmegaN = myOmegaN;
        vars->engineTau = myTau;
        //we cannot invoke recomputeParameters() because we have no pointer to the MSVehicle class
        vars->caccAlpha1 = 1 - vars->caccC1;
        vars->caccAlpha2 = vars->caccC1;
        vars->caccAlpha3 = -(2 * vars->caccXi - vars->caccC1 * (vars->caccXi + sqrt(vars->caccXi * vars->caccXi - 1))) * vars->caccOmegaN;
        vars->caccAlpha4 = -(vars->caccXi + sqrt(vars->caccXi* vars->caccXi - 1)) * vars->caccOmegaN * vars->caccC1;
        vars->caccAlpha5 = -vars->caccOmegaN * vars->caccOmegaN;
        vars->engineAlpha = TS / (vars->engineTau + TS);
        vars->engineOneMinusAlpha = 1 - vars->engineAlpha;
        //by default use a first order lag model for the engine
        vars->engine = new FirstOrderLagModel();
        vars->engine->setParameter(FOLM_PAR_TAU, vars->engineTau);
        vars->engine->setParameter(FOLM_PAR_DT, TS);
        vars->engine->setMaximumAcceleration(myAccel);
        vars->engine->setMaximumDeceleration(myDecel);
        return (VehicleVariables *)vars;
    }

    /**
     * @brief set the cruise control desired speed. notice that this command does
     * not enable the cruise control and can also be used when cruise control is
     * already active
     *
     * @param[in] veh the vehicle for which the desired speed has to be changed
     * @param[in] ccDesiredSpeed the desired speed in m/s
     */
    void setCCDesiredSpeed(const MSVehicle* veh, double ccDesiredSpeed) const;

    /**
     * @brief set CACC desired constant spacing
     *
     * @param[in] veh the vehicle to set constant spacing for
     * @param[in] spacing the spacing in meters
     */
    void setCACCConstantSpacing(const MSVehicle * veh, double spacing) const;

    /**
     * @brief returns CACC desired constant spacing
     *
     * @param[in] veh the vehicle to get constant spacing of
     * @return spacing the spacing in meters
     */
    double getCACCConstantSpacing(const MSVehicle * veh) const;

    /**
     * @brief set the information about the platoon leader. This method should be invoked
     * by TraCI when a wireless message with such data is received. For testing, it might
     * be also invoked from SUMO source code
     *
     * @param[in] veh the vehicle for which the data must be saved
     * @param[in] speed the leader speed
     * @param[in] acceleration the leader acceleration
     * @param[in] position the position of the leader
     * @param[in] time the time at which this data was read from leader's sensors
     */
    void setLeaderInformation(const MSVehicle* veh, double speed, double acceleration, Position position, double time)  const;

    /**
     * @brief set the information about preceding vehicle. This method should be invoked
     * by TraCI when a wireless message with such data is received. For testing, it might
     * be also invoked from SUMO source code
     *
     * @param[in] veh the vehicle for which the data must be saved
     * @param[in] speed the speed of the preceding vehicle
     * @param[in] acceleration the acceleration of the preceding vehicle
     * @param[in] position the position of the preceding vehicle
     * @param[in] time the time at which this data was read from preceding vehicle's sensors
     *
     */
    void setPrecedingInformation(const MSVehicle* const veh, double speed, double acceleration, Position position, double time) const;

    /**
     * @brief set the information about a generic car. This method should be invoked
     * by TraCI when a wireless message with such data is received. For testing, it might
     * be also invoked from SUMO source code
     *
     * @param[in] veh the vehicle for which the data must be saved
     * @param[in] speed the leader speed
     * @param[in] acceleration the leader acceleration
     * @param[in] position the position of the leader
     * @param[in] time the time at which this data was read from leader's sensors
     */
//    void setVehicleInformation(const MSVehicle* veh, double speed, double acceleration, Position position, double time)  const;

    /**
     * @brief generic data passing method to this class
     *
     * @param[in] veh the vehicle to which data is directed to
     * @param[in] header header including information about the actual message
     * @param[in] content pointer to the actual content
     */
    void setGenericInformation(const MSVehicle* veh, const struct Plexe::CCDataHeader &header, const void *content) const;

    /**
     * @brief generic data retrieval method from this class
     *
     * @param[in] veh the vehicle from which data is requested
     * @param[in] request struct including request type, and request parameters length
     * @param[in] reqParams parameters connected to the request, if any
     * @param[out] content pointer where data will be copied to
     * @return actual size of data copied into content
     */
    int getGenericInformation(const MSVehicle *veh, struct Plexe::CCDataHeader request, const void *reqParams, void *content) const;

    /**
     * @brief get the information about a vehicle. This can be used by TraCI in order to
     * get speed and acceleration of the platoon leader before sending them to other
     * vehicles
     *
     * @param[in] veh the vehicle for which the data is requested
     * @param[out] speed where the speed is written
     * @param[out] acceleration where the acceleration is written
     * @param[out] controllerAcceleration the last acceleration value computed by
     * the controller will be written in this variable. This might be different from
     * acceleration because of actuation lag
     */
    void getVehicleInformation(const MSVehicle* veh, double& speed, double& acceleration, double& controllerAcceleration, Position &position, double &time) const;

    /**
     * @brief switch on the ACC, so disabling the human driver car control
     *
     * @param[in] veh the vehicle for which the ACC must be switched on
     * @param[in] ccDesiredSpeed the cruise control speed
     */
    void switchOnACC(const MSVehicle *veh, double ccDesiredSpeed) const;

    /**
     * @brief set the headway time for the ACC controller. This might be useful for
     * analyzing instabilities, for example by setting headway time to a value like
     * 0.3 seconds
     *
     * @param[in] veh the vehicle for which the ACC headway time must be changed
     * @param[in] headwayTime headway time to be set
     */
    void setACCHeadwayTime(const MSVehicle *veh, double headwayTime) const;

    /**
     * @brief enable/disable the use of a fixed acceleration, instead of using the one computed by
     * the controllers. the acceleration is used only if the CC/ACC/CACC is enabled
     *
     * @param[in] veh the vehicle for which the fixed acceleration has to be set
     * @param[in] activate activate (1) or deactivate (0) the fixed acceleration
     * @param[in] acceleration the fixed acceleration to be used (if activate == 1)
     */
    void setFixedAcceleration(const MSVehicle *veh, int activate, double acceleration) const;

    /**
     * @brief set the active controller. Notice that if the selected controller is ACC or CACC
     * the setCCDesiredSpeed must be invoked before, otherwise the speed is set to the default
     * value
     *
     * @param[in] veh the vehicle for which the action is requested
     * @param[in] activeController the controller to be set as active, which can be either the
     * driver, or the ACC or the CACC
     */
    void setActiveController(const MSVehicle *veh, enum Plexe::ACTIVE_CONTROLLER activeController)  const;

    /**
     * @brief return the currently active controller
     *
     * @param[in] veh the vehicle for which the action is requested
     * @return the currently active controller
     */
    enum Plexe::ACTIVE_CONTROLLER getActiveController(const MSVehicle *veh) const;

    /**
     * @brief gets the lane change action requested by the platooning management system.
     * The action is set by the platooning manager via TraCI and it is requested by the
     * MSCACCLaneChanger class
     *
     * @return the lane changing action to be performed
     */
    enum Plexe::PLATOONING_LANE_CHANGE_ACTION getLaneChangeAction(const MSVehicle *veh) const;

    /**
     * @brief sets the lane change action requested by the platooning management system.
     */
    void setLaneChangeAction(const MSVehicle *veh, enum Plexe::PLATOONING_LANE_CHANGE_ACTION action) const;

    /**
     * @brief sets the lane a car should stay in. might be useful to form platoons in any
     * lane and not in a dedicated one.
     * After setting the lane, the lane change action (enum PLATOONING_LANE_CHANE_ACTION) will
     * be set to STAY_IN_CURRENT_LANE. Afterwards, it might be possible to either call this
     * method again to change lane (and still keep it fixed) or setting the lane change action
     * to DRIVER_CHOICE, to let the driver model chose the appropriate lane
     *
     * @param[in] lane index of the lane to move to, starting from 0. 0 is the rightmost.
     * If there is no such lane, the request will be ignored
     */
    void setFixedLane(const MSVehicle *veh, int lane) const;

    /**
     * @brief return the data that is currently being measured by the radar
     */
    void getRadarMeasurements(const MSVehicle * veh, double &distance, double &relativeSpeed) const;

    /**
     * @brief set the fake data which the controller will use for joining while in the
     * management lane. Notice that the function can be used to set data about both leader and
     * front vehicle, or either one or the other. If the speed of one of the two is set to a
     * negative value, then the data about such vehicle is ignored
     */
    void setControllerFakeData(const MSVehicle *veh, double frontDistance, double frontSpeed, double frontAcceleration,
            double leaderSpeed, double leaderAcceleration) const;

    /**
     * @brief tells the module to ignore modifications to the state state of the vehicle. a class is going to invoke
     * methods like followSpeed, but not for changing actually moving the vehicle. for example the lane changer
     * invokes followSpeed to determine whether the car can gain some advantages by moving to another lane. this
     * advantage is computed by using the followSpeed method. in such case, the state of the vehicle should not
     * be changed
     */
    void setIgnoreModifications(const MSVehicle *veh, bool ignore) const;

    /**
     * @brief tells the module that in the last timestep the car has crashed (or not)
     *
     * @param[in] veh the vehicle
     * @param[in] crashed whether the car has crashed or not
     */
    void setCrashed(const MSVehicle *veh, bool crashed) const;

    /**
     * @brief tells whether the car has crashed or not
     *
     * @param[in] veh the vehicle
     * @return true if the vehicle has crashed, false otherwise
     */
    bool isCrashed(const MSVehicle *veh) const;

    /**
     * @brief returns the ACC computed acceleration when the faked
     * CACC is controlling the car. This can be used to check for
     * vehicles in front
     */
    double getACCAcceleration(const MSVehicle *veh) const;

    /**
     * @brief returns the number of lanes set in the configuration file
     */
    int getMyLanesCount() const;

private:

    /**
     * This enum tells to the _v method who has invoked it
     */
    enum CONTROLLER_INVOKER {
        STOP_SPEED,
        FOLLOW_SPEED,
        FREE_SPEED
    };

    /**
     * @brief Recomputes controller related parameters after setting them
     */
    void recomputeParameters(const MSVehicle *veh) const;

    /**
     * @brief Resets the consensus controller. In particular, sets the
     * "initialized" vector all to false. This might be useful when changing
     * topology.
     */
    void resetConsensus(const MSVehicle *veh) const;

private:
    double _v(const MSVehicle* const veh, double gap2pred, double egoSpeed, double predSpeed, double desSpeed, enum CONTROLLER_INVOKER invoker) const;

    /** @brief controller for the CC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @return the acceleration to be given to the actuator
     */
    double _cc(const MSVehicle *veh, double egoSpeed, double desSpeed) const;

    /** @brief controller for the ACC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @param[in] gap2pred the distance to preceding vehicle
     * @param[in] headwayTime the headway time ACC should maintain
     * @return the acceleration to be given to the actuator
     */
    double _acc(const MSVehicle *veh, double egoSpeed, double predSpeed, double gap2pred, double headwayTime) const;

    /** @brief controller for the CACC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @param[in] predAcceleration acceleration of preceding vehicle
     * @param[in] gap2pred the distance to preceding vehicle
     * @param[in] leaderSpeed the speed of the platoon leader
     * @param[in] leaderAcceleration the acceleration of the platoon leader
     * @param[in] spacing the spacing to be kept
     * @return the acceleration to be given to the actuator
     */
    double _cacc(const MSVehicle *veh, double egoSpeed, double predSpeed, double predAcceleration, double gap2pred, double leaderSpeed, double leaderAcceleration, double spacing) const;

    /** @brief controller for the Ploeg's CACC which computes the control input variation.
     * Opposed to other controllers, this method returns a value which needs to be summed
     * to the previous desired acceleration.
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] predSpeed the speed of the front vehicle
     * @param[in] predAcceleration acceleration of preceding vehicle
     * @param[in] gap2pred the distance to preceding vehicle
     * @return the variation of desired acceleration
     */
    double _ploeg(const MSVehicle *veh, double egoSpeed, double predSpeed, double predAcceleration, double gap2pred) const;

    /** @brief controller based on consensus strategy
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] egoPosition vehicle current position
     * @param[in] time current time
     * @return the acceleration to be given to the actuator
     */
    double _consensus(const MSVehicle* veh, double egoSpeed, Position egoPosition, double time) const;

    /** @brief computes the desired distance between vehicle i and vehicle j
     *
     * @param[in] vehicles data about all vehicles
     * @param[in] h vector of times headway
     * @param[in] i index of own vehicle
     * @param[in] j index of vehicle to compute distance from
     * @return the desired distance between vehicle i and j
     *
     */
    double d_i_j(const struct Plexe::VEHICLE_DATA *vehicles, const double h[MAX_N_CARS], int i, int j) const;

    /** @brief computes the actual acceleration the actuator is able to apply to the car, given engine time constant and previous
     * acceleration
     *
     * @param[in] acceleration the acceleration to be applied, computed by the controller
     * @param[in] currentAcceleration the current car acceleration
     * @return the actual acceleration applied by the engine
     */
    double _actuator(const MSVehicle *veh, double acceleration, double currentAcceleration) const;

    double desiredSpeed(const MSVehicle* const veh) const {
        return MIN2(myType->getMaxSpeed(), veh->getLane()->getSpeedLimit());
    }


private:

    /// @brief the car following model which drives the car when automated cruising is disabled, i.e., the human driver
    MSCFModel *myHumanDriver;

    /// @brief The maximum deceleration that the CC can output
    const double myCcDecel;

    /// @brief The maximum acceleration that the CC can output
    const double myCcAccel;

    /// @brief the constant gap for CACC
    const double myConstantSpacing;

    /// @brief design constant for CC
    const double myKp;

    /// @brief design constant for ACC
    const double myLambda;

    /// @brief design constant for CACC
    const double myC1;

    /// @brief design constant for CACC
    const double myXi;

    /// @brief design constant for CACC
    const double myOmegaN;

    /// @brief engine time constant used for actuation lag
    const double myTau;
    /// @brief the alpha parameter for the low-pass filter used to implement the actuation lag
    const double myAlpha;
    /// @brief one minus alpha parameters
    const double myOneMinusAlpha;

    /// @brief A computational shortcut for CACC
    const double myAlpha1;
    /// @brief A computational shortcut for CACC
    const double myAlpha2;
    /// @brief A computational shortcut for CACC
    const double myAlpha3;
    /// @brief A computational shortcut for CACC
    const double myAlpha4;
    /// @brief A computational shortcut for CACC
    const double myAlpha5;

    /// @brief number of lanes in the highway, in the absence of on-/off-ramps. This is used
    /// to move to the correct lane even when a lane is added for on-/off-ramps
    const int myLanesCount;

};

#endif /* MSCFMODEL_CC_H */
