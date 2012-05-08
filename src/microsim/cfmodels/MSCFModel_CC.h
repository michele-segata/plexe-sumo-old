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
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSCFMODEL_CC_H
#define	MSCFMODEL_CC_H

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <microsim/MSCFModel.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <microsim/cfmodels/MSCFModel_Krauss.h>


// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_CC
 * @brief A set of automatic Cruise Controllers, including classic Cruise
 * Control (CC), Adaptive Cruise Control (ACC) and Cooperative Adaptive Cruise
 * Control (CACC). Take as references the chapters 5, 6 and 7 of the Rajamani's
 * book "Vehicle dynamics and control" (2011).
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
     */
    MSCFModel_CC(const MSVehicleType* vtype, SUMOReal accel, SUMOReal decel,
                 SUMOReal ccDecel, SUMOReal headwayTime, SUMOReal constantSpacing,
                 SUMOReal kp, SUMOReal lambda, SUMOReal c1, SUMOReal xi,
                 SUMOReal omegaN, SUMOReal tau);

    /// @brief Destructor
    ~MSCFModel_CC();


    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Applies interaction with stops and lane changing model influences
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
     */
    SUMOReal moveHelper(MSVehicle* const veh, SUMOReal vPos) const;


    /** @brief Computes the vehicle's safe speed (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     * @see MSCFModel::ffeV
     */
    SUMOReal followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel) const;


    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] gap2pred The (netto) distance to the the obstacle
     * @return EGO's safe speed for approaching a non-moving obstacle
     * @see MSCFModel::ffeS
     * @todo generic Interface, models can call for the values they need
     */
    SUMOReal stopSpeed(const MSVehicle* const veh, SUMOReal gap2pred) const;


    /** @brief Returns the maximum gap at which an interaction between both vehicles occurs
     *
     * "interaction" means that the LEADER influences EGO's speed.
     * @param[in] veh The EGO vehicle
     * @param[in] vL LEADER's speed
     * @return The interaction gap
     * @todo evaluate signature
     * @see MSCFModel::interactionGap
     */
    SUMOReal interactionGap(const MSVehicle* const , SUMOReal vL) const;


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
        return new VehicleVariables();
    }

    void setCCDesiredSpeed(const MSVehicle* veh, SUMOReal ccDesiredSpeed);

    void setLeaderInformation(const MSVehicle* veh, SUMOReal speed, SUMOReal acceleration);

    void getVehicleInformation(const MSVehicle* veh, SUMOReal& speed, SUMOReal& acceleration);


private:
    class VehicleVariables : public MSCFModel::VehicleVariables {
    public:
        VehicleVariables() : egoSpeed(0), egoAcceleration(0), frontSpeed(0), frontDistance(0), leaderSpeed(0),
            leaderAcceleration(0), platoonId(""), isPlatoonLeader(false), ccDesiredSpeed(14), activeController(ACC), lastUpdate(0) {}

        /** @enum ActiveController
         * @brief Determines the currently active controller, i.e., ACC, CACC, or the
         * driver. In future we might need to switch off the automatic controller and
         * leave the control to the mobility model which reproduces a human driver
         */
        enum ActiveController
        {ACC, CACC, DRIVER};

        /// @brief current vehicle speed
        SUMOReal egoSpeed;
        /// @brief current vehicle acceleration
        SUMOReal egoAcceleration;
        /// @brief vehicle speed at previous timestep
        SUMOReal egoPreviousSpeed;
        /// @brief current front vehicle speed
        SUMOReal frontSpeed;
        /// @brief current front vehicle distance
        SUMOReal frontDistance;
        /// @brief current front vehicle acceleration (used by CACC)
        SUMOReal frontAcceleration;
        /// @brief last timestep at which front vehicle data has been updated
        SUMOTime lastUpdate;
        /// @brief platoon's leader speed (used by CACC)
        SUMOReal leaderSpeed;
        /// @brief platoon's leader acceleration (used by CACC)
        SUMOReal leaderAcceleration;

        /// @brief CC desired speed
        SUMOReal ccDesiredSpeed;
        /// @brief currently active controller
        enum ActiveController activeController;

        //TODO: most probably the following variables needs to be moved to the application logic (i.e., network protocol)
        /// @brief own platoon id
        std::string platoonId;
        /// @brief is ego vehicle the leader?
        bool isPlatoonLeader;
    };


private:
    SUMOReal _v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal desSpeed, bool invokedFromFollowingSpeed) const;

    /** @brief controller for the CC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @return the acceleration to be given to the actuator
     */
    SUMOReal _cc(SUMOReal egoSpeed, SUMOReal desSpeed) const;

    /** @brief controller for the ACC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @param[in] gap2pred the distance to preceding vehicle
     * @return the acceleration to be given to the actuator
     */
    SUMOReal _acc(SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal gap2pred) const;

    /** @brief controller for the CACC which computes the acceleration to be applied. the value needs to be passed to the actuator
     *
     * @param[in] egoSpeed vehicle current speed
     * @param[in] desSpeed vehicle desired speed
     * @param[in] predAcceleration acceleration of preceding vehicle
     * @param[in] gap2pred the distance to preceding vehicle
     * @param[in] leaderSpeed the speed of the platoon leader
     * @param[in] leaderAcceleration the acceleration of the platoon leader
     * @return the acceleration to be given to the actuator
     */
    SUMOReal _cacc(SUMOReal egoSpeed, SUMOReal predSpeed, SUMOReal predAcceleration, SUMOReal gap2pred, SUMOReal leaderSpeed, SUMOReal leaderAcceleration) const;

    /** @brief computes the actual acceleration the actuator is able to apply to the car, given engine time constant and previous
     * acceleration
     *
     * @param[in] acceleration the acceleration to be applied, computed by the controller
     * @param[in] currentAcceleration the current car acceleration
     * @return the actual acceleration applied by the engine
     */
    SUMOReal _actuator(SUMOReal acceleration, SUMOReal currentAcceleration) const;

    SUMOReal desiredSpeed(const MSVehicle* const veh) const {
        return MIN2(myType->getMaxSpeed(), veh->getLane()->getMaxSpeed());
    }


private:

    /// @brief the car following model which drives the car when automated cruising is disabled, i.e., the human driver
    MSCFModel *myHumanDriver;

    /// @brief The maximum deceleration that the CC can output
    const SUMOReal myCcDecel;

    /// @brief the constant gap for CACC
    const SUMOReal myConstantSpacing;

    /// @brief design constant for CC
    const SUMOReal myKp;

    /// @brief design constant for ACC
    const SUMOReal myLambda;

    /// @brief design constant for CACC
    const SUMOReal myC1;

    /// @brief design constant for CACC
    const SUMOReal myXi;

    /// @brief design constant for CACC
    const SUMOReal myOmegaN;

    /// @brief engine time constant used for actuation lag
    const SUMOReal myTau;
    /// @brief the alpha parameter for the low-pass filter used to implement the actuation lag
    const SUMOReal myAlpha;
    /// @brief one minus alpha parameters
    const SUMOReal myOneMinusAlpha;

    /// @brief A computational shortcut for CACC
    const SUMOReal myAlpha1;
    /// @brief A computational shortcut for CACC
    const SUMOReal myAlpha2;
    /// @brief A computational shortcut for CACC
    const SUMOReal myAlpha3;
    /// @brief A computational shortcut for CACC
    const SUMOReal myAlpha4;
    /// @brief A computational shortcut for CACC
    const SUMOReal myAlpha5;

};

#endif	/* MSCFMODEL_CC_H */
