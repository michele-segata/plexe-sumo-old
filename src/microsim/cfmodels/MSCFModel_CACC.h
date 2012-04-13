/*
 * MSCFModel_CACC.h
 *
 *  Created on: Apr 4, 2012
 *      Author: segata
 */

#ifndef MSCFMODEL_CACC_H_
#define MSCFMODEL_CACC_H_

#include <microsim/MSCFModel.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>
#include <utils/xml/SUMOXMLDefinitions.h>

// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_CACC
 * @brief Car following model implementing a Cooperative Adaptive Cruise Controller, in particular
 * the CACC described in the book of Rajamani "Vehicle dynamics and control", 2011
 * @see MSCFModel
 */
class MSCFModel_CACC: public MSCFModel {
public:

	/** @brief Constructor
	 * @param[in] accel The maximum acceleration
	 * @param[in] decel The maximum deceleration
	 * @param[in] desiredGap the desired gap in meters. Constant, speed-independent
	 */
	MSCFModel_CACC(const MSVehicleType* vtype, SUMOReal accel, SUMOReal decel,
			SUMOReal desiredGap);

    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Applies interaction with stops and lane changing model influences
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
     *
     * //TODO: copy-pasted from IDM: understand it
     */
    SUMOReal moveHelper(MSVehicle* const veh, SUMOReal vPos) const;

	virtual ~MSCFModel_CACC();

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

    /** @brief Set leader speed
     *
     */
    void setLeaderSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal acceleration);

    void getVariables(const MSVehicle* const veh, SUMOReal *speed, SUMOReal *acceleration);

    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
     */
    int getModelID() const {
        return SUMO_TAG_CF_CACC;
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


private:

	/// @brief C1 model constant. 0 < C1 < 1
	const SUMOReal C1;
	/// @brief xi model constant. xi >= 1
	const SUMOReal xi;
	/// @brief bandwidth of the controller, i.e., the range of frequency that the controller can track without attenuation
	const SUMOReal omega_n;

	/// @brief the desired gap for the CACC, usually in the order of 2 to 10 meters
	const SUMOReal myDesiredGap;

	/** @brief time constant for the first order low-pass filter representing engine delayed response
	 * NB: it is assumed that the time step of the simulation is 0.1 seconds!!
	 */
	const SUMOReal engineTimeConst;
	/// @brief alpha and 1-alpha for computing first order lag: filtOut[t] = alpha * val + (1 - alpha) * filtOut[t-1]
	SUMOReal alpha, oneMinusAlpha;

	/// @brief set of variables used to avoid useless re-computations at each timestep
	SUMOReal var1, var2, var3, var4, var5;

	SUMOReal _v(const MSVehicle* const veh, SUMOReal gap2pred, SUMOReal mySpeed, SUMOReal predSpeed, SUMOReal desSpeed) const;

	SUMOReal desiredSpeed(const MSVehicle* const veh) const {
		return MIN2(myType->getMaxSpeed(), veh->getLane()->getMaxSpeed());
	}

	/// @brief class for saving informations about platoon leader and other variables
	class VehicleVariables : public MSCFModel::VehicleVariables {
	    public:
	        VehicleVariables() : prevEgoAcc(0), leaderAcc(0), leaderSpeed(0), predPreviousSpeed(0), isLeader(false), platoonId(-1) {}
	        /// @brief previous acceleration. needed in order to compute the delayed engine response
	        SUMOReal prevEgoAcc;
	        /// @brief acceleration of the platoon leader. this is communicated by means of wireless networking
	        SUMOReal leaderAcc;
	        /// @brief speed of the platoon leader. communicated by means of wireless networking
	        SUMOReal leaderSpeed;
	        /// @brief speed of preceding vehicle at the previous timestep. this is needed in order to compute
	        /// the acceleration of the preceding vehicle, used by the CACC model
	        SUMOReal predPreviousSpeed;
	        /// @brief variable determining whether ego vehicle is the leader of the platoon or not
	        bool isLeader;
	        /// @brief platoon id
	        int platoonId;
	    };

};

#endif /* MSCFMODEL_CACC_H_ */
