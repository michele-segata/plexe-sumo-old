/*
 * MSCFModel_CACC.h
 *
 *  Created on: Apr 4, 2012
 *      Author: segata
 */

#ifndef MSCFMODEL_CACC_H_
#define MSCFMODEL_CACC_H_

#include "MSCFModel.h"

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

	virtual ~MSCFModel_CACC();

private:


	const double C1 = 0.5; //0 < C1 < 1
	const double xi = 1; //xi >= 1
	const double omega_n = 0.2; //bandwidth of the controller, i.e., the range of frequency that the controller can track without attenuation

	//time constant for the first order low-pass filter representing engine delayed response
	//NB: it is assumed that the time step of the simulation is 0.1 seconds!! the time
	//constant of the first order lag is 0.5s
	const double engine_fod_const = 0.1/(0.1+0.5);

};

#endif /* MSCFMODEL_CACC_H_ */
