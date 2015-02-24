/****************************************************************************/
/// @file    FirstOrderLagModel.cpp
/// @author  Michele Segata
/// @date    4 Feb 2015
/// @version $Id: $
///
// An engine model using a first order lag
/****************************************************************************/
// Copyright (C) 2015 Michele Segata (segata@ccs-labs.org)
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "FirstOrderLagModel.h"
#include "CC_Const.h"

FirstOrderLagModel::FirstOrderLagModel() : GenericEngineModel(),
    tau_s(0.5), dt_s(0.01) {
    className = "FirstOrderLagModel";
    computeParameters();
}
FirstOrderLagModel::~FirstOrderLagModel() {}

void FirstOrderLagModel::computeParameters() {
    alpha = dt_s / (tau_s + dt_s);
    oneMinusAlpha = 1 - alpha;
}

double FirstOrderLagModel::getRealAcceleration(double speed_mps, double accel_mps2, double reqAccel_mps2, int timeStep) {
    return alpha * accel_mps2 + oneMinusAlpha * reqAccel_mps2;
}

void FirstOrderLagModel::loadParameters(const ParMap &parameters) {
    parseParameter(parameters, std::string(FOLM_PAR_TAU), tau_s);
    parseParameter(parameters, std::string(FOLM_PAR_DT), dt_s);
    computeParameters();
}

void FirstOrderLagModel::setParameter(const std::string parameter, const std::string &value) {}
void FirstOrderLagModel::setParameter(const std::string parameter, double value) {
    if (parameter == FOLM_PAR_TAU)
        tau_s = value;
    if (parameter == FOLM_PAR_DT)
        dt_s = value;
    computeParameters();
}
void FirstOrderLagModel::setParameter(const std::string parameter, int value) {}
