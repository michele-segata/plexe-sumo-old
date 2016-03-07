/****************************************************************************/
/// @file    CC_VehicleVariables.cpp
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
#include "CC_VehicleVariables.h"

//initialize default L and K matrices
const int CC_VehicleVariables::defaultL[][MAX_N_CARS] =
    {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 0, 0, 0, 0},
    {1, 0, 0, 1, 0, 0, 0, 0},
    {1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 0, 0, 1, 0, 0},
    {1, 0, 0, 0, 0, 0, 1, 0}
    };
const double CC_VehicleVariables::defaultK[][MAX_N_CARS] =
    {
    {0  , 0  , 0  , 0  , 0  , 0  , 0  , 0},
    {460, 0  , 0  , 0  , 0  , 0  , 0  , 0},
    {80 , 860, 0  , 0  , 0  , 0  , 0  , 0},
    {80 , 0  , 860, 0  , 0  , 0  , 0  , 0},
    {80 , 0  , 0  , 860, 0  , 0  , 0  , 0},
    {80 , 0  , 0  , 0  , 860, 0  , 0  , 0},
    {80 , 0  , 0  , 0  , 0  , 860, 0  , 0},
    {80 , 0  , 0  , 0  , 0  , 0  , 860, 0}
    };
const double CC_VehicleVariables::defaultB[MAX_N_CARS] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
const double CC_VehicleVariables::defaultH[MAX_N_CARS] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
