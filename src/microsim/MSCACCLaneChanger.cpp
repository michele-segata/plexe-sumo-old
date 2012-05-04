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
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "MSCACCLaneChanger.h"

MSCACCLaneChanger::MSCACCLaneChanger(std::vector<MSLane*>* lanes, bool allowSwap) : MSLaneChanger(lanes, allowSwap) {}

MSCACCLaneChanger::~MSCACCLaneChanger() {}

bool MSCACCLaneChanger::change() {
    return false;
}

