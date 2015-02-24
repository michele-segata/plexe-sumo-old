/****************************************************************************/
/// @file    GenericEngineModel.cpp
/// @author  Michele Segata
/// @date    4 Feb 2015
/// @version $Id: $
///
// Generic interface for an engine model
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

#include "GenericEngineModel.h"

#include <iostream>
#include <stdio.h>

void GenericEngineModel::printParameterError(std::string parameter, std::string value) {
    std::cerr << className << ": invalid value " << value << " for parameter " << parameter << std::endl;
}

void GenericEngineModel::parseParameter(const ParMap & parameters, std::string parameter, double &value) {
    ParMap::const_iterator par = parameters.find(parameter);
    double v;
    if (par != parameters.end()) {
        if (sscanf(par->second.c_str(), "%lf", &v) != 1)
            printParameterError(par->first, par->second);
        else
            value = v;
    }
}
void GenericEngineModel::parseParameter(const ParMap & parameters, std::string parameter, int &value) {
    ParMap::const_iterator par = parameters.find(parameter);
    int v;
    if (par != parameters.end()) {
        if (sscanf(par->second.c_str(), "%d", &v) != 1)
            printParameterError(par->first, par->second);
        else
            value = v;
    }
}
void GenericEngineModel::parseParameter(const ParMap & parameters, std::string parameter, std::string &value) {
    ParMap::const_iterator par = parameters.find(parameter);
    if (par != parameters.end())
        value = par->second;
}
