/****************************************************************************/
/// @file    GenericEngineModel.h
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

#ifndef GENERICENGINEMODEL_H_
#define GENERICENGINEMODEL_H_

#include <map>
#include <string>

/**
 * This is an interface for plexe engine models. It provides two virtual methods
 * that should be overridden by implementing classes: getRealAcceleration and
 * loadParameters
 */
class GenericEngineModel {

public:

    typedef std::map<std::string, std::string> ParMap;

protected:

    //class name, used to log information
    std::string className;

    /**
     * Prints a parameter error
     */
    void printParameterError(std::string parameter, std::string value);

    /**
     * Parses a value from the parameter map
     */
    void parseParameter(const ParMap & parameters, std::string parameter, double &value);
    void parseParameter(const ParMap & parameters, std::string parameter, int &value);
    void parseParameter(const ParMap & parameters, std::string parameter, std::string &value);

public:

    GenericEngineModel() {};
    virtual ~GenericEngineModel() {};

    /**
     * Computes real vehicle acceleration given current speed, current acceleration,
     * and requested acceleration. Acceleration can be negative as well. The
     * model should handle decelerations as well
     *
     * @param[in] speed_mps current speed in meters per second
     * @param[in] accel_mps2 current acceleration in meters per squared second
     * @param[in] reqAccel_mps2 requested acceleration in meters per squared second
     * @param[in] timeStep current simulation timestep
     * @return the real acceleration that the vehicle applies in meters per
     * squared second
     */
    virtual double getRealAcceleration(double speed_mps, double accel_mps2, double reqAccel_mps2, int timeStep = 0) = 0;

    /**
     * Load model parameters. This method requires a map of strings to be as
     * flexible as possible, independently from the actual model implementation
     *
     * @param[in] parameters a map of strings (from parameter name to parameter
     * value) including configuration parameters
     */
    virtual void loadParameters(const ParMap &parameters) = 0;

    /**
     * Sets a single parameter value
     *
     * @param[in] parameter the name of the parameter
     * @param[in] value the value for the parameter
     */
    virtual void setParameter(const std::string parameter, const std::string &value) = 0;
    virtual void setParameter(const std::string parameter, double value) = 0;
    virtual void setParameter(const std::string parameter, int value) = 0;

};

#endif /* GENERICENGINEMODEL_H_ */
