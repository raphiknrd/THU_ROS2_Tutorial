//======================================================================
/**
 *  @file common.hpp
 *
 *  @section griplink_node
 *
 *  @brief Definitions of custom exceptions and types. 
 * 		   This file should be included by the client.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section common.hpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//======================================================================

#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>


namespace weiss_robotics
{


enum class StatusCode : uint16_t
{
	E_SUCCESS = 0,
	E_OVERRUN = 1,
	E_RANGE_ERROR = 2,
	E_NOT_AVAILABLE = 3,
	E_NOT_INITIALIZED = 4,
	E_TIMEOUT = 5,
	E_INSUFFICIENT_RESOURCES = 6,
	E_CHECKSUM_ERROR = 7,
	E_ACCESS_DENIED = 8,
	E_INVALID_HANDLE = 9,
	E_INVALID_PARAMETER = 10,
	E_INDEX_OUT_OF_BOUNDS = 11,
	E_IO_ERROR = 12,
	E_READ_ERROR = 13,
	E_WRITE_ERROR = 14,
	E_NOT_FOUND = 15,
	E_NOT_OPEN = 16,
	E_EXISTS = 17,
	E_NO_COMM = 18,
	E_STATE_CONFLICT = 19,
	E_NOT_SUPPORTED = 20,
	E_INCONSISTENT_DATA = 21,
	E_CMD_SYNTAX = 22,
	E_CMD_UNKNOWN = 23,
	E_CMD_ABORTED = 24,
	E_CMD_FAILED = 25,
	E_AXIS_BLOCKED = 26,
	E_PENDING = 27
};

std::string StatusCodeStr( StatusCode code );

enum class DeviceState : uint16_t
{
	DS_NOT_CONNECTED = 0,
	DS_NOT_INITIALIZED = 1,
	DS_DISABLED = 2,
	DS_RELEASED = 3,
	DS_NO_PART = 4,
	DS_HOLDING = 5,
	DS_OPERATING = 6,
	DS_FAULT = 7,
};

std::string DeviceStateStr( DeviceState state );

// Griplink exception

class GriplinkException : public std::exception
{
    public:

    	GriplinkException( std::string const &message );

    	std::string what();

    private:

		std::string const message_;
};


} // namespace weiss_robotics

#endif // COMMON_HPP