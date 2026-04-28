//======================================================================
/**
 *  @file common.cpp
 *
 *  @section griplink_node
 *
 *  @brief Implementations of Exceptions, defined custom Griplink types. 
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section common.cpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//======================================================================

#include "griplink/griplink_node/common.hpp"


namespace weiss_robotics
{


/**
 * @brief Converts from StatusCode type to string representation.
 * 
 * @param code Type: StatusCode.
 * @return std::string 
 */
std::string StatusCodeStr( StatusCode code )
{
	switch ( code )
	{
		case StatusCode::E_SUCCESS: return "Command successfully executed";
		case StatusCode::E_OVERRUN: return "Data overrun";
		case StatusCode::E_RANGE_ERROR: return "Value out of range";
		case StatusCode::E_NOT_AVAILABLE: return "Function or data not available";
		case StatusCode::E_NOT_INITIALIZED: return "Device not initialized";
		case StatusCode::E_TIMEOUT: return "Timeout";
		case StatusCode::E_INSUFFICIENT_RESOURCES: return "Not enough memory available";
		case StatusCode::E_CHECKSUM_ERROR: return "Checksum error";
		case StatusCode::E_ACCESS_DENIED: return "Access denied";
		case StatusCode::E_INVALID_HANDLE: return "Invalid handle";
		case StatusCode::E_INVALID_PARAMETER: return "Invalid parameter";
		case StatusCode::E_INDEX_OUT_OF_BOUNDS: return "Index out of bounds";
		case StatusCode::E_IO_ERROR: return "Generic I/O error";
		case StatusCode::E_READ_ERROR: return "Read error";
		case StatusCode::E_WRITE_ERROR: return "Write error";
		case StatusCode::E_NOT_FOUND: return "Resource not found";
		case StatusCode::E_NOT_OPEN: return "File or device not open";
		case StatusCode::E_EXISTS: return "Resource already exists";
		case StatusCode::E_NO_COMM: return "Connection error";
		case StatusCode::E_STATE_CONFLICT: return "Invalid state";
		case StatusCode::E_NOT_SUPPORTED: return "Command or function not supported";
		case StatusCode::E_INCONSISTENT_DATA: return "Data inconsistent";
		case StatusCode::E_CMD_SYNTAX: return "Syntax error";
		case StatusCode::E_CMD_UNKNOWN: return "Unknown command";
		case StatusCode::E_CMD_ABORTED: return "Command aborted";
		case StatusCode::E_CMD_FAILED: return "Command failed";
		case StatusCode::E_AXIS_BLOCKED: return "Axis is blocked";
		case StatusCode::E_PENDING: return "Pending action";
		default: return "Unknown status code";
	}
}


/**
 * @brief Converts from DeviceState type to string representation.
 * 
 * @param state Type: DeviceState.
 * @return std::string
 */
std::string DeviceStateStr( DeviceState state )
{
	switch ( state )
	{
		case DeviceState::DS_NOT_CONNECTED: return "Not connected";
		case DeviceState::DS_NOT_INITIALIZED: return "Not initialized";
		case DeviceState::DS_DISABLED: return "Disabled";
		case DeviceState::DS_RELEASED: return "Released";
		case DeviceState::DS_NO_PART: return "No part";
		case DeviceState::DS_HOLDING: return "Holding";
		case DeviceState::DS_OPERATING: return "Operating";
		case DeviceState::DS_FAULT: return "Fault";
		default: return "Unknown device state";
	}
}


GriplinkException::GriplinkException( std::string const &message ) : message_( message ) {}

std::string GriplinkException::what()
{
    return message_.c_str();
}


} // namespace weiss_robotics
