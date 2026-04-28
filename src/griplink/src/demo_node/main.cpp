//=========================================================================
/**
 *  @file main.cpp
 *
 *  @section demo_node
 *
 *  @brief Main file for the demo node. Implementation of the demo program.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section main.cpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//=========================================================================

#include "griplink/demo_node/demo_node.hpp"


using namespace weiss_robotics;

void demo_home_service( std::shared_ptr<DemoNode> node, uint16_t port );
void demo_gripcfgset_service( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index );
void demo_grip_action( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index );
DeviceState demo_devstate_service( std::shared_ptr<DemoNode> node, uint16_t port );
uint32_t demo_value_service( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t value_index );
void demo_release_action( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index );


// ------------------------------------------------------------------------------------------------------------

// Demonstration to use the griplink node server:

// ------------------------------------------------------------------------------------------------------------


/**
 * @brief Demo program to use the gripper on port 0 of the GRIPLINK.
 * 
 * Start:
 * 1. Home service 					-> Initialize gripper.
 * 2. GripConfigSet service 		-> Set gripper configurations (finger positions, gripping force, ...).
 * 
 * Loop: 
 * 		3. Grip action				-> Grip and wait until it has finished.
 * 		4. DeviceState service		-> Get the device state (holding/no-part/fault).
 * 		If gripper is holding a part: 
 * 			5. Value service 		-> Get finger position in mm.
 * 			Wait for 5 seconds.
 * 		6. Release action			-> Release and wait until it has finished.
 * 
 * @param node ROS2 demo node.
 */
void demo_program( std::shared_ptr<DemoNode> node )
{
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Starting demo." );

	uint16_t port;
	uint16_t index;
	uint16_t value_index;
	uint32_t finger_pos;

	port = 0;
	index = 0;
	value_index = 0;

	demo_home_service( node, port );
	demo_gripcfgset_service( node, port, index );
	while ( true )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "- - - - - - - - - - - - - - - - - - - - - - -" );
		demo_grip_action( node, port, index );
		if ( demo_devstate_service( node, port ) == DeviceState::DS_HOLDING )
		{
			RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Gripper is holding a part. Requesting finger position and waiting for 5 seconds." );
			finger_pos = demo_value_service( node, port, value_index );
			RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Finger position: %.3f mm.", static_cast<float>( finger_pos ) / 1000 );
			rclcpp::spin_until_future_complete( node, std::promise<bool>().get_future(), std::chrono::seconds( 5 ) );
		}
		else
		{
			RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "No part found. Continuing program." );
			rclcpp::spin_until_future_complete( node, std::promise<bool>().get_future(), std::chrono::seconds( 1 ) );
		}
		demo_release_action( node, port, index );
	}
}


void demo_home_service( std::shared_ptr<DemoNode> node, uint16_t port )
{
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started Home Service with port: %u.", port );
	
	if ( node->home_service( port ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Home Service failed. Stopping demo." );
		throw DemoException( "Home Service failed." );
	}

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Home Service successful." );
}


void demo_gripcfgset_service( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index )
{
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started GripConfigSet Service with port: %u, index: %u.", port, index );

	if ( node->gripcfgset_service( port, index, "tag", 19000, 3800, 50000 ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "GripConfigSet Service failed. Stopping demo." );
		throw DemoException( "GripConfigSet Service failed." );
	}
	
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "GripConfigSet Service successful." );

}


void demo_grip_action( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index )
{
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started Grip Action with port: %u, index: %u.", port, index );

	if ( node->grip_action( port, index ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Grip Action failed. Stopping demo." );
		throw DemoException( "Grip Action failed." );
	}

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Grip Action successful." );
}


DeviceState demo_devstate_service( std::shared_ptr<DemoNode> node, uint16_t port )
{
	DeviceState device_state;

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started Devstate Service with port: %u.", port );

	if ( node->devstate_service( port, device_state ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Devstate Service failed. Stopping demo." );
		throw DemoException( "Devstate Service failed." );
	}

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Devstate Service successful with device state: %s.", DeviceStateStr( device_state ).c_str() );

	return device_state;
}


uint32_t demo_value_service( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t value_index )
{
	uint32_t value;

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started Value Service with port: %u, index: %u.", port, value_index );

	if ( node->value_service( port, value_index, value ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Value Service failed. Stopping demo." );
		throw DemoException( "Value Service failed." );
	}

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Value Service successful with value: %u.", value );

	return value;
}


void demo_release_action( std::shared_ptr<DemoNode> node, uint16_t port, uint16_t index )
{
	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Started Release Action with port: %u, index: %u.", port, index );

	if ( node->release_action( port, index ) != StatusCode::E_SUCCESS )
	{
		RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Release Action failed. Stopping demo." );
		throw DemoException( "Release Action failed." );
	}

	RCLCPP_INFO( rclcpp::get_logger( "Demo" ), "Release Action successful." );
}



int main( int argc, char const *argv[] )
{
	try
	{
		rclcpp::init( argc, argv );
		auto node = std::make_shared<DemoNode>();

		demo_program( node );

		rclcpp::shutdown();
		return 0;
	}
	catch ( DemoException &e )
	{
		RCLCPP_ERROR_STREAM( rclcpp::get_logger( "DemoNode" ), "Caught demo exception: " << e.what() );
      	rclcpp::shutdown();
		return 1;
	}
}
