//===================================================================================
/**
 *  @file demo_node.cpp
 *
 *  @section demo_node
 *
 *  @brief Implementations of topic, service and action clients of the griplink node.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section demo_node.cpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//===================================================================================

#include "griplink/demo_node/demo_node.hpp"


using namespace weiss_robotics;

/**
 * @brief Constructs a new Demo Node object.
 * 
 * Subscribes to the device_states topic of the griplink node.
 * Creates service clients and action clients.
 */
DemoNode::DemoNode() : Node( "demo_node" )
{
	using namespace std::placeholders;

	// Create device state topic subscription

	device_states_subscription_ = create_subscription<DeviceStates>( 
	  "/griplink_node/device_states", 
	  10, 
	  std::bind( &DemoNode::device_states_topic_callback, this, _1 ) 
	);
	
	// Create service clients

	home_client_ = create_client<Home>( "/griplink_node/home" );
	gripcfgset_client_ = create_client<Gripcfgset>( "/griplink_node/gripcfgset" );
	grip_client_ = create_client<Grip>( "/griplink_node/grip" );
	release_client_ = create_client<Release>( "/griplink_node/release" );
	value_client_ = create_client<Value>( "/griplink_node/value" );
	devstate_client_ = create_client<Devstate>( "/griplink_node/devstate" );

	// Create action clients

	grip_action_client_ = rclcpp_action::create_client<GripAction>( this, "/griplink_node/grip" );
	release_action_client_ = rclcpp_action::create_client<ReleaseAction>( this, "/griplink_node/release" );
}


// ------------------------------------------------------------------------------------------------------------

// Demo topic subscriber callback:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Callback of the device states topic subscriber.
 * 
 * @param msg Contains the published message.
 */
void DemoNode::device_states_topic_callback( const DeviceStates &msg )
{
	// Update device_states_ with subscription
	for ( uint16_t i=0; i<32; ++i )
	{
		device_states_[i] = static_cast<DeviceState>( msg.device_states[i] );
	}	
	RCLCPP_DEBUG( rclcpp::get_logger("device_states_"), "device_states_[0]=%u", static_cast<uint16_t>(device_states_[0]) );
}


// ------------------------------------------------------------------------------------------------------------

// Demo service clients:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Sends a request to the home service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::home_service( uint16_t port )
{
	while ( !home_client_->wait_for_service(std::chrono::seconds(1)) ) 
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}

	// Send a request to the service and wait for the response
	auto request = std::make_shared<Home::Request>();
	request->port = port;
	auto response_future = home_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		home_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	RCLCPP_DEBUG( get_logger(), "Home service response: status: %u, message %s", response->status, response->message.c_str() );

	return static_cast<StatusCode>( response->status );
}


/**
 * @brief Sends a request to the gripper configuration set service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @param index Recipe/preset index to be set.
 * @param tag String to name the specific recipe.
 * @param no_part_limit No-Part limit in micrometers.
 * @param release_limit Release limit in micrometers.
 * @param force_factor Force factor in percent multiplied by 1000.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::gripcfgset_service( uint16_t port, uint16_t index, std::string tag, uint32_t no_part_limit, uint32_t release_limit, uint32_t force_factor )
{
	while ( !gripcfgset_client_->wait_for_service(std::chrono::seconds(1)) )
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}

	// Send a request to the service and wait for the response
	auto request = std::make_shared<Gripcfgset::Request>();
	request->port = port;
	request->index = index;
	request->tag = tag;
	request->parameters = {no_part_limit, release_limit, force_factor, 0, 0, 0, 0, 0};
	auto response_future = gripcfgset_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		gripcfgset_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	RCLCPP_DEBUG( 
		get_logger(), 
		"Gripconfigset service response: status: %u, message %s, tag %s, parameters %u %u %u %u %u %u", 
		response->status, response->message.c_str(), response->tag.c_str(), 
		response->parameters[0], response->parameters[1], response->parameters[2], 
		response->parameters[3], response->parameters[4], response->parameters[5] 
	);

	return static_cast<StatusCode>( response->status );
}


/**
 * @brief Sends a requests to the grip service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @param index Recipe/preset index.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::grip_service( uint16_t port, uint16_t index )
{
	while ( !grip_client_->wait_for_service( std::chrono::seconds(1) ) ) 
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}
	
	// Send a request to the service and wait for the response
	auto request = std::make_shared<Grip::Request>();
	request->port = port;
	request->index = index;
	auto response_future = grip_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		grip_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	RCLCPP_DEBUG( get_logger(), "Grip service response: status: %u, message %s", response->status, response->message.c_str() );

	return static_cast<StatusCode>( response->status );
}


/**
 * @brief Sends a requests to the release service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @param index Recipe/preset index.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::release_service( uint16_t port, uint16_t index )
{
	while ( !release_client_->wait_for_service( std::chrono::seconds(1) ) ) 
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}
	
	// Send a request to the service and wait for the response
	auto request = std::make_shared<Release::Request>();
	request->port = port;
	request->index = index;
	auto response_future = release_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		release_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	RCLCPP_DEBUG( get_logger(), "Release service response: status: %u, message %s", response->status, response->message.c_str() );
	
	return static_cast<StatusCode>( response->status );
}


/**
 * @brief Sends a requests to the value service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @param index Index of the value to be read. For IEG and WPG Series, value at index 0 gives the finger position in mm.
 * @param value Stores the value response of the service.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::value_service( uint16_t port, uint16_t index, uint32_t &value )
{
	while ( !value_client_->wait_for_service( std::chrono::seconds(1) ) ) 
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}
	
	// Send a request to the service and wait for the response
	auto request = std::make_shared<Value::Request>();
	request->port = port;
	request->index = index;
	auto response_future = value_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		value_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	value = response->value;

	RCLCPP_DEBUG( get_logger(), "Value service response: status: %u, message %s, value %u", response->status, response->message.c_str(), response->value );
	
	return static_cast<StatusCode>( response->status );
}


/**
 * @brief Sends a requests to the device state service and acquires the server response.
 * 
 * @param port GRIPLINK port, at which the service should be performed.
 * @param devstate Stores the device state response of the service.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::devstate_service( uint16_t port, DeviceState &devstate )
{
	while ( !devstate_client_->wait_for_service(std::chrono::seconds(1)) ) 
	{
		if ( !rclcpp::ok() ) 
		{
			RCLCPP_ERROR( get_logger(), "Client interrupted while waiting for service to appear." );
			throw DemoException( "Client interrupted while waiting for service to appear." );
		}
		RCLCPP_DEBUG( get_logger(), "Waiting for service to appear..." );
	}
	
	// Send a request to the service and wait for the response
	auto request = std::make_shared<Devstate::Request>();
	request->port = port;
	auto response_future = devstate_client_->async_send_request( request );

	// Spin node until service responds
	if ( rclcpp::spin_until_future_complete( shared_from_this(), response_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Service call failed :(" );
		devstate_client_->remove_pending_request( response_future );
		throw DemoException( "Service call failed." );
	}

	// Get the response
	auto response = response_future.get();

	devstate = static_cast<DeviceState>( response->state );
	
	RCLCPP_DEBUG( get_logger(), "Devstate service response: status: %u, message %s, devstate: %u", response->status, response->message.c_str(), response->state );

	return static_cast<StatusCode>( response->status );
}


// ------------------------------------------------------------------------------------------------------------

// Demo action clients:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Sends a goal to the grip action and acquires the feedbacks and the result from the action server.
 * 
 * @param port GRIPLINK port, at which the action should be performed.
 * @param index Recipe/preset index.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::grip_action( uint16_t port, uint16_t index )
{
	using namespace std::placeholders;

	if ( !grip_action_client_->wait_for_action_server() ) 
	{
		RCLCPP_ERROR( get_logger(), "Action server not available after waiting" );
		rclcpp::shutdown();
	}

	// Set goal arguments and send it to the action server
	auto goal_msg = GripAction::Goal();
	goal_msg.port = port;
	goal_msg.index = index;
	RCLCPP_DEBUG( get_logger(), "Sending goal" );
	auto goal_handle_future = grip_action_client_->async_send_goal( goal_msg );

	// Spin the node until action server responds to the goal request
	if ( rclcpp::spin_until_future_complete( shared_from_this(), goal_handle_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Send goal call failed :(" );
		throw DemoException( "Send goal call failed." );
	}

	// Check whether the goal was accepted
	rclcpp_action::ClientGoalHandle<GripAction>::SharedPtr goal_handle = goal_handle_future.get();
	if ( !goal_handle ) 
	{
		RCLCPP_ERROR( get_logger(), "Goal was rejected by server" );
		throw DemoException( "Goal was rejected by server." );
	}

	// Wait for action server to finish the goal
	auto result_future = grip_action_client_->async_get_result( goal_handle );
	RCLCPP_DEBUG( get_logger(), "Waiting for result" );
	if ( rclcpp::spin_until_future_complete( shared_from_this(), result_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Get result call failed :(" );
		throw DemoException( "Get result call failed." );
	} 

	// Get the result from the response
	rclcpp_action::ClientGoalHandle<GripAction>::WrappedResult wrapped_result = result_future.get();
	
	switch ( wrapped_result.code ) 
	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR( get_logger(), "Goal was aborted" );
			throw DemoException( "Goal was aborted." );
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR( get_logger(), "Goal was canceled" );
			throw DemoException( "Goal was canceled." );
		default:
			RCLCPP_ERROR( get_logger(), "Unknown result code" );
			throw DemoException( "Unknown result code." );
	}

	RCLCPP_DEBUG( get_logger(), "Result received: status %u, message %s", wrapped_result.result->status, wrapped_result.result->message.c_str() );

	return static_cast<StatusCode>( wrapped_result.result->status );
}


/**
 * @brief Sends a goal to the release action and acquires the feedbacks and the result from the action server.
 * 
 * @param port GRIPLINK port, at which the action should be performed.
 * @param index Recipe/preset index.
 * @return StatusCode: Status response of the server. 
 */
StatusCode DemoNode::release_action( uint16_t port, uint16_t index )
{
	using namespace std::placeholders;

	if ( !release_action_client_->wait_for_action_server() ) 
	{
		RCLCPP_ERROR( get_logger(), "Action server not available after waiting" );
		rclcpp::shutdown();
	}

	// Set goal arguments and send it to the action server
	auto goal_msg = ReleaseAction::Goal();
	goal_msg.port = port;
	goal_msg.index = index;
	RCLCPP_DEBUG( get_logger(), "Sending goal" );
	auto goal_handle_future = release_action_client_->async_send_goal( goal_msg );

	// Spin the node until action server responds to the goal request
	if ( rclcpp::spin_until_future_complete( shared_from_this(), goal_handle_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Send goal call failed :(" );
		throw DemoException( "Send goal call failed." );
	}

	// Check whether the goal was accepted
	rclcpp_action::ClientGoalHandle<ReleaseAction>::SharedPtr goal_handle = goal_handle_future.get();
	if ( !goal_handle ) 
	{
		RCLCPP_ERROR( get_logger(), "Goal was rejected by server" );
		throw DemoException( "Goal was rejected by server." );
	}

	// Wait for action server to finish the goal
	auto result_future = release_action_client_->async_get_result( goal_handle );

	RCLCPP_DEBUG( get_logger(), "Waiting for result" );
	if ( rclcpp::spin_until_future_complete( shared_from_this(), result_future ) != rclcpp::FutureReturnCode::SUCCESS )
	{
		RCLCPP_ERROR( get_logger(), "Get result call failed :(" );
		throw DemoException( "Get result call failed." );
	} 

	// Get the result from the response
	rclcpp_action::ClientGoalHandle<ReleaseAction>::WrappedResult wrapped_result = result_future.get();
	
	switch ( wrapped_result.code ) 
	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR( get_logger(), "Goal was aborted" );
			throw DemoException( "Goal was aborted." );
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR( get_logger(), "Goal was canceled" );
			throw DemoException( "Goal was canceled." );
		default:
			RCLCPP_ERROR( get_logger(), "Unknown result code" );
			throw DemoException( "Unknown result code." );
	}

	RCLCPP_DEBUG( get_logger(), "Result received: status %u, message %s", wrapped_result.result->status, wrapped_result.result->message.c_str() );

	return static_cast<StatusCode>( wrapped_result.result->status );
}


// ------------------------------------------------------------------------------------------------------------

// Misc:

// ------------------------------------------------------------------------------------------------------------

DemoException::DemoException( std::string const &message ) : message_( message ) {}

std::string DemoException::what()
{
    return message_.c_str();
}

