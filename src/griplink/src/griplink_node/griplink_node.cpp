//==================================================================================
/**
 *  @file griplink_node.cpp
 *
 *  @section griplink_node
 *
 *  @brief Implementations of ROS2 topics, services and actions of the griplink node.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section griplink_node.cpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//==================================================================================

#include <griplink/griplink_node/griplink_node.hpp>
namespace weiss_robotics
{

/**
 * @brief Constructs a new Griplink Node object
 * 
 * Reads node parameters, constructs an object of the griplink class.
 * Initializes the device_states array, then updates and publishes its current state to a topic.
 * Creates a ROS2 multithreaded callback group, a publisher, services, and actions servers.
 */
GriplinkNode::GriplinkNode() : rclcpp::Node( "griplink_node" )
{
	using namespace std::placeholders;

	//  Handle node parameters

	std::string ip;
	uint16_t port;

	declare_parameter<std::string>( "ip", "192.168.1.40" );
	declare_parameter<uint16_t>( "port", 10001 );

	get_parameter( "ip", ip );
	get_parameter( "port", port );

	// Create a griplink object

	griplink_ = std::make_unique<Griplink>( ip, port );

	//  Create a callback group for multithreading

	callback_group_ = create_callback_group( rclcpp::CallbackGroupType::Reentrant );

	// Set up device states topic

	device_states_publisher_ = this->create_publisher<DeviceStates>( "device_states", 10 );
	initialize_device_states();
	using namespace std::chrono_literals;
	device_states_timer_ = create_wall_timer( 
		10ms, 
		std::bind( &GriplinkNode::update_device_states, this ), 
		callback_group_ 
	);

	// Create services

	id_srv_ = create_service<Id>( 
		"id", 
		std::bind( &GriplinkNode::id, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	protocol_srv_ = create_service<Protocol>( 
		"protocol", 
		std::bind( &GriplinkNode::protocol, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);
	
	protassert_srv_ = create_service<Protassert>( 
		"protassert", 
		std::bind( &GriplinkNode::protassert, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);
	
	sn_srv_ = create_service<Sn>( 
		"sn", 
		std::bind( &GriplinkNode::sn, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	labelget_srv_ = create_service<Labelget>( 
		"labelget", 
		std::bind( &GriplinkNode::labelget, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	labelset_srv_ = create_service<Labelset>( 
		"labelset", 
		std::bind( &GriplinkNode::labelset, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	ver_srv_ = create_service<Ver>( 
		"ver", 
		std::bind( &GriplinkNode::ver, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	verbose_srv_= create_service<Verbose>( 
		"verbose", 
		std::bind( &GriplinkNode::verbose, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devvid_srv_ = create_service<Devvid>( 
		"devvid", 
		std::bind( &GriplinkNode::devvid, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devpid_srv_ = create_service<Devpid>( 
		"devpid", 
		std::bind( &GriplinkNode::devpid, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devassert_srv_ = create_service<Devassert>( 
		"devassert", 
		std::bind( &GriplinkNode::devassert, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devname_srv_ = create_service<Devname>( 
		"devname", 
		std::bind( &GriplinkNode::devname, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devvendor_srv_ = create_service<Devvendor>( 
		"devvendor", 
		std::bind( &GriplinkNode::devvendor, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devsn_srv_ = create_service<Devsn>( 
		"devsn", 
		std::bind( &GriplinkNode::devsn, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devtagget_srv_ = create_service<Devtagget>( 
		"devtagget", 
		std::bind( &GriplinkNode::devtagget, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devtagset_srv_ = create_service<Devtagset>( 
		"devtagset", 
		std::bind( &GriplinkNode::devtagset, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devver_srv_ = create_service<Devver>( 
		"devver", 
		std::bind( &GriplinkNode::devver, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	bye_srv_ = create_service<Bye>( 
		"bye", 
		std::bind( &GriplinkNode::bye, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	enable_srv_ = create_service<Enable>( 
		"enable", 
		std::bind( &GriplinkNode::enable, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	disable_srv_ = create_service<Disable>( 
		"disable", 
		std::bind( &GriplinkNode::disable, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	home_srv_ = create_service<Home>( 
		"home", 
		std::bind( &GriplinkNode::home, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	grip_srv_ = create_service<Grip>( 
		"grip", 
		std::bind( &GriplinkNode::grip, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	release_srv_ = create_service<Release>( 
		"release", 
		std::bind( &GriplinkNode::release, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	flexgrip_srv_ = create_service<Flexgrip>( 
		"flexgrip", 
		std::bind( &GriplinkNode::flexgrip, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	flexrelease_srv_ = create_service<Flexrelease>( 
		"flexrelease", 
		std::bind( &GriplinkNode::flexrelease, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	led_srv_ = create_service<Led>( 
		"led", 
		std::bind( &GriplinkNode::led, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	clamp_srv_ = create_service<Clamp>( 
		"clamp", 
		std::bind( &GriplinkNode::clamp, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	wstr_srv_ = create_service<Wstr>( 
		"wstr", 
		std::bind( &GriplinkNode::wstr, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	setval_srv_ = create_service<Setval>( 
		"setval", 
		std::bind( &GriplinkNode::setval, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	waitval_srv_ = create_service<Waitval>( 
		"waitval", 
		std::bind( &GriplinkNode::waitval, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	devstate_srv_ = create_service<Devstate>( 
		"devstate", 
		std::bind( &GriplinkNode::devstate, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	value_srv_ = create_service<Value>( 
		"value", 
		std::bind( &GriplinkNode::value, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	gripcfgget_srv_ = create_service<Gripcfgget>( 
		"gripcfgget", 
		std::bind( &GriplinkNode::gripcfgget, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	gripcfgset_srv_ = create_service<Gripcfgset>( 
		"gripcfgset", 
		std::bind( &GriplinkNode::gripcfgset, this, _1, _2 ), 
		rmw_qos_profile_services_default, 
		callback_group_ 
	);

	// Create action servers

	grip_action_server_ = rclcpp_action::create_server<GripAction>(
	  this,
	  "grip",
	  std::bind( &GriplinkNode::grip_handle_goal, this, _1, _2 ),
	  std::bind( &GriplinkNode::grip_handle_cancel, this, _1 ),
	  std::bind( &GriplinkNode::grip_handle_accepted, this, _1 ),
	  rcl_action_server_get_default_options(),
	  callback_group_
	);

	release_action_server_ = rclcpp_action::create_server<ReleaseAction>(
	  this,
	  "release",
	  std::bind( &GriplinkNode::release_handle_goal, this, _1, _2 ),
	  std::bind( &GriplinkNode::release_handle_cancel, this, _1 ),
	  std::bind( &GriplinkNode::release_handle_accepted, this, _1 ),
	  rcl_action_server_get_default_options(),
	  callback_group_
	);

	flexgrip_action_server_ = rclcpp_action::create_server<FlexgripAction>(
	  this,
	  "flexgrip",
	  std::bind( &GriplinkNode::flexgrip_handle_goal, this, _1, _2 ),
	  std::bind( &GriplinkNode::flexgrip_handle_cancel, this, _1 ),
	  std::bind( &GriplinkNode::flexgrip_handle_accepted, this, _1 ),
	  rcl_action_server_get_default_options(),
	  callback_group_
	);

	flexrelease_action_server_ = rclcpp_action::create_server<FlexreleaseAction>(
	  this,
	  "flexrelease",
	  std::bind( &GriplinkNode::flexrelease_handle_goal, this, _1, _2 ),
	  std::bind( &GriplinkNode::flexrelease_handle_cancel, this, _1 ),
	  std::bind( &GriplinkNode::flexrelease_handle_accepted, this, _1 ),
	  rcl_action_server_get_default_options(),
	  callback_group_
	);
}


// ------------------------------------------------------------------------------------------------------------

// Device States Update and Topic Implementations:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the device states array to filter out ports with no device connected to it. 
 * 
 * This function is called once at startup of the griplink node. It checks for not connected devices.
 */
void GriplinkNode::initialize_device_states()
{
	for ( uint16_t i = 0; i < 32; ++i )
	{
		std::string response;
		StatusCode status;

		max_number_of_devices_ = 32;

		status = griplink_->devstate( response, i );

		// Skip all devstate requests with bigger port numbers upon encountering an E_INDEX_OUT_OF_BOUNDS error on port i
		if ( status == StatusCode::E_INDEX_OUT_OF_BOUNDS )
		{
			max_number_of_devices_ = i;
			for ( uint16_t j = i; j < 32; ++j )
			{
				device_states_[j] = DeviceState::DS_NOT_CONNECTED;
			}
			break;
		}

		// Set the device state of device i in the device_states_ array
		if ( status == StatusCode::E_SUCCESS && static_cast<DeviceState>( std::stoul( response ) ) != DeviceState::DS_NOT_CONNECTED )
		{
			device_states_[i] = static_cast<DeviceState>( std::stoul( response ) );
		}
		else
		{
			device_states_[i] = DeviceState::DS_NOT_CONNECTED;
		}

		RCLCPP_DEBUG( get_logger(), "Initial device state at port %u is %u.", 
			i,
			static_cast<uint16_t>( device_states_[i] ) 
		);
	}
}


/**
 * @brief Update the device states and publish to topic.
 */
void GriplinkNode::update_device_states()
{
	std::string response;
	StatusCode status;
	DeviceStates message;

	for ( uint16_t i = 0; i < max_number_of_devices_; ++i )
	{	
		// Skip the update of all initially not connected devices
		if ( device_states_[i] != DeviceState::DS_NOT_CONNECTED )
		{
			status = griplink_->devstate( response, i );

			if ( status == StatusCode::E_SUCCESS )
			{
				// Update the device state of device i
				device_states_[i] = static_cast<DeviceState>( std::stoul( response ) );
			}
		}

		// Set the device state of the not connected device to 0
		message.device_states.push_back( static_cast<uint16_t>( device_states_[i] ) );
	}

	// Publish all device states to the topic
	device_states_publisher_->publish( message );
}	


// ------------------------------------------------------------------------------------------------------------

// Action Implementations: 

// ------------------------------------------------------------------------------------------------------------

// ************************************ Grip Action ************************************ //

/**
 * @brief Accepts the action goal.
 */
rclcpp_action::GoalResponse GriplinkNode::grip_handle_goal( const rclcpp_action::GoalUUID & uuid, 
															std::shared_ptr<const GripAction::Goal> goal )
{
	RCLCPP_DEBUG( get_logger(), "Received goal request with port %u and index %u", goal->port, goal->index );
	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


/**
 * @brief Cancels the action. 
 */
rclcpp_action::CancelResponse GriplinkNode::grip_handle_cancel( const std::shared_ptr<GoalHandleGripAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Received request to cancel goal" );
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}


/**
 * @brief Executes the grip action.
 * 
 * Performs a grip[port], waits for state transition of the device. 
 * Upon state transition, the action is completed and returns results: status, message, device state.
 * 
 * Action Feedback: device state[port]. Feedback rate: 100 Hz.
 * 
 * @param goal_handle ROS2 action variable.
 */
void GriplinkNode::grip_handle_accepted( const std::shared_ptr<GoalHandleGripAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Executing goal");

	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<GripAction::Feedback>();
	auto result = std::make_shared<GripAction::Result>();

	rclcpp::Rate loop_rate( 100 );
	DeviceState device_state;
	
	// Perform a grip, then set the result to the status and message of the grip command response.
	StatusCode status = griplink_->grip( goal->port, goal->index );
	result->status = static_cast<uint16_t>( status );
	result->message = StatusCodeStr( status );

	// Check whether grip was a success
	if ( status != StatusCode::E_SUCCESS )
	{
		goal_handle->abort( result );
		RCLCPP_DEBUG( get_logger(), "Goal aborted: Grip command failed" );
		return;
	}

	// Start timer for the timeout
	using namespace std::chrono_literals;
	auto start = std::chrono::steady_clock::now();

	// Loop to find out when the state transits to holding or no-part state after starting to perform grip
	do
	{
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = now - start;

		// Check if the timeout was reached
		if ( elapsed_seconds > 15s )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Goal aborted: Timeout. Holding or No Part States not reached" );
			return;
		}

		// Check if there is a cancel request
		if ( goal_handle->is_canceling() )
		{
			goal_handle->canceled( result );
			RCLCPP_DEBUG( get_logger(), "Goal canceled" );
			return;
		}

		// Get the current device states array and update feedback and result
		device_state = device_states_[goal->port];
		feedback->device_state = static_cast<uint16_t>( device_state );
		result->device_state = static_cast<uint16_t>( device_state );

		// Check whether device is in fault state
		if ( device_state == DeviceState::DS_FAULT )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Device state is DS_FAULT" );
			return;
		}

		// Publish updated feedback
		goal_handle->publish_feedback( feedback );
		RCLCPP_DEBUG( get_logger(), "Publish feedback" );

		loop_rate.sleep();
	}
	while ( device_state != DeviceState::DS_NO_PART && device_state != DeviceState::DS_HOLDING && rclcpp::ok() );

	// Check if goal is done
	if ( rclcpp::ok() )
	{
		goal_handle->succeed( result );
		RCLCPP_DEBUG( get_logger(), "Goal succeeded" );
	}
}


// ************************************ Release Action ************************************ //

/**
 * @brief Accepts the action goal.
 */
rclcpp_action::GoalResponse GriplinkNode::release_handle_goal( const rclcpp_action::GoalUUID & uuid, 
															   std::shared_ptr<const ReleaseAction::Goal> goal )
{
	RCLCPP_DEBUG( get_logger(), "Received goal request with port %u and index %u", goal->port, goal->index );
	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


/**
 * @brief Cancels the action. 
 */
rclcpp_action::CancelResponse GriplinkNode::release_handle_cancel( const std::shared_ptr<GoalHandleReleaseAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Received request to cancel goal" );
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}


/**
 * @brief Executes the release Action.
 * 
 * Performs a release[port], waits for state transition of the device. 
 * Upon state transition, the action is completed and returns results: status, message, device state.
 * 
 * Action Feedback: device state[port]. Feedback rate: 100 Hz.
 * 
 * @param goal_handle ROS2 action variable.
 */
void GriplinkNode::release_handle_accepted( const std::shared_ptr<GoalHandleReleaseAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Executing goal");

	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<ReleaseAction::Feedback>();
	auto result = std::make_shared<ReleaseAction::Result>();

	rclcpp::Rate loop_rate( 100 );
	DeviceState device_state;

	// Perform a release, then set the result to the status and message of the release command response.
	StatusCode status = griplink_->release( goal->port, goal->index );
	result->status = static_cast<uint16_t>( status );
	result->message = StatusCodeStr( status );

	// Check whether release was a success
	if ( status != StatusCode::E_SUCCESS )
	{
		goal_handle->abort( result );
		RCLCPP_DEBUG( get_logger(), "Goal aborted: Release command failed" );
		return;
	}

	// Start timer for the timeout
	using namespace std::chrono_literals;
	auto start = std::chrono::steady_clock::now();

	// Loop to find out when the state transits to holding or no-part state after starting to perform release
	do
	{
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = now - start;

		// Check if the timeout was reached
		if ( elapsed_seconds > 15s )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Goal aborted: Timeout. Holding or No Part States not reached" );
			return;
		}

		// Check if there is a cancel request
		if ( goal_handle->is_canceling() )
		{
			goal_handle->canceled( result );
			RCLCPP_DEBUG( get_logger(), "Goal canceled" );
			return;
		}

		// Get the current device states array and update feedback and result
		device_state = device_states_[goal->port];
		feedback->device_state = static_cast<uint16_t>( device_state );
		result->device_state = static_cast<uint16_t>( device_state );

		// Check whether device is in fault state
		if ( device_state == DeviceState::DS_FAULT )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Device state is DS_FAULT" );
			return;
		}

		// Publish updated feedback
		goal_handle->publish_feedback( feedback );
		RCLCPP_DEBUG( get_logger(), "Publish feedback" );

		loop_rate.sleep();
	}
	while ( device_state != DeviceState::DS_RELEASED && rclcpp::ok() );

	// Check if goal is done
	if ( rclcpp::ok() )
	{
		goal_handle->succeed( result );
		RCLCPP_DEBUG( get_logger(), "Goal succeeded" );
	}
}


// ************************************ Flexgrip Action ************************************ //

/**
 * @brief Accepts the action goal.
 */
rclcpp_action::GoalResponse GriplinkNode::flexgrip_handle_goal( const rclcpp_action::GoalUUID & uuid, 
																std::shared_ptr<const FlexgripAction::Goal> goal )
{
	RCLCPP_DEBUG( 
		get_logger(), 
		"Received goal request with port %u, position %u, force %u, speed %u, acceleration %u", 
		goal->port, goal->position, goal->force, goal->speed, goal->acceleration 
	);
	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


/**
 * @brief Cancels the action. 
 */
rclcpp_action::CancelResponse GriplinkNode::flexgrip_handle_cancel( const std::shared_ptr<GoalHandleFlexgripAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Received request to cancel goal" );
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}


/**
 * @brief Executes the flexgrip action.
 * 
 * Performs a flexgrip[port], waits for state transition of the device. 
 * Upon state transition, the action is completed and returns results: status, message, device state.
 * 
 * Action Feedback: device state[port]. Feedback rate: 100 Hz.
 * 
 * @param goal_handle ROS2 action variable.
 */
void GriplinkNode::flexgrip_handle_accepted( const std::shared_ptr<GoalHandleFlexgripAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Executing goal");

	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<FlexgripAction::Feedback>();
	auto result = std::make_shared<FlexgripAction::Result>();

	rclcpp::Rate loop_rate( 100 );
	DeviceState device_state;
	
	// Perform a flexgrip, then set the result to the status and message of the flexgrip command response.
	StatusCode status = griplink_->flexgrip( goal->port, goal->position, goal->force, goal->speed, goal->acceleration );
	result->status = static_cast<uint16_t>( status );
	result->message = StatusCodeStr( status );

	// Check whether flexgrip was a success
	if ( status != StatusCode::E_SUCCESS )
	{
		goal_handle->abort( result );
		RCLCPP_DEBUG( get_logger(), "Goal aborted: Flexgrip command failed" );
		return;
	}

	// Start timer for the timeout
	using namespace std::chrono_literals;
	auto start = std::chrono::steady_clock::now();

	// Loop to find out when the state transits to holding or no-part state after starting to perform flexgrip
	do
	{
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = now - start;

		// Check if the timeout was reached
		if ( elapsed_seconds > 15s )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Goal aborted: Timeout. Holding or No Part States not reached" );
			return;
		}

		// Check if there is a cancel request
		if ( goal_handle->is_canceling() )
		{
			goal_handle->canceled( result );
			RCLCPP_DEBUG( get_logger(), "Goal canceled" );
			return;
		}

		// Get the current device states array and update feedback and result
		device_state = device_states_[goal->port];
		feedback->device_state = static_cast<uint16_t>( device_state );
		result->device_state = static_cast<uint16_t>( device_state );

		// Check whether device is in fault state
		if ( device_state == DeviceState::DS_FAULT )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Device state is DS_FAULT" );
			return;
		}

		// Publish updated feedback
		goal_handle->publish_feedback( feedback );
		RCLCPP_DEBUG( get_logger(), "Publish feedback" );

		loop_rate.sleep();
	}
	while ( device_state != DeviceState::DS_NO_PART && device_state != DeviceState::DS_HOLDING && rclcpp::ok() );

	// Check if goal is done
	if ( rclcpp::ok() )
	{
		goal_handle->succeed( result );
		RCLCPP_DEBUG( get_logger(), "Goal succeeded" );
	}
}


// ************************************ Flexrelease Action ************************************ //

/**
 * @brief Accepts the action goal.
 */
rclcpp_action::GoalResponse GriplinkNode::flexrelease_handle_goal( const rclcpp_action::GoalUUID & uuid, 
																   std::shared_ptr<const FlexreleaseAction::Goal> goal )
{
	RCLCPP_DEBUG( 
		get_logger(), "Received goal request with port %u, position %u, speed %u, acceleration %u", 
		goal->port, 
		goal->position, 
		goal->speed, 
		goal->acceleration 
	);

	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


/**
 * @brief Cancels the action. 
 */
rclcpp_action::CancelResponse GriplinkNode::flexrelease_handle_cancel( const std::shared_ptr<GoalHandleFlexreleaseAction> goal_handle )
{
	RCLCPP_DEBUG( get_logger(), "Received request to cancel goal" );
	(void)goal_handle;
	return rclcpp_action::CancelResponse::ACCEPT;
}


/**
 * @brief Executes the flexrelease action.
 * 
 * Performs a flexrelease[port], waits for state transition of the device. 
 * Upon state transition, the action is completed and returns results: status, message, device state.
 * 
 * Action Feedback: device state[port]. Feedback rate: 100 Hz.
 * 
 * @param goal_handle ROS2 action variable.
 */
void GriplinkNode::flexrelease_handle_accepted( const std::shared_ptr<GoalHandleFlexreleaseAction> goal_handle )
{	
	RCLCPP_DEBUG( get_logger(), "Executing goal");

	const auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<FlexreleaseAction::Feedback>();
	auto result = std::make_shared<FlexreleaseAction::Result>();

	rclcpp::Rate loop_rate( 100 );
	DeviceState device_state;
	
	// Perform a flexrelease, then set the result to the status and message of the flexrelease command response.
	StatusCode status = griplink_->flexrelease( goal->port, goal->position, goal->speed, goal->acceleration );
	result->status = static_cast<uint16_t>( status );
	result->message = StatusCodeStr( status );

	// Check whether flexrelease was a success
	if ( status != StatusCode::E_SUCCESS )
	{
		goal_handle->abort( result );
		RCLCPP_DEBUG( get_logger(), "Goal aborted: Flexrelease command failed" );
		return;
	}

	// Start timer for the timeout
	using namespace std::chrono_literals;
	auto start = std::chrono::steady_clock::now();

	// Loop to find out when the state transits to holding or no-part state after starting to perform flexrelease
	do
	{
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = now - start;

		// Check if the timeout was reached
		if ( elapsed_seconds > 15s )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Goal aborted: Timeout. Holding or No Part States not reached" );
			return;
		}

		// Check if there is a cancel request
		if ( goal_handle->is_canceling() )
		{
			goal_handle->canceled( result );
			RCLCPP_DEBUG( get_logger(), "Goal canceled" );
			return;
		}

		// Get the current device states array and update feedback and result
		device_state = device_states_[goal->port];
		feedback->device_state = static_cast<uint16_t>( device_state );
		result->device_state = static_cast<uint16_t>( device_state );

		// Check whether device is in fault state
		if ( device_state == DeviceState::DS_FAULT )
		{
			goal_handle->abort( result );
			RCLCPP_DEBUG( get_logger(), "Device state is DS_FAULT" );
			return;
		}

		// Publish updated feedback
		goal_handle->publish_feedback( feedback );
		RCLCPP_DEBUG( get_logger(), "Publish feedback" );

		loop_rate.sleep();
	}
	while( device_state != DeviceState::DS_RELEASED && rclcpp::ok() );

	// Check if goal is done
	if ( rclcpp::ok() )
	{
		goal_handle->succeed( result );
		RCLCPP_DEBUG( get_logger(), "Goal succeeded" );
	}
}


// ------------------------------------------------------------------------------------------------------------

// GRIPLINK UNIFIED COMMAND SET - ROS2 Layer:

// ------------------------------------------------------------------------------------------------------------

void GriplinkNode::id( Id::Request::SharedPtr req, Id::Response::SharedPtr res )
{
	(void)req;
	
	std::string response;
	StatusCode status = griplink_->id( response );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->id = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->id = "";
	}
}

void GriplinkNode::protocol( Protocol::Request::SharedPtr req, Protocol::Response::SharedPtr res )
{
	(void)req;
	
	std::string response;
	StatusCode status = griplink_->protocol( response );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->protocol = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->protocol = "";
	}
}

void GriplinkNode::protassert( Protassert::Request::SharedPtr req, Protassert::Response::SharedPtr res )
{
	StatusCode status = griplink_->protassert( req->protocol, req->version );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::sn( Sn::Request::SharedPtr req, Sn::Response::SharedPtr res )
{
	(void)req;
	
	std::string response;
	StatusCode status = griplink_->sn( response );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->serial = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->serial = "";
	}
}

void GriplinkNode::labelget( Labelget::Request::SharedPtr req, Labelget::Response::SharedPtr res )
{
	(void)req;

	std::string response;
	StatusCode status = griplink_->labelget( response );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->label = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->label = "";
	}
}

void GriplinkNode::labelset( Labelset::Request::SharedPtr req, Labelset::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->labelset( response, req->label );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->label = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->label = "";
	}
}

void GriplinkNode::ver( Ver::Request::SharedPtr req, Ver::Response::SharedPtr res )
{
	(void)req;
	
	std::string response;
	StatusCode status = griplink_->ver( response );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->version = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->version = "";
	}
}

void GriplinkNode::verbose( Verbose::Request::SharedPtr req, Verbose::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->verbose( response, req->enable );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		res->enable = std::stoul( response );
	}
	else
	{
		res->enable = 0;
	}
}

void GriplinkNode::devvid( Devvid::Request::SharedPtr req, Devvid::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devvid( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		res->vid = std::stoul( response );
	}
	else
	{
		res->vid = 0;
	}
}

void GriplinkNode::devpid( Devpid::Request::SharedPtr req, Devpid::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devpid( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );

	if ( status == StatusCode::E_SUCCESS )
	{
		res->pid = std::stoul( response );
	}
	else
	{
		res->pid = 0;
	}
}

void GriplinkNode::devassert( Devassert::Request::SharedPtr req, Devassert::Response::SharedPtr res )
{
	StatusCode status = griplink_->devassert( req->port, req->vid, req->pid );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::devname( Devname::Request::SharedPtr req, Devname::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devname( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->name = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->name = "";
	}
}

void GriplinkNode::devvendor( Devvendor::Request::SharedPtr req, Devvendor::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devvendor( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->vendor = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->vendor = "";
	}
}

void GriplinkNode::devsn( Devsn::Request::SharedPtr req, Devsn::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devsn( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->serial = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->serial = "";
	}
}

void GriplinkNode::devtagget( Devtagget::Request::SharedPtr req, Devtagget::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devtagget( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->tag = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->tag = "";
	}
}

void GriplinkNode::devtagset( Devtagset::Request::SharedPtr req, Devtagset::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devtagset( response, req->port, req->tag );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->tag = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->tag = "";
	}
}

void GriplinkNode::devver( Devver::Request::SharedPtr req, Devver::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devver( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		res->version = response.substr( 1, response.size() - 2 );
	}
	else
	{
		res->version = "";
	}
}

void GriplinkNode::bye( Bye::Request::SharedPtr req, Bye::Response::SharedPtr res )
{
	(void)req;
	
	StatusCode status = griplink_->bye();

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::enable( Enable::Request::SharedPtr req, Enable::Response::SharedPtr res )
{
	StatusCode status = griplink_->enable( req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::disable( Disable::Request::SharedPtr req, Disable::Response::SharedPtr res )
{
	StatusCode status = griplink_->disable( req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::home( Home::Request::SharedPtr req, Home::Response::SharedPtr res )
{
	StatusCode status = griplink_->home( req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::grip( Grip::Request::SharedPtr req, Grip::Response::SharedPtr res )
{
	StatusCode status = griplink_->grip( req->port, req->index );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::release( Release::Request::SharedPtr req, Release::Response::SharedPtr res )
{
	StatusCode status = griplink_->release( req->port, req->index );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::flexgrip( Flexgrip::Request::SharedPtr req, Flexgrip::Response::SharedPtr res )
{
	StatusCode status = griplink_->flexgrip( req->port, req->position, req->force, req->speed, req->acceleration );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::flexrelease( Flexrelease::Request::SharedPtr req, Flexrelease::Response::SharedPtr res )
{
	StatusCode status = griplink_->flexrelease( req->port, req->position, req->speed, req->acceleration );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::led( Led::Request::SharedPtr req, Led::Response::SharedPtr res )
{
	StatusCode status = griplink_->led( req->port, req->index );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::clamp( Clamp::Request::SharedPtr req, Clamp::Response::SharedPtr res )
{
	StatusCode status = griplink_->clamp( req->port, req->enable );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::wstr( Wstr::Request::SharedPtr req, Wstr::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->wstr( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		res->state = std::stoul( response );
	}
	else
	{
		res->state = 0;
	}
}

void GriplinkNode::setval( Setval::Request::SharedPtr req, Setval::Response::SharedPtr res )
{
	StatusCode status = griplink_->setval( req->port, req->index, req->value );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::waitval( Waitval::Request::SharedPtr req, Waitval::Response::SharedPtr res )
{
	StatusCode status = griplink_->waitval( req->port, req->index, req->threshold, req->window, req->timeout );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
}

void GriplinkNode::devstate( Devstate::Request::SharedPtr req, Devstate::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->devstate( response, req->port );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		res->state = std::stoul( response );
	}
	else
	{
		res->state = 0;
	}
}

void GriplinkNode::value( Value::Request::SharedPtr req, Value::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->value( response, req->port, req->index );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		res->value = std::stoul( response );
	}
	else
	{
		res->value = 0;
	}
}

void GriplinkNode::gripcfgget( Gripcfgget::Request::SharedPtr req, Gripcfgget::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->gripcfgget( response, req->port, req->index );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		response = response.substr( 1, response.size() - 2 );

		// Split string response into a string (tag) and a uint32 array (parameters)

		std::stringstream test( response );
		std::string segment;
		std::vector<std::string> seglist;

		while ( std::getline( test, segment, ',' ) )
		{
			seglist.push_back( segment );
		}

		res->tag = seglist[0].substr( 1, seglist[0].size() - 2 );

		for ( int i=0; i<8; i++ )
		{
			res->parameters[i] = std::stoul( seglist[i+1] );
		}
	}
	else
	{
		res->tag = "";
		res->parameters = {0, 0, 0, 0, 0, 0, 0, 0};
	}
}

void GriplinkNode::gripcfgset( Gripcfgset::Request::SharedPtr req, Gripcfgset::Response::SharedPtr res )
{
	std::string response;
	StatusCode status = griplink_->gripcfgset( response, req->port, req->index, req->tag, req->parameters );

	res->status = static_cast<uint16_t>( status );
	res->message = StatusCodeStr( status );
	
	if ( status == StatusCode::E_SUCCESS )
	{
		// Remove quotation marks from string
		response = response.substr( 1, response.size() - 2 );

		// Split string response into a string (tag) and a uint32 array (parameters)

		std::stringstream test( response );
		std::string segment;
		std::vector<std::string> seglist;

		while ( std::getline(test, segment, ',') )
		{
			seglist.push_back(segment);
		}

		res->tag = seglist[0].substr( 1, seglist[0].size() - 2 );

		for ( int i=0; i<8; i++ )
		{
			res->parameters[i] = std::stoul( seglist[i+1] );
		}
	}
	else
	{
		res->tag = "";
		res->parameters = {0, 0, 0, 0, 0, 0, 0, 0};
	}
}

} // namespace weiss_robotics