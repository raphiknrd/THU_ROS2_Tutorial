//======================================================================
/**
 *  @file demo_node.hpp
 *
 *  @section demo_node
 *
 *  @brief The demo node class.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section demo_node.hpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//======================================================================

#ifndef DEMO_NODE_HPP
#define DEMO_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/wait_for_message.hpp>

#include "griplink/griplink_node/common.hpp"

#include "griplink_interfaces/msg/device_states.hpp"

#include "griplink_interfaces/srv/home.hpp"
#include "griplink_interfaces/srv/gripcfgset.hpp"
#include "griplink_interfaces/srv/grip.hpp"
#include "griplink_interfaces/srv/release.hpp"
#include "griplink_interfaces/srv/value.hpp"
#include "griplink_interfaces/srv/devstate.hpp"

#include "griplink_interfaces/action/grip.hpp"
#include "griplink_interfaces/action/release.hpp"

using namespace weiss_robotics;

using DeviceStates = griplink_interfaces::msg::DeviceStates;

using Home = griplink_interfaces::srv::Home;
using Gripcfgset = griplink_interfaces::srv::Gripcfgset;
using Grip = griplink_interfaces::srv::Grip;
using Release = griplink_interfaces::srv::Release;
using Value = griplink_interfaces::srv::Value;
using Devstate = griplink_interfaces::srv::Devstate;

using GripAction = griplink_interfaces::action::Grip;
using GoalHandleGripAction = rclcpp_action::ClientGoalHandle<GripAction>;
using ReleaseAction = griplink_interfaces::action::Release;
using GoalHandleReleaseAction = rclcpp_action::ClientGoalHandle<ReleaseAction>;


class DemoNode : public rclcpp::Node
{
  public:

	DemoNode();

	StatusCode home_service( uint16_t port );
	StatusCode gripcfgset_service( uint16_t port, uint16_t index, std::string tag, uint32_t no_part_limit, uint32_t release_limit, uint32_t force_factor );
	StatusCode grip_service( uint16_t port, uint16_t index );
	StatusCode release_service( uint16_t port, uint16_t index );
	StatusCode value_service( uint16_t port, uint16_t index, uint32_t &value );
	StatusCode devstate_service( uint16_t port, DeviceState &devstate );

	StatusCode grip_action( uint16_t port, uint16_t index );
	StatusCode release_action( uint16_t port, uint16_t index );

  private:

	rclcpp::Subscription<DeviceStates>::SharedPtr device_states_subscription_;
	void device_states_topic_callback( const DeviceStates &msg );
	std::array<DeviceState, 32> device_states_;

	rclcpp::Client<Home>::SharedPtr home_client_;
	rclcpp::Client<Gripcfgset>::SharedPtr gripcfgset_client_;
	rclcpp::Client<Grip>::SharedPtr grip_client_;
	rclcpp::Client<Release>::SharedPtr release_client_;
	rclcpp::Client<Value>::SharedPtr value_client_;
	rclcpp::Client<Devstate>::SharedPtr devstate_client_;

	rclcpp_action::Client<GripAction>::SharedPtr grip_action_client_;
	rclcpp_action::Client<ReleaseAction>::SharedPtr release_action_client_;
};


// Demo exception

class DemoException : public std::exception
{
  public:

	DemoException( std::string const &message );

	std::string what();

  private:

	std::string const message_;
};



#endif // DEMO_NODE_HPP