//======================================================================
/**
 *  @file griplink_node.hpp
 *
 *  @section griplink_node
 *
 *  @brief The griplink node class.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section griplink_node.hpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//======================================================================

#ifndef GRIPLINK_NODE_HPP
#define GRIPLINK_NODE_HPP

#include <string>
#include <vector>
#include <sstream>
#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "griplink/griplink_node/griplink.hpp"
#include "griplink/griplink_node/common.hpp"

#include "griplink_interfaces/msg/device_states.hpp"

#include "griplink_interfaces/srv/id.hpp"
#include "griplink_interfaces/srv/protocol.hpp"
#include "griplink_interfaces/srv/protassert.hpp"
#include "griplink_interfaces/srv/sn.hpp"
#include "griplink_interfaces/srv/labelget.hpp"
#include "griplink_interfaces/srv/labelset.hpp"
#include "griplink_interfaces/srv/ver.hpp"
#include "griplink_interfaces/srv/verbose.hpp"
#include "griplink_interfaces/srv/devvid.hpp"
#include "griplink_interfaces/srv/devpid.hpp"
#include "griplink_interfaces/srv/devassert.hpp"
#include "griplink_interfaces/srv/devname.hpp"
#include "griplink_interfaces/srv/devvendor.hpp"
#include "griplink_interfaces/srv/devsn.hpp"
#include "griplink_interfaces/srv/devtagget.hpp"
#include "griplink_interfaces/srv/devtagset.hpp"
#include "griplink_interfaces/srv/devver.hpp"
#include "griplink_interfaces/srv/bye.hpp"
#include "griplink_interfaces/srv/enable.hpp"
#include "griplink_interfaces/srv/disable.hpp"
#include "griplink_interfaces/srv/home.hpp"
#include "griplink_interfaces/srv/grip.hpp"
#include "griplink_interfaces/srv/release.hpp"
#include "griplink_interfaces/srv/flexgrip.hpp"
#include "griplink_interfaces/srv/flexrelease.hpp"
#include "griplink_interfaces/srv/led.hpp"
#include "griplink_interfaces/srv/clamp.hpp"
#include "griplink_interfaces/srv/wstr.hpp"
#include "griplink_interfaces/srv/setval.hpp"
#include "griplink_interfaces/srv/waitval.hpp"
#include "griplink_interfaces/srv/devstate.hpp"
#include "griplink_interfaces/srv/value.hpp"
#include "griplink_interfaces/srv/gripcfgget.hpp"
#include "griplink_interfaces/srv/gripcfgset.hpp"

#include "griplink_interfaces/action/grip.hpp"
#include "griplink_interfaces/action/release.hpp"
#include "griplink_interfaces/action/flexgrip.hpp"
#include "griplink_interfaces/action/flexrelease.hpp"


namespace weiss_robotics
{


class GriplinkNode : public rclcpp::Node
{
  public:

	GriplinkNode();

  protected:

	using DeviceStates = griplink_interfaces::msg::DeviceStates;

	using Id = griplink_interfaces::srv::Id;
	using Protocol = griplink_interfaces::srv::Protocol;
	using Protassert = griplink_interfaces::srv::Protassert;
	using Sn = griplink_interfaces::srv::Sn;
	using Labelget = griplink_interfaces::srv::Labelget;
	using Labelset = griplink_interfaces::srv::Labelset;
	using Ver = griplink_interfaces::srv::Ver;
	using Verbose = griplink_interfaces::srv::Verbose;
	using Devvid = griplink_interfaces::srv::Devvid;
	using Devpid = griplink_interfaces::srv::Devpid;
	using Devassert = griplink_interfaces::srv::Devassert;
	using Devname = griplink_interfaces::srv::Devname;
	using Devvendor = griplink_interfaces::srv::Devvendor;
	using Devsn = griplink_interfaces::srv::Devsn;
	using Devtagget = griplink_interfaces::srv::Devtagget;
	using Devtagset = griplink_interfaces::srv::Devtagset;
	using Devver = griplink_interfaces::srv::Devver;
	using Bye = griplink_interfaces::srv::Bye;
	using Enable = griplink_interfaces::srv::Enable;
	using Disable = griplink_interfaces::srv::Disable;
	using Home = griplink_interfaces::srv::Home;
	using Grip = griplink_interfaces::srv::Grip;
	using Release = griplink_interfaces::srv::Release;
	using Flexgrip = griplink_interfaces::srv::Flexgrip;
	using Flexrelease = griplink_interfaces::srv::Flexrelease;
	using Led = griplink_interfaces::srv::Led;
	using Clamp = griplink_interfaces::srv::Clamp;
	using Wstr = griplink_interfaces::srv::Wstr;
	using Setval = griplink_interfaces::srv::Setval;
	using Waitval = griplink_interfaces::srv::Waitval;
	using Devstate = griplink_interfaces::srv::Devstate;
	using Value = griplink_interfaces::srv::Value;
	using Gripcfgget = griplink_interfaces::srv::Gripcfgget;
	using Gripcfgset = griplink_interfaces::srv::Gripcfgset;

	using GripAction = griplink_interfaces::action::Grip;
	using GoalHandleGripAction = rclcpp_action::ServerGoalHandle<GripAction>;
	using ReleaseAction = griplink_interfaces::action::Release;
	using GoalHandleReleaseAction = rclcpp_action::ServerGoalHandle<ReleaseAction>;
	using FlexgripAction = griplink_interfaces::action::Flexgrip;
	using GoalHandleFlexgripAction = rclcpp_action::ServerGoalHandle<FlexgripAction>;
	using FlexreleaseAction = griplink_interfaces::action::Flexrelease;
	using GoalHandleFlexreleaseAction = rclcpp_action::ServerGoalHandle<FlexreleaseAction>;

	Griplink::UniquePtr griplink_;

	std::array<DeviceState, 32> device_states_;
	uint16_t max_number_of_devices_;
	rclcpp::TimerBase::SharedPtr device_states_timer_;
	
	rclcpp::CallbackGroup::SharedPtr callback_group_;

	rclcpp::Publisher<DeviceStates>::SharedPtr device_states_publisher_;

	rclcpp::Service<Id>::SharedPtr id_srv_;
	rclcpp::Service<Protocol>::SharedPtr protocol_srv_;
	rclcpp::Service<Protassert>::SharedPtr protassert_srv_;
	rclcpp::Service<Sn>::SharedPtr sn_srv_;
	rclcpp::Service<Labelget>::SharedPtr labelget_srv_;
	rclcpp::Service<Labelset>::SharedPtr labelset_srv_;
	rclcpp::Service<Ver>::SharedPtr ver_srv_;
	rclcpp::Service<Verbose>::SharedPtr verbose_srv_;
	rclcpp::Service<Devvid>::SharedPtr devvid_srv_;
	rclcpp::Service<Devpid>::SharedPtr devpid_srv_;
	rclcpp::Service<Devassert>::SharedPtr devassert_srv_;
	rclcpp::Service<Devname>::SharedPtr devname_srv_;
	rclcpp::Service<Devvendor>::SharedPtr devvendor_srv_;
	rclcpp::Service<Devsn>::SharedPtr devsn_srv_;
	rclcpp::Service<Devtagget>::SharedPtr devtagget_srv_;
	rclcpp::Service<Devtagset>::SharedPtr devtagset_srv_;
	rclcpp::Service<Devver>::SharedPtr devver_srv_;
	rclcpp::Service<Bye>::SharedPtr bye_srv_;
	rclcpp::Service<Enable>::SharedPtr enable_srv_;
	rclcpp::Service<Disable>::SharedPtr disable_srv_;
	rclcpp::Service<Home>::SharedPtr home_srv_;
	rclcpp::Service<Grip>::SharedPtr grip_srv_;
	rclcpp::Service<Release>::SharedPtr release_srv_;
	rclcpp::Service<Flexgrip>::SharedPtr flexgrip_srv_;
	rclcpp::Service<Flexrelease>::SharedPtr flexrelease_srv_;
	rclcpp::Service<Led>::SharedPtr led_srv_;
	rclcpp::Service<Clamp>::SharedPtr clamp_srv_;
	rclcpp::Service<Wstr>::SharedPtr wstr_srv_;
	rclcpp::Service<Setval>::SharedPtr setval_srv_;
	rclcpp::Service<Waitval>::SharedPtr waitval_srv_;
	rclcpp::Service<Devstate>::SharedPtr devstate_srv_;
	rclcpp::Service<Value>::SharedPtr value_srv_;
	rclcpp::Service<Gripcfgget>::SharedPtr gripcfgget_srv_;
	rclcpp::Service<Gripcfgset>::SharedPtr gripcfgset_srv_;

	rclcpp_action::Server<GripAction>::SharedPtr grip_action_server_;
	rclcpp_action::Server<ReleaseAction>::SharedPtr release_action_server_;
	rclcpp_action::Server<FlexgripAction>::SharedPtr flexgrip_action_server_;
	rclcpp_action::Server<FlexreleaseAction>::SharedPtr flexrelease_action_server_;

	void initialize_device_states();
	void update_device_states();

	void id( Id::Request::SharedPtr req, Id::Response::SharedPtr res );
	void protocol( Protocol::Request::SharedPtr req, Protocol::Response::SharedPtr res );
	void protassert( Protassert::Request::SharedPtr req, Protassert::Response::SharedPtr res );
	void sn( Sn::Request::SharedPtr req, Sn::Response::SharedPtr res );
	void labelget( Labelget::Request::SharedPtr req, Labelget::Response::SharedPtr res );
	void labelset( Labelset::Request::SharedPtr req, Labelset::Response::SharedPtr res );
	void ver( Ver::Request::SharedPtr req, Ver::Response::SharedPtr res );
	void verbose( Verbose::Request::SharedPtr req, Verbose::Response::SharedPtr res );
	void devvid( Devvid::Request::SharedPtr req, Devvid::Response::SharedPtr res );
	void devpid( Devpid::Request::SharedPtr req, Devpid::Response::SharedPtr res );
	void devassert( Devassert::Request::SharedPtr req, Devassert::Response::SharedPtr res );
	void devname( Devname::Request::SharedPtr req, Devname::Response::SharedPtr res );
	void devvendor( Devvendor::Request::SharedPtr req, Devvendor::Response::SharedPtr res );
	void devsn( Devsn::Request::SharedPtr req, Devsn::Response::SharedPtr res );
	void devtagget( Devtagget::Request::SharedPtr req, Devtagget::Response::SharedPtr res );
	void devtagset( Devtagset::Request::SharedPtr req, Devtagset::Response::SharedPtr res );
	void devver( Devver::Request::SharedPtr req, Devver::Response::SharedPtr res );
	void bye( Bye::Request::SharedPtr req, Bye::Response::SharedPtr res );
	void enable( Enable::Request::SharedPtr req, Enable::Response::SharedPtr res );
	void disable( Disable::Request::SharedPtr req, Disable::Response::SharedPtr res );
	void home( Home::Request::SharedPtr req, Home::Response::SharedPtr res );
	void grip( Grip::Request::SharedPtr req, Grip::Response::SharedPtr res );
	void release( Release::Request::SharedPtr req, Release::Response::SharedPtr res );
	void flexgrip( Flexgrip::Request::SharedPtr req, Flexgrip::Response::SharedPtr res );
	void flexrelease( Flexrelease::Request::SharedPtr req, Flexrelease::Response::SharedPtr res );
	void led( Led::Request::SharedPtr req, Led::Response::SharedPtr res );
	void clamp( Clamp::Request::SharedPtr req, Clamp::Response::SharedPtr res );
	void wstr( Wstr::Request::SharedPtr req, Wstr::Response::SharedPtr res );
	void setval( Setval::Request::SharedPtr req, Setval::Response::SharedPtr res );
	void waitval( Waitval::Request::SharedPtr req, Waitval::Response::SharedPtr res );
	void devstate( Devstate::Request::SharedPtr req, Devstate::Response::SharedPtr res );
	void value( Value::Request::SharedPtr req, Value::Response::SharedPtr res );
	void gripcfgget( Gripcfgget::Request::SharedPtr req, Gripcfgget::Response::SharedPtr res );
	void gripcfgset( Gripcfgset::Request::SharedPtr req, Gripcfgset::Response::SharedPtr res );


	rclcpp_action::GoalResponse grip_handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripAction::Goal> goal );
	rclcpp_action::CancelResponse grip_handle_cancel( const std::shared_ptr<GoalHandleGripAction> goal_handle );
	void grip_handle_accepted( const std::shared_ptr<GoalHandleGripAction> goal_handle );

	rclcpp_action::GoalResponse release_handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ReleaseAction::Goal> goal );
	rclcpp_action::CancelResponse release_handle_cancel( const std::shared_ptr<GoalHandleReleaseAction> goal_handle );
	void release_handle_accepted( const std::shared_ptr<GoalHandleReleaseAction> goal_handle );

	rclcpp_action::GoalResponse flexgrip_handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FlexgripAction::Goal> goal );
	rclcpp_action::CancelResponse flexgrip_handle_cancel( const std::shared_ptr<GoalHandleFlexgripAction> goal_handle );
	void flexgrip_handle_accepted( const std::shared_ptr<GoalHandleFlexgripAction> goal_handle );

	rclcpp_action::GoalResponse flexrelease_handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FlexreleaseAction::Goal> goal );
	rclcpp_action::CancelResponse flexrelease_handle_cancel( const std::shared_ptr<GoalHandleFlexreleaseAction> goal_handle );
	void flexrelease_handle_accepted( const std::shared_ptr<GoalHandleFlexreleaseAction> goal_handle );
};



} // namespace weiss_robotics


#endif // GRIPLINK_NODE_HPP


