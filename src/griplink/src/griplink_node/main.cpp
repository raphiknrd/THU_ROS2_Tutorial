//======================================================================
/**
 *  @file main.cpp
 *
 *  @section griplink_node
 *
 *  @brief Main file for the griplink ROS2 node. 
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
//======================================================================

#include <griplink/griplink_node/griplink_node.hpp>


using namespace weiss_robotics;


int main( int argc, char const *argv[] )
{
	try
	{
		rclcpp::init( argc, argv );
		auto node = std::make_shared<GriplinkNode>();
		rclcpp::executors::MultiThreadedExecutor executor;
		executor.add_node(node);
		executor.spin();
		rclcpp::shutdown();
		return 0;
	}
	catch ( GriplinkException &e )
	{
		RCLCPP_ERROR_STREAM( rclcpp::get_logger( "GriplinkNode" ), "Caught griplink exception: " << e.what() );
      	rclcpp::shutdown();
		return 1;
	}
}