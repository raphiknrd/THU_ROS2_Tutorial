//======================================================================
/**
 *  @file griplink.hpp
 *
 *  @section griplink_node
 *
 *  @brief The griplink class.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section griplink.hpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 */
//======================================================================

#ifndef GRIPLINK_HPP
#define GRIPLINK_HPP


extern "C"
{
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
}

#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "griplink/griplink_node/common.hpp"


#define TXBUF_SIZE 64
#define RXBUF_SIZE 64


namespace weiss_robotics
{


class Griplink
{
  private:

	std::string ip_;
	uint16_t port_;
	int socket_fd_;
	std::mutex sr_mutex_; // Send-Receive mutex 

  public:

	// Constructor and destructor:

	Griplink(std::string ip, uint16_t port);
	~Griplink();

	// Connection management:

	void connect();
	void disconnect();

	// Send and receive messages:

	void send_buf( std::array<char, TXBUF_SIZE> &txbuf );
	void recv_buf( std::array<char, RXBUF_SIZE> &rxbuf );

	// Handling of diffrent types of griplink commands:

	StatusCode handle_command( std::string format, ... );
	StatusCode handle_query( std::string &response, std::string format, ... );
	StatusCode handle_assignment( std::string &response, std::string format, ... );
	StatusCode parse_error( std::array<char, RXBUF_SIZE> &rxbuf );

	// Griplink commands:

	StatusCode id( std::string &response );
	StatusCode protocol( std::string &response );
	StatusCode protassert( std::string protocol, uint16_t version );
	StatusCode sn( std::string &response );
	StatusCode labelget( std::string &response );
	StatusCode labelset( std::string &response, std::string label );
	StatusCode ver( std::string &response );
	StatusCode verbose( std::string &response, bool enable );
	StatusCode devvid( std::string &response, uint16_t port );
	StatusCode devpid( std::string &response, uint16_t port );
	StatusCode devassert( uint16_t port, uint32_t vid, uint32_t pid );
	StatusCode devname( std::string &response, uint16_t port );
	StatusCode devvendor( std::string &response, uint16_t port );
	StatusCode devsn( std::string &response, uint16_t port );
	StatusCode devtagget( std::string &response, uint16_t port );
	StatusCode devtagset( std::string &response, uint16_t port, std::string tag );
	StatusCode devver( std::string &response, uint16_t port );
	StatusCode bye();
	StatusCode enable( uint16_t port );
	StatusCode disable( uint16_t port );
	StatusCode home( uint16_t port );
	StatusCode grip( uint16_t port, uint16_t index );
	StatusCode release( uint16_t port, uint16_t index );
	StatusCode flexgrip( uint16_t port, uint32_t position, uint32_t force, uint32_t speed, uint32_t acceleration );
	StatusCode flexrelease( uint16_t port, uint32_t position, uint32_t speed, uint32_t acceleration );
	StatusCode led( uint16_t port, uint16_t index );
	StatusCode clamp( uint16_t port, bool enable );
	StatusCode wstr( std::string &response, uint16_t port );
	StatusCode setval( uint16_t port, uint16_t index, uint32_t value );
	StatusCode waitval( uint16_t port, uint16_t index, uint32_t threshold, uint32_t window, uint32_t timeout );
	StatusCode devstate( std::string &response, uint16_t port );
	StatusCode value( std::string &response, uint16_t port, uint16_t index );
	StatusCode gripcfgget( std::string &response, uint16_t port, uint16_t index );
	StatusCode gripcfgset( std::string &response, uint16_t port, uint16_t index, std::string tag, std::array<uint32_t, 8> parameters );

	// Misc:

	using UniquePtr = std::unique_ptr<Griplink>;
	using SharedPtr = std::shared_ptr<Griplink>;
};


} // namespace weiss_robotics


#endif // GRIPLINK_HPP
