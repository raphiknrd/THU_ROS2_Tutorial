//======================================================================
/**
 *  @file griplink.cpp
 *
 *  @section griplink_node
 *
 *  @brief Implementations of the communication with the GRIPLINK.
 *  
 *
 *  @author Hannes & Jiayi
 *  @date	10.12.24
 *  
 *  
 *  @section griplink.cpp Copyright
 *  
 *  Copyright 2024 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  This file is part of GRIPLINK_ROS2 which is released under 
 *  MIT License. See file LICENSE for full license details.
 * 
 */
//======================================================================

#include "griplink/griplink_node/griplink.hpp"


namespace weiss_robotics
{


/**
 * @brief Constructs a new Griplink object.
 * 
 * @param ip The IP address of the GRIPLINK.
 * @param port The TCP/IP listening port of the GRIPLINK.
 */
Griplink::Griplink( std::string ip, uint16_t port )
{
	ip_ = ip;
	port_ = port;
	socket_fd_ = -1;

	connect();
}


/**
 * @brief Destroys the Griplink object
 */
Griplink::~Griplink()
{
	disconnect();
}


// ------------------------------------------------------------------------------------------------------------

// GRIPLINK Connection Management:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Opens a connection to the GRIPLINK.
 */
void Griplink::connect()
{
	// Create client socket
	socket_fd_ = socket( AF_INET, SOCK_STREAM, 0 );
	if ( socket_fd_ == -1 )
	{		
		std::string error_str = strerror( errno );
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "connect(): socket() failed! Errno: %s", error_str.c_str() );
		throw GriplinkException( "connect(): socket() failed! Errno: " + error_str );
	}

	// Assign server ip and port
	sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr( ip_.c_str() );
	address.sin_port = htons( port_ );

	RCLCPP_INFO( rclcpp::get_logger( "griplink" ), "Trying to connect to the GRIPLINK socket..." );

	// Connect client socket to server socket
	if ( ::connect( socket_fd_, reinterpret_cast<sockaddr *>( &address ), sizeof( address ) ) == -1 )
	{
		std::string error_str = strerror( errno );
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "connect(): connect() failed! Errno: %s", error_str.c_str() );
		close( socket_fd_ );
		socket_fd_ = -1;
		throw GriplinkException( "connect(): connect() failed! Errno: " + error_str );
	} 

	// Set socket receive timeout ( timeout for recv() )
	struct timeval timeout;
	timeout.tv_sec = 30;
	timeout.tv_usec = 0;
	if ( setsockopt( socket_fd_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<struct timeval *>( &timeout ), sizeof( struct timeval ) ) == -1 )
	{
		std::string error_str = strerror( errno );
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "connect(): setsockopt() failed! Errno: %s", error_str.c_str() );
		close( socket_fd_ );
		socket_fd_ = -1;
		throw GriplinkException( "connect(): setsockopt() failed! Errno: " + error_str );
	} 

	RCLCPP_INFO( rclcpp::get_logger( "griplink" ), "Connected to GRIPLINK." );
}


/**
 * @brief Disconnects from the GRIPLINK.
 */
void Griplink::disconnect()
{
	shutdown( socket_fd_, SHUT_RDWR );
	close( socket_fd_ );
	socket_fd_ = -1;

	RCLCPP_INFO( rclcpp::get_logger( "griplink" ), "Disconnected from GRIPLINK." );
}


// ------------------------------------------------------------------------------------------------------------

// GRIPLINK Communication and Response Interpretation:

// ------------------------------------------------------------------------------------------------------------

/**
 * @brief Sends data of the transmit buffer to the GRIPLINK.
 * 
 * @param txbuf The transmit buffer.
 */
void Griplink::send_buf( std::array<char, TXBUF_SIZE> &txbuf )
{
	// Determine the number of bytes to send
	size_t len = strlen( txbuf.begin() );

	// Send the string in txbuf and check if the correct number of bytes was sent
	if ( send( socket_fd_, txbuf.begin(), len, 0 ) != static_cast<ssize_t>( len ) )
	{
		std::string error_str = strerror( errno );
		sr_mutex_.unlock(); 
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "send_buf(): send() failed! Errno: %s", error_str.c_str() );
		throw GriplinkException( "send_buf(): send() failed! Errno: " + error_str );
	}

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Sent to GRIPLINK: '%s'", txbuf.begin() );
}


/**
 * @brief Reads out data from the socket and stores it in the receive buffer.
 * 
 * @param rxbuf The receive buffer.
 */
void Griplink::recv_buf( std::array<char, RXBUF_SIZE> &rxbuf )
{
	size_t len = 0;

	do
	{
		// Receive one byte and write into rxbuf, check that the byte was received correctly
		if ( recv( socket_fd_, rxbuf.begin() + len, 1, 0 ) != 1 )
		{
			std::string error_str = strerror( errno );
			sr_mutex_.unlock(); 
			RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "recv_buf(): recv() failed! Errno: %s", error_str.c_str() );
			throw GriplinkException( "recv_buf(): recv() failed! Errno: " + error_str );
		}
		// Prepare for receiving next byte
		len++;

		// Check whether buffer is full
		if ( len == RXBUF_SIZE )
		{
			// Buffer is full. Eat any remaining data from the socket
			while ( recv( socket_fd_, rxbuf.begin(), RXBUF_SIZE, MSG_DONTWAIT ) > 0 );
			
			sr_mutex_.unlock(); 
			RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "recv_buf(): rxbuf full" );
			throw GriplinkException( "recv_buf(): rxbuf full" );
		}
	} while ( rxbuf[len - 1] != '\n' ); // Receive until '\n' is found in buffer
	
	// Terminate received string by replacing '\n' with '\0'
	rxbuf[len - 1] = '\0';

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Received from GRIPLINK: '%s'", rxbuf.begin() );
}


/**
 * @brief Sends the command to the GRIPLINK and receives the status code from the GRIPLINK.
 * 
 * Assembles format string with arguments and sends the composed command to the GRIPLINK. 
 * Returns the received status code. 
 * 
 * @param format Printf style format string.
 * @param ... Optional arguments of the GRIPLINK command.
 * @return StatusCode returned by the GRIPLINK.
 */
StatusCode Griplink::handle_command( std::string format, ... )
{
	va_list args;
	size_t len;
	std::array<char, TXBUF_SIZE> txbuf;
	std::array<char, RXBUF_SIZE> rxbuf;

	// Assemble command with arguments
	va_start( args, format );
	len = vsnprintf( txbuf.begin(), txbuf.size(), format.c_str(), args );
	va_end( args );

	if ( len <= 0 )
	{
		// Failed to compose string
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_command(): failed to compose string" );
		throw GriplinkException( "handle_command(): failed to compose string" );
	}

	if ( len > txbuf.size() - 2 )
	{
		// Assembled command is too long
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_command(): assembled query too long" );
		throw GriplinkException( "handle_command(): assembled query too long" );
	}

	// Add '\n' and '\0'
	txbuf[len] = '\n';
	len++;
	txbuf[len] = '\0';

	// Lock the mutex for sending and receiving
	sr_mutex_.lock();

	// Remove any data from the socket, send command and receive response from GRIPLINK.
	while ( recv( socket_fd_, rxbuf.begin(), RXBUF_SIZE, MSG_DONTWAIT ) > 0 );
	send_buf( txbuf ); 	
	recv_buf( rxbuf );

	// Unlock the mutex after receiving
	sr_mutex_.unlock(); 

	// Parse command return message
	if ( strcmp( rxbuf.begin(), "ACK" ) == 0 )
	{
		return StatusCode::E_SUCCESS;
	}
	else
	{
		// Determine error status code
		return parse_error( rxbuf );
	}
}


/**
 * @brief Sends the query to the GRIPLINK, receives the status code and the query response from the GRIPLINK.
 * 
 * Assembles format string with arguments and sends the composed command to the GRIPLINK. 
 * Writes the query response into the response variable and returns the received status code.
 * 
 * @param response Variable storing the query response of the GRIPLINK.
 * @param format Printf style format string.
 * @param ... Optional Arguments of the GRIPLINK query.
 * @return StatusCode returned by the GRIPLINK.
 */
StatusCode Griplink::handle_query( std::string &response, std::string format, ... )
{
	va_list args;
	size_t len;
	std::array<char, TXBUF_SIZE> txbuf;
	std::array<char, RXBUF_SIZE> rxbuf;

	// Assemble query with arguments
	va_start( args, format );
	len = vsnprintf( txbuf.begin(), txbuf.size(), format.c_str(), args );
	va_end( args );

	if ( len <= 0 )
	{
		// Failed to compose string
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_query(): failed to compose string" );
		throw GriplinkException( "handle_query(): failed to compose string" );
	}

	if ( len > txbuf.size() - 3 )
	{
		// Assembled query is too long
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_query(): assembled query too long" );
		throw GriplinkException( "handle_query(): assembled query too long" );
	}

	// Add '?', '\n', and '\0'
	txbuf[len] = '?';
	len++;
	txbuf[len] = '\n';
	len++;
	txbuf[len] = '\0';

	// Lock the mutex for sending and receiving
	sr_mutex_.lock();

	// Remove any data from the socket, send query and receive response from GRIPLINK.
	while ( recv( socket_fd_, rxbuf.begin(), RXBUF_SIZE, MSG_DONTWAIT ) > 0 );
	send_buf( txbuf ); 	
	recv_buf( rxbuf );

	// Unlock the mutex after receiving
	sr_mutex_.unlock(); 

	// Replace '?' by '=' for comparing rxbuf and txbuf until '='
	txbuf[len - 2] = '=';

	// Check whether first part of received response matches the sent query
	if ( strncmp( rxbuf.begin(), txbuf.begin(), len - 1 ) == 0 )
	{
		// Success. Set response to the informative part of the GRIPLINK response (after '=')
		response = std::string( rxbuf.begin() + len - 1, rxbuf.begin() + strlen( rxbuf.begin() ) );
		return StatusCode::E_SUCCESS;
	}
	else
	{
		// Error. Set response to an empty string and determine error status code
		response = std::string( "" );
		return parse_error( rxbuf );
	}
}


/**
 * @brief Sends the assignment to the GRIPLINK, receives the status code and the assignment response from the GRIPLINK.
 * 
 * Assembles format string with arguments and sends the composed command to the GRIPLINK. 
 * Writes the assignment response into the response variable and returns the received status code.
 * 
 * @param response Variable storing the assignment response of the GRIPLINK.
 * @param format Printf style format string.
 * @param ... Optional Arguments of the GRIPLINK assignment.
 * @return StatusCode returned by the GRIPLINK.
 */
StatusCode Griplink::handle_assignment( std::string &response, std::string format, ... )
{
	va_list args;
	size_t len;
	std::array<char, TXBUF_SIZE> txbuf;
	std::array<char, RXBUF_SIZE> rxbuf;

	// Assemble assignment with arguments
	va_start( args, format );
	len = vsnprintf( txbuf.begin(), txbuf.size(), format.c_str(), args );
	va_end( args );

	if ( len <= 0 )
	{
		// Failed to compose string
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_assignment(): failed to compose string" );
		throw GriplinkException( "handle_assignment(): failed to compose string" );
	}

	if ( len > txbuf.size() - 2 )
	{
		// Assembled query too long
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_assignment(): assembled query too long" );
		throw GriplinkException( "handle_assignment(): assembled query too long" );
	}

	// Add '\n' and '\0'
	txbuf[len] = '\n';
	len++;
	txbuf[len] = '\0';

	// Lock the mutex for sending and receiving
	sr_mutex_.lock();

	// Remove any data from the socket, send assignment and receive response from GRIPLINK.
	while ( recv( socket_fd_, rxbuf.begin(), RXBUF_SIZE, MSG_DONTWAIT ) > 0 );
	send_buf( txbuf ); 	
	recv_buf( rxbuf );

	// Unlock the mutex after receiving
	sr_mutex_.unlock(); 

	// Check whether the full received response matches the sent assignment
	if ( strncmp( rxbuf.begin(), txbuf.begin(), len - 1 ) == 0 )
	{
		// Find '=' in rxbuf
		char const *equal = strchr( rxbuf.begin(), '=' );
		if ( equal == nullptr )
		{
			// Error. No '=' is found in rxbuf
			RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "handle_assignment(): no '=' is found in rxbuf string" );
			throw GriplinkException( "handle_assignment(): no '=' is found in rxbuf string" );
		}

		// Success. Set response to the informative part of the GRIPLINK response (after '=')
		char const *start = equal + 1;
		response = std::string( start, strlen( start ) );
		return StatusCode::E_SUCCESS;
	}
	else
	{
		// Error. Set response to an empty string and determine error status code
		response = std::string( "" );
		return parse_error( rxbuf );
	}
}


/**
 * @brief Parse the error response of the GRIPLINK.
 * 
 * @param rxbuf The receive buffer with the GRIPLINK response.
 * @return StatusCode: The error code returned by the GRIPLINK.
 */
StatusCode Griplink::parse_error( std::array<char, RXBUF_SIZE> &rxbuf ) 
{
	char *endp;
	uint32_t status_code;

	// Check whether GRIPLINK response starts with "ERR "
	if ( strncmp( rxbuf.begin(), "ERR ", 4 ) == 0 )
	{
		// Parse for status code in string and convert to uint32_t
		status_code = strtoul( &( rxbuf[4] ), &endp, 10 );

		// Status code must be exactly two digits long
		if ( endp != &( rxbuf[6] ) )
		{
			// Failed to convert status code
			RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "parse_error(): failed to convert status code: %s", rxbuf.begin() );
			throw GriplinkException( "parse_error(): failed to convert status code" );
		}

		// Success
		return static_cast<StatusCode>( status_code );
	} 
	else
	{
		// Failed. Response did not start with "ERR "
		RCLCPP_ERROR( rclcpp::get_logger( "griplink" ), "parse_error(): cannot parse error message: %s", rxbuf.begin() );
		throw GriplinkException( "parse_error(): cannot parse error message" );
	}
}


// ------------------------------------------------------------------------------------------------------------

// GRIPLINK UNIFIED COMMAND SET - GRIPLINK Layer:

// ------------------------------------------------------------------------------------------------------------

StatusCode Griplink::id( std::string &response )
{
	StatusCode status;
	status = handle_query( response, "ID" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: id. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::protocol( std::string &response )
{
	StatusCode status;
	status = handle_query( response, "PROTOCOL" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: protocol. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::protassert( std::string protocol, uint16_t version )
{
	StatusCode status;
	status = handle_command( "PROTASSERT(\"%s\",%u)", protocol.c_str(), version );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: protassert. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::sn( std::string &response )
{
	StatusCode status;
	status = handle_query( response, "SN" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: sn. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::labelget( std::string &response )
{
	StatusCode status;
	status = handle_query( response, "LABEL" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: labelget. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::labelset( std::string &response, std::string label )
{
	StatusCode status;
	status = handle_assignment( response, "LABEL=\"%s\"", label.c_str() );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: labelset. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::ver( std::string &response )
{
	StatusCode status;
	status = handle_query( response, "VER" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: ver. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::verbose( std::string &response, bool enable )
{
	StatusCode status;
	status = handle_assignment( response, "VERBOSE=%u", enable );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: verbose. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devvid( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVVID[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devvid. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devpid( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVPID[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devpid. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devassert( uint16_t port, uint32_t vid, uint32_t pid )
{
	StatusCode status;
	status = handle_command( "DEVASSERT(%u,%u,%u)", port, vid, pid );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devassert. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::devname( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVNAME[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devname. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devvendor( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVVENDOR[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devvendor. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devsn( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVSN[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devsn. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );


	return status;
}

StatusCode Griplink::devtagget( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVTAG[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devtagget. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devtagset( std::string &response, uint16_t port, std::string tag )
{
	StatusCode status;
	status = handle_assignment( response, "DEVTAG[%u]=\"%s\"", port, tag.c_str() );
	
	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devtagset. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::devver( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVVER[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devver. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::bye()
{
	StatusCode status;
	status = handle_command( "BYE()" );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: bye. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::enable( uint16_t port )
{
	StatusCode status;
	status = handle_command( "ENABLE(%u)", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: enable. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::disable( uint16_t port )
{
	StatusCode status;
	status = handle_command( "DISABLE(%u)", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: disable. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::home( uint16_t port )
{
	StatusCode status;
	status = handle_command( "HOME(%u)", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: home. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::grip( uint16_t port, uint16_t index )
{
	StatusCode status;
	status = handle_command( "GRIP(%u,%u)", port, index );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: grip. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::release( uint16_t port, uint16_t index )
{
	StatusCode status;
	status = handle_command( "RELEASE(%u,%u)", port, index );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: release. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::flexgrip( uint16_t port, uint32_t position, uint32_t force, uint32_t speed, uint32_t acceleration )
{
	StatusCode status;
	status = handle_command( "FLEXGRIP(%u,%u,%u,%u,%u)", port, position, force, speed, acceleration );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: flexgrip. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::flexrelease( uint16_t port, uint32_t position, uint32_t speed, uint32_t acceleration )
{
	StatusCode status;
	status = handle_command( "FLEXRELEASE(%u,%u,%u,%u)", port, position, speed, acceleration );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: flexrelease. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::led( uint16_t port, uint16_t index )
{
	StatusCode status;
	status = handle_command( "LED(%u,%u)", port, index );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: led. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::clamp( uint16_t port, bool enable )
{
	StatusCode status;
	status = handle_command( "CLAMP(%u,%u)", port, enable );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: clamp. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::wstr( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "WSTR[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: wstr. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::setval( uint16_t port, uint16_t index, uint32_t value )
{
	StatusCode status;
	status = handle_command( "SETVAL(%u,%u,%u)", port, index, value );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: setval. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::waitval( uint16_t port, uint16_t index, uint32_t threshold, uint32_t window, uint32_t timeout )
{
	StatusCode status;
	status = handle_command( "WAITVAL(%u,%u,%u,%u,%u)", port, index, threshold, window, timeout );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: waitval. Status: %u", static_cast<uint16_t>( status ) );

	return status;
}

StatusCode Griplink::devstate( std::string &response, uint16_t port )
{
	StatusCode status;
	status = handle_query( response, "DEVSTATE[%u]", port );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: devstate. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::value( std::string &response, uint16_t port, uint16_t index )
{
	StatusCode status;
	status = handle_query( response, "VALUE[%u][%u]", port, index );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: value. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::gripcfgget( std::string &response, uint16_t port, uint16_t index )
{
	StatusCode status;
	status = handle_query( response, "GRIPCFG[%u][%u]", port, index );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: gripcfgget. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

StatusCode Griplink::gripcfgset( std::string &response, uint16_t port, uint16_t index, std::string tag, std::array<uint32_t, 8> parameters )
{
	StatusCode status;
	status = handle_assignment( response, "GRIPCFG[%u][%u]=[\"%s\",%u,%u,%u,%u,%u,%u,%u,%u]", port, index, tag.c_str(), parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], parameters[6], parameters[7] );

	RCLCPP_DEBUG( rclcpp::get_logger( "griplink" ), "Griplink command: gripcfgset. Status: %u, Response: %s", static_cast<uint16_t>( status ), response.c_str() );

	return status;
}

} // namespace weiss_robotics
