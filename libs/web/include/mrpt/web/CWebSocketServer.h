#pragma once

#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/CAbstractServerConnector.h>
#include <mrpt/web/CPubSubManagerBase.h>

namespace mrpt::web
{
  class CWebSocketServer : public  mrpt::web::CAbstractServerConnector {
  private:
	WsServer m_server;
	WsServer::Endpoint& m_endpoint;
	std::string m_str;
	std::thread m_thread;
  	std::shared_ptr<CPubSubManagerBase> m_manager;
  public:
	CWebSocketServer(unsigned short port, std::shared_ptr<CPubSubManagerBase> manager):m_endpoint(m_server.endpoint["^/?$"]), m_manager(manager) {
		m_server.config.port = port;
	}
	/** This method launched the listening loop that will handle client connections.
	 * @return true for success, false otherwise.
	 */
	bool StartListening() override
	{
		// Define the actions for the base endpoint("^/?$")
		m_endpoint.on_message = [&](std::shared_ptr<WsServer::Connection> connection,
		 						std::shared_ptr<WsServer::Message> message) {
									std::string message_str = message->string();
									std::cout << "Message : " << message_str << std::endl; 
									//Process the output
									std::string output_str;
									this->ProcessRequest(message_str, output_str, connection);
									//Create the stream to send output			
									auto send_stream = std::make_shared<WsServer::SendStream>();
									*send_stream << output_str;
									//
									// connection->send is an asynchronous function
									connection->send(send_stream, [](const mrpt::web::error_code &ec){
										if(ec) {
											std::cout << "Server: Error sending message. " <<
											"Error" << ec << ", error_message: " << ec.message() << std::endl;
										}
									});
								};
		m_endpoint.on_open = [](std::shared_ptr<WsServer::Connection> connection) {
			std::cout << "Server: Opened connection " << connection.get()  << std::endl;
		};
		m_endpoint.on_close = [](std::shared_ptr<WsServer::Connection> connection, int status, const std::string& reason) {
			std::cout << "Server: closed connection " << connection.get() << " with status code" << status << std::endl;
			//Remove the dead Connection from the PubSubManager	
		};
		// See RFC 6455 7.4.1 for status codes
		m_endpoint.on_error = [](std::shared_ptr<WsServer::Connection> connection, const mrpt::web::error_code &ec) {
			std::cout << "Server: Error in connection " << connection.get() << ". "
				<<"Error: " << ec << ", error message: " << ec.message() << std::endl;
		};

		std::thread server_thread([this]() {
			// Start WS-server
			this->m_server.start();
		});
		// Move the thread to class member m_thread
		server_thread.swap(m_thread);
	}
	/** 
	 * This method stops the listening loop that will handle client connections
	 * @return True if successful, false otherwise or if not listening.
	 */
	bool StopListening() override
	{
		//Code here
		//Capture SIGINT and SIGTERM to perform a clean shutdown
		// Check for this code how to do this
		
		//--------------------------------
		// Clean Exit Extra code

		//--------------------------------
		m_thread.join();
	}
  };
}