#pragma once
#include <string>
#include <cstdlib>
#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/CRequestHandler.h>
namespace mrpt::web {

class CAbstractServerConnector {
public:
  CAbstractServerConnector() {
      this->handler = NULL;
  }
  virtual ~CAbstractServerConnector() {}

  /**
   * This method should signal the Connector to start waiting for requests, in
   * any way that is appropriate for the derived connector class.
   * If something went wrong, this method should return false, otherwise true.
   */
  virtual bool StartListening() = 0;
  /**
   * This method should signal the Connector to stop waiting for requests, in
   * any way that is appropriate for the derived connector class.
   * If something went wrong, this method should return false, otherwise true.
   */
  virtual bool StopListening() = 0;

  inline void ProcessRequest(const std::string &request, std::string &response, ConnectionPointer _conn) {

      if(this->handler != NULL) {
          this->handler->handleRequest(request, response, _conn);
      }
  }

  inline void SetHandler(CRequestHandler *handler)
  {
      this->handler = handler;
  }

  CRequestHandler *GetHandler(){
      return this->handler;
  }

private:
  CRequestHandler *handler;
};

} /* namespace jsonrpc */
