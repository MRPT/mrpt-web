#include <CRPCRawLogAbstract.h>
#include <mrpt/web/CWebSocketJsonRpcServer.h>
#include <mrpt/web/CWebSocket.h>

class CRPCRawLog : public CRPCRawLogAbstract
{
public:
  Json::Value Playlist_GetPlaylists() override
  {

  }
  Json::Value Playlist_GetItems() override
  {

  }
};

int main(int argc, char *argv[])
{

  return 0;
}
