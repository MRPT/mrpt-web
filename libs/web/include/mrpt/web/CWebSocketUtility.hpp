#pragma once
#include <cstdlib>
#include <boost/beast/websocket.hpp>
// Report a failure
void
fail(boost::system::error_code ec, char const* what)
{
    std::cerr << (std::string(what) + ": " + ec.message() + "\n");
}
