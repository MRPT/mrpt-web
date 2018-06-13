#pragma once
#include <iostream>
#include <thread>
#include <atomic>
#include <memory>

namespace mrpt::web
{   
    //pure abstract class 
    class CPubSubManagerBase
    {
        virtual int add(int a,int b) = 0;
        // Publisher side work:
        // Contains methods for adding client_id to set published_topics["top"]
        // and for removing client_id from set published_topics["top"]

        // Subscription side work:
        // Contains methods for adding client_id to set subscribed_topics["top"]
        // and for removing client_id from set subscribed_topics["top"]

    };
}