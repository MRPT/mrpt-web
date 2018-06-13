#pragma once
#include <iostream>
#include <thread>
#include <atomic>
#include <memory>

namespace mrpt::web
{
    //class  derived from pure abstract class CPubSubManagerBase
    class CPubSubManager: public CPubSubManagerBase
    {
    public:
        CPubSubManager()
        {
        }
        virtual int add(int a,int b) override
        {
            return a + b;
        }
    };
}
 