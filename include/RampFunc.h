#ifndef __RAMPFUNC__
#define __RAMPFUNC__

#include "ros/ros.h"

template<typename T>
class RampFunc
{
public:
    T operator()(const T& Cur, const T& Tar, const T& step){
        ROS_WARN("Cur : %f, Tar: %f, Step : %f", Cur, Tar, step);
        if(Cur < Tar){
            return (Cur + step > Tar) ? Tar : (Cur + step);
        }
        else
        {
            return (Cur - step < Tar) ? Tar : (Cur - step);
        }
    }
};

#endif
