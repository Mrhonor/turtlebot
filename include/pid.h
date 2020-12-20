#ifndef __PID__
#define __PID__


template<typename T, typename LimitFunc>
class pid
{
public:
    typedef T ValueType;
    typedef LimitFunc AbsLimit;

    double kp;
    double ki;
    double kd;

    ValueType Ilimit;
    ValueType Outlimit;

private:
    ValueType Pout;
    ValueType Iout;
    ValueType Dout;

    ValueType lastErr;

public:
    pid(){}
    pid(double kp_, double ki_, double kd_, ValueType Ilimit_, ValueType Outlimit_);


    ValueType pid_calc(ValueType set, ValueType get);

    ~pid(){};
};

template<typename ValueType, typename LimitFunc>
ValueType pid<ValueType, LimitFunc>::pid_calc(ValueType set, ValueType get){

    ValueType err = set - get;
    // ROS_INFO("err: %f, %f, %f", err(0,0), err(1, 0), err(2,0));
    // ROS_INFO("set: %f, %f, %f", set(0,0), set(1, 0), set(2,0));
    // ROS_INFO("get: %f, %f, %f", get(0,0), get(1, 0), get(2,0));


    Pout = kp * err;

    Iout = AbsLimit()(Iout + ki* err, Ilimit);
    Dout = kd * (err - lastErr);

    lastErr = err;

    return AbsLimit()(Pout + Iout + Dout, Outlimit);
}

template<typename ValueType, typename LimitFunc>
pid<ValueType, LimitFunc>::pid(double kp_, double ki_, double kd_, ValueType Ilimit_, ValueType Outlimit_):kp(kp_), ki(ki_), kd(kd_), Ilimit(Ilimit_), Outlimit(Outlimit_){
    Pout = 0;
    Iout = 0;
    Dout = 0;
    lastErr = 0;
}


#endif

