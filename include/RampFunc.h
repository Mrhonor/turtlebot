#ifndef __RAMPFUNC__
#define __RAMPFUNC__


template<typename T>
class RampFunc
{
public:
    T operator()(const T& Cur, const T& Tar, const T& step){
        if(Cur < Tar){
            return (Cur + step > Tar) ? Tar : (Cur + step);
        }
        else
        {
            return (Cur - step < Tar) ? Tar : (Cur - step);
        }
    }
};

template<>
class RampFunc<Eigen::Vector3d>
{
public:
    Eigen::Vector3d operator()(const Eigen::Vector3d& Cur, const Eigen::Vector3d& Tar, const Eigen::Vector3d& step){
        Eigen::Vector3d out;
        for(int i = 0; i < 3; i++){
            out(i, 0) = RampFunc<double>()(Cur(i, 0), Tar(i, 0), step(i, 0));
        }
        return out;
    }
};

#endif
