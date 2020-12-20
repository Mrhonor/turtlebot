#ifndef __LIMIT__
#define __LIMIT__
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

template<typename T>
class Limit
{
public:
    T operator()(const T& A, const T& B){
        if(A > B){
            return B;
        }
        else if(A < -B)
        {
            return -B;
        }
        else
        {
            return A;
        }
    }
};

template<>
class Limit<Eigen::Vector3d>
{
public:
    Eigen::Vector3d operator()(const Eigen::Vector3d& A, const Eigen::Vector3d& B){
        Eigen::Vector3d out;
        for(int i = 0; i < 3; i++){
            out(i, 0) = Limit<double>()(A(i, 0), B(i, 0));
        }
        return out;
    }
};



#endif
