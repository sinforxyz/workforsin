#include<iostream>
#include<array>
#include<cmath>

class quaternion{
private:
    double a,b,c,d;
public:
    quaternion(double a,double b,double c,double d):a(a),b(b),c(c),d(d){}
    quaternion():a(0),b(0),c(0),d(0){}
    quaternion operator+(const quaternion&other)const{
        return quaternion(a+other.a,b+other.b,c+other.c,d+other.d);
    }
    quaternion operator*(const quaternion&other)const{
        return quaternion(a*other.a-b*other.b-c*other.c-d*other.d,
                          a*other.b+b*other.a+c*other.d-d*other.c,
                          a*other.c-b*other.d+c*other.a+d*other.b,
                          a*other.d+b*other.c-c*other.a+d*other.a);
    }
    double norm(const quaternion&q){
        return pow(q.a*q.a+q.b*q.b+q.c*q.c+q.d*q.d,0.5);
    }
    quaternion conjugate(const quaternion&q){
        return quaternion(q.a,-q.b,-q.c,-q.d);
    }
    quaternion inverse(const quaternion&q){
        quaternion q_=conjugate(q);
        double qn=norm(q);
        return quaternion(q_.a/qn,q_.b/qn,q_.c/qn,q_.d/qn);
    }
    quaternion rotation(const quaternion v,const quaternion q){
        quaternion q_=inverse(q);
        return q*v*q_;
    }
};

class pose{
    private:
    std::array<double,3>position;
    std::array<double,3>euler;
    public:
    pose():position{0,0,0},euler{0,0,0}{}
    pose(double x,double y,double z,double yaw,double pitch,double row):
    position{x,y,z},euler{yaw,pitch,row}{}    
};
