#include<iostream>
#include<array>
#include<cmath>

class quaternion{
private:
    double a,b,c,d;
public:
    quaternion(double a,double b,double c,double d):a(a),b(b),c(c),d(d){}
    quaternion():a(0),b(0),c(0),d(0){}
    quaternion(double yaw,double pitch,double roll){
        double cy=std::cos(yaw*0.5);
        double sy=std::sin(yaw*0.5);
        double cp=std::cos(pitch*0.5);
        double sp=std::sin(pitch*0.5);
        double cr=std::cos(roll*0.5);
        double sr=std::sin(roll*0.5);

        a=cr*cp*cy+sr*sp*sy;
        b=sr*cp*cy-cr*sp*sy;
        c=cr*sp*cy+sr*cp*sy;
        d=cr*sy*cp-sr*sp*cy;
    }
    std::array<double,3>to_euler()const{
        double yaw,pitch,roll;
        double sin_pitch=-2*(b*d-a*c);

        if(std::abs(sin_pitch)>=0.999){
            pitch=std::copysign(M_PI/2.0,sin_pitch);
            yaw=std::atan2(-b*c-a*d,0.5-c*c-d*d);
            roll=0;
        }else{
            pitch=std::asin(sin_pitch);
            yaw=std::atan2(2*(c*d+a*b),1-2*(b*b+c*c));
            roll+std::atan2(2*(b*c+a*d),1-2*(b*b+d*d));
        }
        return{yaw,pitch,roll};
    }
    quaternion operator+(const quaternion&other)const{
        return quaternion(a+other.a,b+other.b,c+other.c,d+other.d);
    }
    quaternion operator*(const quaternion&other)const{
        return quaternion(a*other.a-b*other.b-c*other.c-d*other.d,
                          a*other.b+b*other.a+c*other.d-d*other.c,
                          a*other.c-b*other.d+c*other.a+d*other.b,
                          a*other.d+b*other.c-c*other.a+d*other.a);
    }
    double norm(){
        return pow(a*a+b*b+c*c+d*d,0.5);//模
    }
    quaternion conjugate(){
        return quaternion(a,-b,-c,-d);//共轭
    }
    quaternion inverse(){
        quaternion q_=conjugate();
        double qn=norm();
        return quaternion(q_.a/(qn*qn),q_.b/(qn*qn),q_.c/(qn*qn),q_.d/(qn*qn));//逆
    }
    quaternion rotation(const quaternion& v){//旋转
        quaternion q_=inverse();
        return (*this)*v*q_;
    }
    friend trans;
};

class pose{
    private:
    std::array<double,3>position;
    std::array<double,3>euler;
    public:
    pose():position{0,0,0},euler{0,0,0}{}
    pose(double x,double y,double z,double yaw,double pitch,double row):
    position{x,y,z},euler{yaw,pitch,row}{}    
    friend trans;
};

class trans{
    private:
    std::array<double,3>t;
    std::array<std::array<double,3>,3>R;
    public:
    trans():t{0,0,0},R{{{1,0,0},{0,1,0},{0,0,1}}}{}
    trans(const std::array<double,3>&t,const std::array<std::array<double,3>,3>R):t(t),R(R){}
    std::array<double,3>to_euler()const{
        double yaw,pitch,roll;
        pitch=std::asin(-R[2][0]);
        if(std::cos(pitch)>1e-6){
            yaw=std::atan2(R[2][1],R[2][2]);
            roll=std::atan2(R[1][0],R[0][0]);
        }else{
            yaw=0;
            roll=std::atan2(-R[0][1],R[1][1]);
        }
        return {yaw,pitch,roll};
    }
    void TF(pose B,trans B_A,pose A){
        std::array<double,3> B_A_R_e=B_A.to_euler();
        quaternion B_A_q(B_A_R_e[0],B_A_R_e[1],B_A_R_e[2]);//旋转矩阵变为四元数
        quaternion B_p_q(0,B.position[0],B.position[1],B.position[2]);//坐标B转换为四元数
        quaternion B_inv=B_A_q.rotation(B_p_q);//旋转
        A.position={B_inv.b+B.position[0],B_inv.c+B.position[1],B_inv.d+B.position[2]};
        quaternion B_e_q(0,B.euler[0],B.euler[1],B.euler[2]);//姿态转换为四元数
        A.euler=(B_A_q*B_e_q).to_euler();
    };
};


int main(){

    return 0;
}