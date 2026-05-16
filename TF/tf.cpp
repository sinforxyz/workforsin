#include<iostream>
#include<array>
#include<cmath>
#include<map>
#include<limits>
#include"tf.hpp"


int main(){
    std::cout<<"初始坐标系为Camera\n";
    std::cout<<"请输入坐标系间的转换关系\n格式:转换前的坐标系 转换后的坐标系 x y z yaw pitch roll(输入end以停止)"<<std::endl;
    std::string s1,s2;
    double x_s,y_s,z_s,yaw_s,pitch_s,roll_s;
    std::map<std::string,std::map<std::string,trans>> b_a;
    while(std::cin>>s1) {
        if(s1=="end")break;
        std::cin>>s2>>x_s>>y_s>>z_s>>yaw_s>>pitch_s>>roll_s;
        b_a[s1][s2] =trans({x_s,y_s,z_s},{yaw_s,pitch_s,roll_s});
    }
    std::cin.clear();

    double x,y,z,yaw,pitch,roll;
    
    std::string line;
    std::cout<<"输入当前坐标\n";
    std::cin>>x>>y>>z>>yaw>>pitch>>roll;
    pose b{x,y,z,yaw,pitch,roll},a;
    std::cout<<"输入目标坐标系\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    std::getline(std::cin,line);
    size_t pos=line.find('/');
    std::string b_a_t=line.substr(pos+1);
    std::string now="Camera";
    auto it=b_a.find(now);
    while(it->second.find(b_a_t)==it->second.end()){
        now=it->second.begin()->first;
        it=b_a.find(now);
        it->second.begin()->second.TF(b,a);
        b=a;
    }
    std::cout<<"output:"<<std::endl;
    a.print();
    return 0;
}