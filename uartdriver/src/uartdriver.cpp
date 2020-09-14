#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sensor_msgs/Imu.h>
#include "nav_msgs/Odometry.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <tf/transform_broadcaster.h>
static const Eigen::Vector3d NED_ENU_RPY(M_PI, 0, M_PI_2);
class jiexi{
    public: 
    std::vector<double> jiexiuart(const std::string &buff);
    Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy);
    Eigen::Quaterniond toENU(const Eigen::Quaterniond &q);
    private:
    std::vector<double> imu;
    std::vector<double> null;
    std::vector<int>    fengepos;
    const std::string fenge=",";
    int position=0;
    int i=0,j=0;
};
Eigen::Quaterniond jiexi::fromRPY(const Eigen::Vector3d &rpy){
    return Eigen::Quaterniond(Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}
Eigen::Quaterniond jiexi::toENU(const Eigen::Quaterniond& q) {
  Eigen::Quaterniond result;
  Eigen::Quaterniond ned_enu_q = fromRPY(NED_ENU_RPY);
  result = ned_enu_q * q;
  return result;
}
std::vector<double> jiexi::jiexiuart(const std::string &buff){
    //std::cout<<"buff.empty():"<<buff.empty()<<",buff.at(0)"
    //<<buff.at(0)<<"buff.at(buff.size()-1)::"<<buff.at(buff.size()-3)<<std::endl;
    //for(int i=0;i<sizeof(buff);i++){
    //printf("i%d=%c ",i,buff[i]);
    //}
    //printf("\r\n");
      if(buff.empty())return null;
    if(buff.at(1)!=',')return null;
    if(buff.at(buff.size()-3)!=',')return null;
     
 
    //std::cout<<"step1!!"<<std::endl;
    while((position=buff.find(fenge,position))!=std::string::npos){
        fengepos.push_back(position);
        position++;
        i++;
    }
    //std::cout<<"step2!!"<<std::endl;
    //std::cout<<"fengepos.size()"<<fengepos.size()<<std::endl;
    if(fengepos.size()!=11){return null;}
    imu.clear();
    for(j=0;j<fengepos.size()-1;j++){
        
        imu.push_back(std::stod(buff.substr(fengepos.at(j)+1,fengepos.at(j+1)-1)));
        
       // catch(std::exception &e){
        //    return null;
       // }
        std::cout<<imu[j] ;   
    }
    std::cout<<"step3!!,imusize="<<imu.size()<<std::endl;
    //fengepos.clear();
    return imu;


}


geometry_msgs::PoseStamped pose;
std::string sendbuff=",123,456,7890,";
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr msg){
    std::string x,y,z;
    pose=*msg;
    
    x=std::to_string(int( pose.pose.position.x*100));
    y=std::to_string(int(pose.pose.position.y*100));
    z=std::to_string(int(pose.pose.position.z*100));
    sendbuff=','+x+','+y+','+z+',';
    std::cout<<pose.pose.position.x<<pose.pose.position.y<<pose.pose.position.z<<std::endl;
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"uart_node");
    ros::NodeHandle nh;
    serial::Serial sp;
    serial::Timeout to=serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    ros::Subscriber pose_sub =nh.subscribe<geometry_msgs::PoseStamped>
            ("ORB_SLAM/pose",14,pose_cb);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data",10);
    ros::Publisher ov_pub=nh.advertise<nav_msgs::Odometry>("vo",10);
    ros::Publisher ov_pub2=nh.advertise<nav_msgs::Odometry>("vo2",10);
    ros::Rate loop_rate(100);
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry odom2;
    imu.header.frame_id="base_footprint";
    imu.orientation_covariance={1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu.angular_velocity_covariance={1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu.linear_acceleration_covariance={-1,0,0,0,0,0,0,0,0};
    odom.header.frame_id="base_footprint";
    odom.child_frame_id="base_footprint";
    odom.pose.covariance={1e-3, 0, 0, 0, 0, 0, 
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};
    odom2.header.frame_id="base_footprint";
    odom2.child_frame_id="base_footprint";
    odom2.pose.covariance={1e-3, 0, 0, 0, 0, 0, 
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};
    /*odom.twist.covariance={1e-3, 0, 0, 0, 0, 0, 
                           0, 1e-3, 0, 0, 0, 0,
                           0, 0, 1e6, 0, 0, 0,
                           0, 0, 0, 1e6, 0, 0,
                           0, 0, 0, 0, 1e6, 0,
                           0, 0, 0, 0, 0, 1e3};
                           */


    

    try {
         //open serial
         sp.open();
    }
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("unable to open port.");
        return -1;
    }
    if(sp.isOpen()){
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened^-^");
    }
    else{
        return -1;
    }
    
    std::vector<double> imu_raw;
    geometry_msgs::PoseStamped imupos;
    while(ros::ok()){
        jiexi jiexi;
        size_t n=sp.available();
        std::cout<<"available::"<<n<<std::endl;
        std::string buff;
        if(n!=0){
            n=sp.readline(buff);std::cout<<"rawbuff="<<buff<<std::endl;
            imu_raw=jiexi.jiexiuart(buff);
            if(imu_raw.empty())continue;
            for(double i:imu_raw)std::cout<<i;//std::cout<<std::endl;
            std::cout<<std::endl;
            //imu.clear();
            
            imu.orientation.x=imu_raw.at(1)/1000;
            imu.orientation.y=imu_raw.at(2)/1000;
            imu.orientation.z=imu_raw.at(3)/1000;
            imu.orientation.w=imu_raw.at(0)/1000;
            imu.linear_acceleration.x=imu_raw.at(4)/100;
            imu.linear_acceleration.y=imu_raw.at(5)/100;
            imu.linear_acceleration.z=imu_raw.at(6)/100;
            imu.angular_velocity.x=imu_raw.at(7)/1000;
            imu.angular_velocity.y=imu_raw.at(8)/1000;
            imu.angular_velocity.z=imu_raw.at(9)/1000;
            imu.header.stamp = ros::Time::now();
            imu_pub.publish(imu);
            
        }   
        Eigen::Quaterniond quain(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
        Eigen::Quaterniond quaout(1,0,0,0);
        quaout=jiexi.toENU(quain);
        odom.pose.pose.position.x=pose.pose.position.x;
        odom.pose.pose.position.y=pose.pose.position.y;
        odom.pose.pose.position.z=pose.pose.position.z;
        odom.pose.pose.orientation.x=pose.pose.orientation.x;
        odom.pose.pose.orientation.y=pose.pose.orientation.y;
        odom.pose.pose.orientation.z=pose.pose.orientation.z;
        odom.pose.pose.orientation.w=pose.pose.orientation.w;
        
        odom.pose.pose.orientation.x=0;
        odom.pose.pose.orientation.y=0;
        odom.pose.pose.orientation.z=0;
        odom.pose.pose.orientation.w=1;
        
        odom.header.stamp= ros::Time::now();

        odom2.pose.pose.position.x=pose.pose.position.x;
        odom2.pose.pose.position.y=pose.pose.position.y;
        odom2.pose.pose.position.z=pose.pose.position.z;
        odom2.pose.pose.orientation.x=pose.pose.orientation.x;
        odom2.pose.pose.orientation.y=pose.pose.orientation.y;
        odom2.pose.pose.orientation.z=pose.pose.orientation.z;
        odom2.pose.pose.orientation.w=pose.pose.orientation.w;
        

        
        odom2.header.stamp= ros::Time::now();
        //std::cout<<"odom.pose.pose.position.x"<<odom.pose.pose.position.x<<std::endl;
        ov_pub.publish(odom);
        ov_pub2.publish(odom2);
        int x=(int)(odom.pose.pose.position.x*1000);
        int y=(int)(odom.pose.pose.position.y*1000);
        int z=(int)(odom.pose.pose.position.z*1000);
        //std::cout<<"x="<<x<<" y="<<y<<" z="<<z<<std::endl;
        u_char xyz[12];
        xyz[0]=0xAA;xyz[1]=0xAA;xyz[2]=0xF3;xyz[3]=0x00;
        xyz[4]=x>>8;xyz[5]=x;xyz[6]=y>>8;xyz[7]=y;
        xyz[8]=z>>8;xyz[9]=z;xyz[10]=0;xyz[11]=0x00;
        sp.write(xyz,12);
        //sp.write(sendbuff);
        ros::spinOnce();
        loop_rate.sleep();
    }
    sp.close();
    return 0;
}
