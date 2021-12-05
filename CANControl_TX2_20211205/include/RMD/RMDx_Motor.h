
#ifndef RMDx_Motor_h
#define RMDx_Motor_h

#include "ros/ros.h"
#include "RMD/canalystii.h"
#include "RMD/can.h"
#include <vector>

#define recv_max_len 100   //每次CAN接收最大长度
#define torque_A_ratio 3.3   //转矩常数 Nm/A
#define R_cur_span 33      //反馈的数据范围,其中速度为1deg/LSB
#define R_cur_span_int 2048
#define R_pos_span 360    //角度
#define R_pos_span_int 16384

#define W_cur_span 32  //控制命令的范围
#define W_cur_span_int 2000  //控制命令的范围
#define W_vel_ratio 0.01   //其中速度为0.01dps/LSB
#define W_pos_ratio 0.01   //其中速度为0.01deg/LSB

class RMDx_Motor {
public:
    // 
    RMDx_Motor(ros::NodeHandle &nh, CANalystii &USB_CAN, unsigned int can_channel_ID);  
    
    int posKp, posKi, velKp, velKi, iqKp, iqKi; //各环PI参数，这里是整型
    double pos_reply, vel_reply, cur_reply;
    bool first_data_flag_;
    double angle_bias;
    double angle, last_pos_reply_;
    int n_round;

    // Commands
    bool Init();
    void readState();
    int64_t read_angle_mul();
    void clearState();
    void closeMotor();
    void stopMotor(int mode);
    void fstopMotor();
    void runMotor();
    void readPID();
    void writePID();
    void getState();
    void writeCurrent(int16_t current);
    void writeVelocity(int32_t velocity); 
    void writePosition(int32_t position);

    std::vector<double> MygetState();
    std::vector<double> MyreadState();

private:
    ros::NodeHandle nh_; 
    CANalystii &USB_CAN_; 

    const int motor_ID;   //default:=0x141
    unsigned int CAN_Channel_ID; //电机设备所在的通道，CAN通道，0为CAN1,1为CAN2
    VCI_CAN_OBJ write_cmd_;  
    VCI_CAN_OBJ recv_msg[recv_max_len];  //接收缓冲区
    unsigned int recv_len; //接收到的帧数
    void writeCmd(VCI_CAN_OBJ write_cmd);
    bool readMsg(unsigned int& recv_len);
    int double_to_uint(double x, double span, int span_int); 
    double uint_to_double(int x_int, double span, int span_int);
};

#endif
