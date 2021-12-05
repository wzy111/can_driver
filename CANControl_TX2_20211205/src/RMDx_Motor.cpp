
#include "RMD/RMDx_Motor.h"

// constructor
RMDx_Motor::RMDx_Motor(ros::NodeHandle &nh, CANalystii &USB_CAN, unsigned int can_channel_ID) 
    :USB_CAN_(USB_CAN), nh_(nh),motor_ID(0x141)
    {
        
        //motor_ID = 0x141;  //
        CAN_Channel_ID = can_channel_ID; //初始化ID值
        //配置CAN帧格式，标准帧，标识符
        write_cmd_.ID=motor_ID;  //帧ID
        write_cmd_.SendType=0;  //发送帧类型，0为正常模式，失败会自动重发，1为单次发送，不关心节点是否接收，提高发送速度
        write_cmd_.RemoteFlag=0;  //非远程帧
        write_cmd_.ExternFlag=0;  //非扩展帧
        write_cmd_.DataLen=8;  //数据长度
		//
        posKp=0; posKi=0; velKp=0; velKi=0; iqKp=0; iqKi=0;
        pos_reply =0; vel_reply =0; cur_reply =0;

        first_data_flag_ = true;
        n_round = 0;
    }


bool RMDx_Motor::Init(){
    // if(!USB_CAN_.start_device()){  //启动USB转CAN设备
    //     ROS_ERROR("device starts error");  
    //     return FALSE;    
    // }
    if(!USB_CAN_.init_can_interface(CAN_Channel_ID)){
        ROS_ERROR("device port init error");
        return FALSE;
    }
    return TRUE;
}

void RMDx_Motor::clearState() {
    write_cmd_.Data[0] = 0x9B;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
}
//关闭电机
void RMDx_Motor::closeMotor() {
    write_cmd_.Data[0] = 0x80;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
}
//停止电机，此处需要判断当前状态，进行紧急停止还是缓慢停止
void RMDx_Motor::stopMotor(int mode) {      //0:急停；s1:电流模式；2,速度模式， 3位置模式   
    double step_cmd, tmp_cmd;
    int step = 100;
    //cur_reply = -50;
    switch(mode){
        case 1:
            step_cmd = ((double)cur_reply)/step;
            tmp_cmd=(double)cur_reply;     
            //while((tmp_cmd-=(max_cmd/step))>0)
            while(ros::ok()&&fabs(tmp_cmd)>1)
            { 
                writeCurrent((int)tmp_cmd);
                ROS_INFO_STREAM("tmp_cmd:"<<((int)tmp_cmd));              
                usleep(10000);
                tmp_cmd-=step_cmd;
            }
            break;
        case 0: fstopMotor();break;
    }
    fstopMotor();
}
//停止电机，此处需要判断当前状态，进行紧急停止还是缓慢停止
void RMDx_Motor::fstopMotor() {     
    write_cmd_.Data[0] = 0x81;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
}

//关闭电机
void RMDx_Motor::runMotor() {
    write_cmd_.Data[0] = 0x88;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
}

std::vector<double> RMDx_Motor::MygetState() {   
    write_cmd_.Data[0] = 0x9c;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
    usleep(5000);
    return MyreadState();
}

std::vector<double> RMDx_Motor::MyreadState() {
    //unsigned int recv_len=0; //接收到的帧数
    //while(recv_len>0) readMsg(recv_len);
    std::vector<double> state(3,0);
    if(readMsg(recv_len)){
        int16_t cur_int = recv_msg[recv_len-1].Data[2] + (recv_msg[recv_len-1].Data[3] << 8);  //
        int16_t vel_int = recv_msg[recv_len-1].Data[4] + (recv_msg[recv_len-1].Data[5] << 8);
        uint16_t pos_int = recv_msg[recv_len-1].Data[6] + (recv_msg[recv_len-1].Data[7] << 8);  

        cur_reply = uint_to_double(cur_int,R_cur_span,R_cur_span_int);
        //tor_reply = cur_reply * torque_A_ratio;
        vel_reply =(double)vel_int;  
        pos_reply = pos_int/65536.0*360;
        //类型转换

        state[0] = cur_reply;
        state[1] = vel_reply;
        state[2] = pos_reply;

        if(first_data_flag_) { angle_bias = pos_reply/6.0; first_data_flag_ = false;}
        else
        {
            if(last_pos_reply_ > 300 && pos_reply < 60) ++n_round;
            if(last_pos_reply_ < 60 && pos_reply > 300) --n_round;
            angle = (360 * n_round + pos_reply)/6 - angle_bias;
        }
        last_pos_reply_ = pos_reply;
    }
    return state;       
}

//单独读取电机状态的发送指令
void RMDx_Motor::getState() {   
    write_cmd_.Data[0] = 0x9c;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    writeCmd(write_cmd_);
    usleep(5000);
    readState();
}

// 读取运行时的驱动回复，返回传感器数据
void RMDx_Motor::readState() {
    //unsigned int recv_len=0; //接收到的帧数
    //while(recv_len>0) readMsg(recv_len);
    if(readMsg(recv_len)){
        int16_t cur_int = recv_msg[recv_len-1].Data[2] + (recv_msg[recv_len-1].Data[3] << 8);  //
        int16_t vel_int = recv_msg[recv_len-1].Data[4] + (recv_msg[recv_len-1].Data[5] << 8);
        uint16_t pos_int = recv_msg[recv_len-1].Data[6] + (recv_msg[recv_len-1].Data[7] << 8);  

        cur_reply = uint_to_double(cur_int,R_cur_span,R_cur_span_int);
        //tor_reply = cur_reply * torque_A_ratio;
        vel_reply =(double)vel_int;  
        pos_reply = pos_int/65536.0*360;
        //类型转换    

        if(first_data_flag_) { angle_bias = pos_reply/6.0; first_data_flag_ = false;}
        else
        {
            if(last_pos_reply_ > 300 && pos_reply < 60) ++n_round;
            if(last_pos_reply_ < 60 && pos_reply > 300) --n_round;
            angle = (360 * n_round + pos_reply)/6 - angle_bias;
        }
        last_pos_reply_ = pos_reply;
    }       
}

// 读取电机PID
void RMDx_Motor::readPID() {
    write_cmd_.Data[0] = 0x30;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    // Send message
    writeCmd(write_cmd_);
    usleep(5000);
    //unsigned int recv_len=0; //接收到的帧数
    if(readMsg(recv_len)){
        unsigned char cmd_byte = recv_msg[recv_len-1].Data[0];
        posKp = recv_msg[recv_len-1].Data[2];
        posKi = recv_msg[recv_len-1].Data[3];
        velKp = recv_msg[recv_len-1].Data[4];
        velKi = recv_msg[recv_len-1].Data[5];
        iqKp  = recv_msg[recv_len-1].Data[6];
        iqKi  = recv_msg[recv_len-1].Data[7];
    }
}

int64_t RMDx_Motor::read_angle_mul()
{
    write_cmd_.Data[0] = 0x92;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = 0x00;
    write_cmd_.Data[5] = 0x00;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;
    // Send message
    writeCmd(write_cmd_);   //发两次读一次，保证能读到数据
    writeCmd(write_cmd_);
    usleep(5000);
    int64_t angle_mul = 0;
    if(readMsg(recv_len)){
        unsigned char cmd_byte = recv_msg[recv_len-1].Data[0];
        if(cmd_byte == 0x92){
            if(recv_msg[recv_len-1].Data[7] & 0x80) 
                angle_mul = angle_mul + 0xff;
            else 
                angle_mul = angle_mul + 0x00;
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[7] & 0xff);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[6]);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[5]);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[4]);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[3]);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[2]);
            angle_mul = angle_mul<<8;
            angle_mul = angle_mul + (recv_msg[recv_len-1].Data[1]);
        }
        else angle_mul = -1;
    } 
    if(angle_mul == 0) angle_mul = -1;  //因为发了两次，所以可能会读到自己发过去的，所以要避免这种情况
    return angle_mul;
}

void RMDx_Motor::writePID() {
    write_cmd_.Data[0] = 0x31;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = posKp;
    write_cmd_.Data[3] = posKi;
    write_cmd_.Data[4] = velKp;
    write_cmd_.Data[5] = velKi;
    write_cmd_.Data[6] = iqKp;
    write_cmd_.Data[7] = iqKi;

    // Send message
    writeCmd(write_cmd_);
}

void RMDx_Motor::writeCurrent(int16_t current) {
    // current control is int16_t type. (2byte符号整数)
    write_cmd_.Data[0] = 0xA1;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = current & 0xFF;
    write_cmd_.Data[5] = (current >> 8) & 0xFF;
    write_cmd_.Data[6] = 0x00;
    write_cmd_.Data[7] = 0x00;

    // Send message
    writeCmd(write_cmd_);
}

void RMDx_Motor::writeVelocity(int32_t velocity) {
    // velocity control is int32_t type. (4byte符号整数)
    //int32_t velocity = (int32_t)(vel/W_vel_ratio)
    write_cmd_.Data[0] = 0xA2;
    write_cmd_.Data[1] = 0x00;
    write_cmd_.Data[2] = 0x00;
    write_cmd_.Data[3] = 0x00;
    write_cmd_.Data[4] = velocity & 0xFF;
    write_cmd_.Data[5] = (velocity >> 8) & 0xFF;
    write_cmd_.Data[6] = (velocity >> 16) & 0xFF;
    write_cmd_.Data[7] = (velocity >> 24) & 0xFF;

    // Send message
    writeCmd(write_cmd_);
}

void RMDx_Motor::writePosition(int32_t position) {
  // position control is int32_t type. (4byte符号整数)
  write_cmd_.Data[0] = 0xA3;
  write_cmd_.Data[1] = 0x00;
  write_cmd_.Data[2] = 0x00;
  write_cmd_.Data[3] = 0x00;
  write_cmd_.Data[4] = position & 0xFF;
  write_cmd_.Data[5] = (position >> 8) & 0xFF;
  write_cmd_.Data[6] = (position >> 16) & 0xFF;
  write_cmd_.Data[7] = (position >> 24) & 0xFF;

  // Send message
  writeCmd(write_cmd_);
}


// Private
void RMDx_Motor::writeCmd(VCI_CAN_OBJ write_cmd) {
    // CAN通信
    unsigned int send_len = 1;  //每次发送单帧，提高效率
    //VCI_CAN_OBJ send_can_obj[1];
    if(USB_CAN_.send_can_frame(CAN_Channel_ID,&write_cmd,send_len)==-1)
        ROS_INFO(" No USB-CAN Device Found!");//时间标识。
    else
        //ROS_INFO("CAN send frame!");
        ;
}

bool RMDx_Motor::readMsg(unsigned int& recv_len) {
    int WaitTime = 10;  
    recv_len=0;
    //如果一个通道上多个设备，接收部分的代码需要调整，要么在CAN类里加个线程，要么再写一个根据从设备数量的数据处理的函数
    recv_len = USB_CAN_.receive_can_frame(CAN_Channel_ID,recv_msg,recv_max_len);//调用接收函数,读取通道的所有指定ID的数据，如果有数据，进行数据处理显示。
    if(recv_len>0){ 
        return TRUE;     //
    }       
    else{
       // ROS_INFO("NO Data from BUS!");//时间标识。
        return FALSE;
    }
}

int RMDx_Motor::double_to_uint(double x, double span, int span_int)
{
    return (int)((x)*((double)span_int)/span);
}
double RMDx_Motor::uint_to_double(int x_int, double span, int span_int)
{
    return ((double)x_int)*span/((double)span_int);
}
