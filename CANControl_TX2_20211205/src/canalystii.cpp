#include "RMD/canalystii.h"

CANalystii::CANalystii(int vci_device_type, int vci_device_ind):can_ind_port_({0,1}){
    vci_device_type_ = vci_device_type;
    vci_device_ind_ = vci_device_ind;
    is_dev_start_ = false;
    
        vci_conf_.AccCode = 0x80000008;  //
        vci_conf_.AccMask = 0xFFFFFFFF;  //接收所有ID的报文数据
        vci_conf_.Filter = 1;//receive all frames 接收标准帧和扩展帧
        vci_conf_.Timing0 = 0x00;  
        vci_conf_.Timing1 = 0x14;//baudrate 1000kbps
        vci_conf_.Mode = 0;//normal mode 收发进行
    for(int i=0;i<2;i++){
        is_port_init_[i] = false;
        is_port_start_[i] = false;
    }
}

CANalystii::~CANalystii(){
    if(is_dev_start_ == true){
        close_device();
    }
}
//开启USB转CAN设备
bool CANalystii::start_device(){
    if(!is_dev_start_){
        is_dev_start_ = (VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0)==1);
    }
    return is_dev_start_;
}

bool CANalystii::close_device(){
   
    if(is_dev_start_){
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
        is_dev_start_ = !(VCI_CloseDevice(vci_device_type_, vci_device_ind_)==1);
    }
    is_port_init_[0] = false;
    is_port_init_[1] = false;
    is_port_start_[0] = false;
    is_port_start_[1] = false;
    return !is_dev_start_;
}
//初始化选定的CAN通道，
bool CANalystii::init_can_interface(unsigned int can_idx){
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;
    //vci_conf_= vci_conf;
    is_port_init_[can_port_idx] = (VCI_InitCAN(vci_device_type_, vci_device_ind_, can_port_idx,&vci_conf_)==1);
    is_port_start_[can_port_idx] = (VCI_StartCAN(vci_device_type_, vci_device_ind_, can_port_idx)==1);
    VCI_ClearBuffer(vci_device_type_, vci_device_ind_, can_port_idx);
    VCI_ClearBuffer(vci_device_type_, vci_device_ind_, can_port_idx);
    return is_port_init_[can_port_idx]&&is_port_start_[can_port_idx];
}

unsigned int CANalystii::receive_can_frame(unsigned int can_idx, VCI_CAN_OBJ *recv_obj, unsigned int recv_len, int wait_time){
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;
    unsigned int receive_len = 0;
    receive_len = VCI_Receive(vci_device_type_, vci_device_ind_, can_port_idx, recv_obj, recv_len, wait_time);
    return receive_len;
}

bool CANalystii::send_can_frame(unsigned int can_idx, VCI_CAN_OBJ *send_obj, unsigned int send_len){
    //TODO: (vincent.cheung.mcer@gmail.com) Not yet implemented.
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;    
    bool status;
    status = (VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx, send_obj, send_len)==1);
    return status;
}
