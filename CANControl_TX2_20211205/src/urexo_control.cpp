#include "ros/ros.h"
#include "RMD/RMDx_Motor.h"

//上臂和下臂机械部分的重量和重心

int main(int argc, char **argv){
    ros::init(argc, argv, "urexo_control");
    ros::NodeHandle nh;
    //ros::Rate Mloop_rate(125);

    //处理订阅信息
    /*ros::Subscriber sub_TCP_pose = nh.subscribe("/tool_position",5,TCP_pose_CB);                //订阅工具坐标系位置
    ros::Subscriber sub_TCP_vec = nh.subscribe("/tool_velocity",5,TCP_vec_CB);                  //订阅工具坐标系速度
    ros::Subscriber sub_joint_data = nh.subscribe("/joint_states",5,joint_data_CB);             //订阅UR关节角度
    ros::Subscriber sub_upper_arm_data = nh.subscribe("/netft_data1",5,upper_wrench_CB);        //订阅上臂力矩
    ros::Subscriber sub_forearm_data = nh.subscribe("/netft_data2",5,fore_wrench_CB);           //订阅前臂力矩
    ros::Subscriber sub_joint = nh.subscribe("/stm32_sensor",5,elbow_angle_CB);                 //订阅肘关节角度
    ros::Publisher  pub_URScript = nh.advertise<std_msgs::String>("/ur_driver/URScript", 5);    //发布UR脚本话题,用以发送speedl
    ros::ServiceClient payload_client = nh.serviceClient<ur_msgs::SetPayload>("ur_driver/set_payload"); //更新UR负载重量和质心
*/

    //肘关节电机连接初始化
    CANalystii usb_can;            //负责can通讯协议
    if(!usb_can.start_device()){   //启动USB转CAN设备
        ROS_ERROR("USB2CAN device starts error"); 
        return -1;  
    }
    RMDx_Motor elbow_motor(nh,usb_can,0); //控制肘关节电机
    if(!initElbowMotor(nh,elbow_motor)) return -1;
    
    int key;
    ROS_INFO("Enter any key to start robot.");
    cin >> key;

    while(ros::ok()){
        //ros::spinOnce();
        int64_t start_p = elbow_motor.read_angle_mul();
        //elbow_motor.writeVelocity(20);  
        //elbow_motor.readState();    //reflash the elbow motor state
        
        Mloop_rate.sleep();
    }

    closeElbowMotor(usb_can,elbow_motor);
    return 0;
}

void closeElbowMotor(CANalystii& usb_can,RMDx_Motor& elbow_motor)
{
    elbow_motor.writeVelocity(0);  
 	usleep(500000);//延时单位us，0.5s后关闭接收线程，并退出主程序。
    elbow_motor.stopMotor(1); //电机退出操作模式
    elbow_motor.closeMotor();   
    usb_can.close_device();//关闭CAN
}
