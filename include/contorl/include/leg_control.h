
#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H
#include <vector>// 包含std::make_shared的头文件
#include "motor.h"

class leg_control
{

public:
    leg_control(const std::string& leg_name, const std::string& port,const std::vector<float>& arr)
           : _leg_name(leg_name),
             _leg_serial(std::make_shared<SerialPort>(port)),  // 初始化 SerialPort 对象
             _leg_motor0(arr[0],arr[1],_leg_serial),
             _leg_motor1(arr[2],arr[3],_leg_serial),
             _leg_motor2(arr[4],arr[5],_leg_serial)
    {
      if (arr.size()<6)
      {
        std::cout<<"Please re-enter the position that limits joint movement"<<std::endl;
        return;
      }
      _leg_motor0.motor_enable(0);
      _leg_motor1.motor_enable(1);
      _leg_motor2.motor_enable(2);
    }
    ~leg_control(){
    };
    void set_one_motor_cmd(int id,  float kp, float kd, float q, float dq, float tau);    //控制腿中的一个电机
    void leg_disable(float kd);//阻尼模式
    void leg_pose(const std::vector<float>& arr);  //位置控制
    void leg_torque(const std::vector<float>& arr); //力矩控制
    void print_legdata(); //打印整条腿数据
    void leg_limit(); //限制腿的关节位置
    std::string _leg_name;            // 定义腿的名称
    std::shared_ptr<SerialPort> _leg_serial;  // 定义 SerialPort 对象
    motor _leg_motor0;           // 定义三个个 Motor 对象
    motor _leg_motor1; 
    motor _leg_motor2;
};



#endif