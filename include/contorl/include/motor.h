#ifndef MOTOR_H
#define MOTOR_H
#include "unitreeMotor/unitreeMotor.h"
#include <string> // 包含std::string的头文件
#include "serialPort/SerialPort.h"
#include <memory>
#include <iostream>
#include <algorithm>
class motor
{

public:
    motor(float max_,float min_,std::shared_ptr<SerialPort> serial = nullptr) 
    : max_pose(max_),
      min_pose(min_),
      serial_(serial)
    {
        if (serial_ == nullptr)
        {
            serial_ = std::make_shared<SerialPort>("/dev/ttyUSB0");
        }
    };
    ~motor()
    {}
    void motor_enable(int n) // 添加电机id
    {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = n;
    }
    void set_motor(int id, float kp, float kd, float q, float dq, float tau) // 力位混合控制
    {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = id;
        cmd.kp=std::max(-0.03f, std::min(kp, 0.03f));
        cmd.kd=std::max(-5.f, std::min(kd, 5.f));
        cmd.tau=std::max(-0.5f, std::min(tau, 0.5f));
        cmd.q = q;
        cmd.dq = dq;
        std::cout<<"cmd.q"<<q<<std::endl;
        if (motor_limit(id)){
            // std::cout <<  "motor.kp: "    << cmd.kp    <<  std::endl;
            // std::cout <<  "motor.kd: " << cmd.kd <<  std::endl;
            std::cout <<  "motor.cmd: "    << cmd.q     <<  std::endl;
            std::cout <<  "motor.dq: " << cmd.dq <<  std::endl;
            // std::cout <<  "motor.tau: " << cmd.tau <<  std::endl;
            motor_sendRecv();
        }
    }
    void set_motor_pose(int n, float pose) // 位置模式
    {   
        motor_enable(n);
        cmd.kp = rotor_kp;
        cmd.kd = rotor_kd;
        cmd.q = pose;
        cmd.dq = 0.0;
        cmd.tau = 0;
        if (motor_limit(n))
            motor_sendRecv();
    }
    void set_motor_disable(int n, float kd) // 阻尼模式
    {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = n;
        cmd.kp = 0.;
        cmd.kd = kd;
        cmd.q = 0;
        cmd.dq = 0.0;
        cmd.tau = 0.0;
        motor_sendRecv();
        // usleep(200);
    }
    void set_motor_torque(int n, float t) // 力矩模式
    {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = n;
        cmd.kp = 0.;
        cmd.kd = 0;
        cmd.q = 0;
        cmd.dq = 0.0;
        cmd.tau = t;
        cmd.tau=std::max(-0.5f, std::min(cmd.tau, 0.5f));
        motor_sendRecv();
    }

public:
    MotorCmd cmd;
    MotorData data;
    std::shared_ptr<SerialPort> serial_;
    float output_kp = 25;
    float output_kd = 0.6;
    float gear_ratio = queryGearRatio(MotorType::A1);
    float rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
    float rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;
    float max_pose;
    float min_pose;
private:
    bool motor_limit(int n)//限制力矩
    {
        if (max_pose==0||min_pose==0)
        {
            set_motor_torque(n,0.2);
            return false;
        }
        // if(data.q<min_pose+1.5||data.q>max_pose-1.5){
        if(data.q<min_pose+0.5||data.q>max_pose-0.5){
            if(data.tau>1||data.tau<-1){
                set_motor_disable(n,3);
                return false;
            }
        }
        //     if(data.tau>1){
        //         cmd.tau-=0.1;
        //     }
        //     else if(data.tau<-1){
        //         cmd.tau+=0.1;
        //     }
        // }
        // cmd.q=std::max(min_pose, std::min(cmd.q, max_pose));
        return true;
    }
    void motor_sendRecv() // 发送数据
    {
        if (cmd.motorType == MotorType::A1 && cmd.mode == queryMotorMode(MotorType::A1, MotorMode::FOC))
        {
            serial_->sendRecv(&cmd, &data);
            usleep(200);
        }
        else
        {
            std::cout << "Please initialize the motor" << std::endl;
        }
        // usleep(200);
    }
};
#endif
//使用示例
//1.无最大值和最小值时
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(0,0,serial_);

//     while (1)
//     {
//         motor_->set_motor_torque(0,0.2);
//         std::cout<<"motor.q"<<motor_->data.q<<std::endl;
//     }
    
// }
//2.力位混合控制 以小腿为例
// #define calf_max 28.5448f
// #define calf_min 4.52717f
// #define calf_joint 20.f
// #define calf_start_joint 4.f
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(calf_max,calf_min,serial_);
// //   //calf_max calf_min为小腿范围
//     float _percent = 0;
//     float _duration = 10; 
//     while (1)
//     {
//         _percent += (float)1/_duration;
//         _percent = _percent > 1 ? 1 : _percent;
//         float pose = (1 - _percent)*calf_start_joint + _percent*calf_joint;
//         motor_->set_motor(2,0.015,3,pose,7,0.3);
//     }
// }




