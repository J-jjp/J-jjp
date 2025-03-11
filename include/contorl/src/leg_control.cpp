#include "../include/leg_control.h"


void leg_control::set_one_motor_cmd(int id,  float kp, float kd, float q, float dq, float tau){
    if (id==0)
    {
        _leg_motor0.set_motor(id ,kp ,kd ,q ,dq ,tau);
        _leg_serial->sendRecv(&_leg_motor0.cmd,&_leg_motor0.data);
    }
    else if (id==1)
    {
        _leg_motor1.set_motor(id ,kp ,kd ,q ,dq ,tau);
        _leg_serial->sendRecv(&_leg_motor1.cmd,&_leg_motor1.data);
    }
    else if (id==2)
    {
        _leg_motor2.set_motor(id ,kp , kd ,q ,dq ,tau);
        _leg_serial->sendRecv(&_leg_motor2.cmd,&_leg_motor2.data);
    }
    else{
        std::cout<<"id does not exist"<<std::endl;
    }
}
void leg_control::print_legdata(){
    std::cout<< "motor.q: "<<_leg_motor0.data.q<<"\t"<<_leg_motor1.data.q<<"\t\t"<<_leg_motor2.data.q<<std::endl;
}
void leg_control::leg_disable(float kd){
    _leg_motor0.set_motor_disable(0,kd);
    _leg_motor1.set_motor_disable(1,kd);
    _leg_motor2.set_motor_disable(2,kd);
}
void leg_control::leg_pose(const std::vector<float>& arr){
    _leg_motor0.set_motor_pose(0,arr[0]);
    _leg_motor1.set_motor_pose(1,arr[1]);
    _leg_motor2.set_motor_pose(2,arr[2]);
}
void leg_control::leg_torque(const std::vector<float>& arr){
    _leg_motor0.set_motor_torque(0,arr[0]);
    _leg_motor1.set_motor_torque(1,arr[1]);
    _leg_motor2.set_motor_torque(2,arr[2]);
}
void leg_control::leg_limit(){
}