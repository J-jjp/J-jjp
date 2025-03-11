#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <unistd.h>
#include <vector>
#include "../serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
class robot_control
{
private:
  /* data */
public:
  robot_control(/* args */);
  ~robot_control();
};

class Motor
{
public:
    Motor() :
        serial_0(new SerialPort("/dev/ttyUSB0")),
        serial_1(new SerialPort("/dev/ttyUSB1")),
        serial_2(new SerialPort("/dev/ttyUSB2")),
        serial_3(new SerialPort("/dev/ttyUSB3"))
    {
      for (int i = 0; i < 4; i++){
        for (int n = 0; n < 3; n++)
        {
          MotorCmd cmd_0;
          MotorData data_0;
          cmd_0.motorType = MotorType::A1;
          data_0.motorType = MotorType::A1;
          cmd_0.mode  = queryMotorMode(MotorType::A1,MotorMode::FOC);
          cmd_0.id = n;
          cmd_0.kp    = 0;
          cmd_0.kd    = 0;
          cmd_0.q     = 0;
          cmd_0.dq    = 0;
          cmd_0.tau   = 0;
          cmd.push_back(cmd_0);
          data.push_back(data_0);
        }
      }
      //发送a1电机命令
      serial_0.sendRecv(&cmd_0,&data_0);
    }
  
    ~Motor() {
        delete serial_0;
        delete serial_1;
        delete serial_2;
        delete serial_3;
    }
    SerialPort serial_choose(int id){
      switch (id)
      {
        case 0:
          return serial_0;
        break;
        case 1:
          return serial_1;
        break;
        case 2:
          return serial_2;
        break;
        case 3:
          return serial_3;
        break;
      default:
        break;
      }
    }
    void Motor_control(int id,int motor_id,int kp,float kd,float pose,float speed,float tau){
      serial = std::make_shared<SerialPort>("/dev/ttyUSB0", B1000000);
      cmd_0.q = std::clamp(cmd_0.q, -N, N);
      serial_0.sendRecv(&cmd_0,&data_0);
    }
public:

    SerialPort* serial_0;
    SerialPort* serial_1;
    SerialPort* serial_2;
    SerialPort* serial_3;
    std::vector<SerialPort> serial;
    std::vector<MotorCmd> cmd;
    std::vector<MotorData> data;
};

#endif