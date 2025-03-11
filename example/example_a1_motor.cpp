#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <yaml-cpp/yaml.h> 
#include <fstream> // 用于 std::ofstream
#include <vector>
#include "contorl/include/leg_control.h"
#define N 56.52f    //一圈最大值
#define SERVER_IP "127.0.0.1" // 本地服务器 IP
#define PORT 8080   //端口
#define Motor_lap 6.2831852




#define FL_hip_max 6.f
#define FL_hip_min -6.f
#define FL_hip_start_joint 1.42f

#define FR_hip_max 8.f
#define FR_hip_min -4.f
#define FR_hip_start_joint 2.f

#define RL_hip_max 8.f
#define RL_hip_min -4.f
#define RL_hip_start_joint 2.f

#define RR_hip_max 10.f
#define RR_hip_min -2.0f
#define RR_hip_start_joint 5.8f
// ---------------------------------------------------
#define FL_thigh_max 4.f
#define FL_thigh_min -8.f
#define FL_thigh_start_joint 1.4f

#define FR_thigh_max 14.f
#define FR_thigh_min 2.f
#define FR_thigh_start_joint 5.6f


#define RL_thigh_max 7.f
#define RL_thigh_min -5.f
#define RL_thigh_start_joint 4.9f


#define RR_thigh_max 10.0f
#define RR_thigh_min -2.0f
#define RR_thigh_start_joint 1.f
// ---------------------------------------------------
#define FL_calf_max 28.5448f
#define FL_calf_min 4.26f
#define FL_calf_start_joint 6.f

#define FR_calf_max 4.6f
#define FR_calf_min -19.f
#define FR_calf_start_joint 2.6f

#define RL_calf_max 25.9934f
#define RL_calf_min 1.6f
#define RL_calf_start_joint 2.5f

#define RR_calf_max  2.57f
#define RR_calf_min -20.8982f
#define RR_calf_start_joint 0.5f
int main() {
  YAML::Node config = YAML::LoadFile("../yaml/A1.yaml");
  //FL
  std::shared_ptr<SerialPort> serial_FL=std::make_shared<SerialPort>("/dev/ttyUSB0");
  //FR
  std::shared_ptr<SerialPort> serial_FR=std::make_shared<SerialPort>("/dev/ttyUSB1");
  //RL
  std::shared_ptr<SerialPort> serial_RL=std::make_shared<SerialPort>("/dev/ttyUSB2");

  std::shared_ptr<SerialPort> serial_RR=std::make_shared<SerialPort>("/dev/ttyUSB3");
//calf
  // std::shared_ptr<motor> motor_=std::make_shared<motor>(FR_calf_max,FR_calf_min ,serial_);
  // std::shared_ptr<motor> motor_=std::make_shared<motor>(RL_calf_max,RL_calf_min ,serial_);
  // std::shared_ptr<motor> motor_=std::make_shared<motor>(RR_calf_max,RR_calf_min ,serial_);
  //FL
  std::shared_ptr<motor> FL_motor_0=std::make_shared<motor>(FL_hip_max,FL_hip_min ,serial_FL);
  std::shared_ptr<motor> FL_motor_1=std::make_shared<motor>(FL_thigh_max,FL_thigh_min ,serial_FL);
  std::shared_ptr<motor> FL_motor_2=std::make_shared<motor>(FL_calf_max,FL_calf_min ,serial_FL);
  //FR 
  std::shared_ptr<motor> FR_motor_0=std::make_shared<motor>(FR_hip_max,FR_hip_min,serial_FR);
  std::shared_ptr<motor> FR_motor_1=std::make_shared<motor>(FR_thigh_max,FR_thigh_min ,serial_FR);
  std::shared_ptr<motor> FR_motor_2=std::make_shared<motor>(FR_calf_max,FR_calf_min ,serial_FR);
  //serial_FR
  std::shared_ptr<motor> RL_motor_0=std::make_shared<motor>(RL_hip_max,RL_hip_min,serial_RL);
  std::shared_ptr<motor> RL_motor_1=std::make_shared<motor>(RL_thigh_max,RL_thigh_min ,serial_RL);
  std::shared_ptr<motor> RL_motor_2=std::make_shared<motor>(RL_calf_max,RL_calf_min ,serial_RL);
  //RR
  std::shared_ptr<motor> RR_motor_0=std::make_shared<motor>(RR_hip_max,RR_hip_min,serial_RR);
  std::shared_ptr<motor> RR_motor_1=std::make_shared<motor>(RR_thigh_max,RR_thigh_min,serial_RR);
  std::shared_ptr<motor> RR_motor_2=std::make_shared<motor>(RR_calf_max,RR_calf_min ,serial_RR);

  // std::shared_ptr<motor> motor_0=std::make_shared<motor>(0,0 ,serial_);
  // int client_fd;
  // struct sockaddr_in server_addr;

  // // 创建套接字
  // if ((client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
  //   perror("Socket creation failed");
  //   return -1;
  // }
  // // 设置服务器地址
  // server_addr.sin_family = AF_INET;
  // server_addr.sin_port = htons(PORT);
  // if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
  //   perror("Invalid address or address not supported");
  //   close(client_fd);
  //   return -1;
  // }


  // //导入yaml文件参数
  // float kp = config["motor"]["kp"].as<float>();
  // float kd = config["motor"]["kp"].as<float>();
  // float pose = config["motor"]["pose"].as<float>();
  float _percent_0 = 0;
  float _percent_1 = 0;
  float _percent_2 = 0;
  float _duration = 100; 
  while(true) 
  {
    _percent_0 += (float)2/_duration;
    _percent_0 = _percent_0 > 1 ? 1 : _percent_0;
    _percent_1 += (float)2/_duration;
    _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
    _percent_2 += (float)1/_duration;
    _percent_2 = _percent_2 > 1 ? 1 : _percent_2;
    //FL
    float FL_pose_0 = (1 - _percent_0)*FL_hip_start_joint + _percent_0*(FL_hip_start_joint-1);
    float FL_pose_1 = (1 - _percent_1)*FL_thigh_start_joint + _percent_1*(FL_thigh_start_joint-2);
    float FL_pose_2 = (1 - _percent_2)*FL_calf_start_joint + _percent_2*(FL_calf_start_joint+11);
    //FR
    float FR_pose_0 = (1 - _percent_0)*FR_hip_start_joint + _percent_0*(FR_hip_start_joint+1);
    float FR_pose_1 = (1 - _percent_1)*FR_thigh_start_joint + _percent_1*(FR_thigh_start_joint+2);
    float FR_pose_2 = (1 - _percent_2)*FR_calf_start_joint + _percent_2*(FR_calf_start_joint-11);

    float RL_pose_0 = (1 - _percent_0)*RL_hip_start_joint + _percent_0*(RL_hip_start_joint+1);
    float RL_pose_1 = (1 - _percent_1)*RL_thigh_start_joint + _percent_1*(RL_thigh_start_joint-2);
    float RL_pose_2 = (1 - _percent_2)*RL_calf_start_joint + _percent_2*(RL_calf_start_joint+10);

    float RR_pose_0 = (1 - _percent_0)*RR_hip_start_joint + _percent_0*(RR_hip_start_joint-1);
    float RR_pose_1 = (1 - _percent_1)*RR_thigh_start_joint + _percent_1*(RR_thigh_start_joint+2);
    float RR_pose_2 = (1 - _percent_2)*RR_calf_start_joint + _percent_2*(RR_calf_start_joint-10);
    // RR_motor_0->set_motor_torque(2,0.1);
    //FL
    FL_motor_0->set_motor(0,0.015,2,FL_pose_0,-7,-0.3);
    FL_motor_1->set_motor(1,0.015,2,FL_pose_1,-7,-0.3);
    FL_motor_2->set_motor(2,0.015,2,FL_pose_2,7,0.3);
    //FR
    FR_motor_0->set_motor(0,0.015,2,FR_pose_0,7,0.3);
    FR_motor_1->set_motor(1,0.015,2,FR_pose_1,7,0.3);
    FR_motor_2->set_motor(2,0.015,2,FR_pose_2,-7,-0.3);
    //RL
    RL_motor_0->set_motor(0,0.015,2,RL_pose_0,7,0.3);
    RL_motor_1->set_motor(1,0.015,2,RL_pose_1,-7,-0.3);
    RL_motor_2->set_motor(2,0.02,2,RL_pose_2,7,0.6);
    //RR
    RR_motor_0->set_motor(0,0.015,2,RR_pose_0,-7,-0.3);
    RR_motor_1->set_motor(1,0.015,2,RR_pose_1,7,0.3);
    RR_motor_2->set_motor(2,0.02,2,RR_pose_2,-7,-0.5);
    // _percent += (float)1/_duration;
    // _percent = _percent > 1 ? 1 : _percent;
    // pose[2] = (1 - _percent)*start_pose[2] + _percent*RR_calf_joint;
    // pose[1] = (1 - _percent)*start_pose[1] + _percent*RR_thigh_joint;
    // pose[0] = (1 - _percent)*start_pose[0] + _percent*RR_hip_joint;
    // FL->leg_disable(2);
    // FL->leg_pose(pose);
    // FL->print_legdata();
    // FR->print_legdata();
    // std::cout <<  "motor.temp: "   << FL->_leg_motor0.data.temp   <<  std::endl;
    // YAML::Node config = YAML::LoadFile("../yaml/A1.yaml");//重新读取yaml文件

    // //A1电机命令配置

    // //判断是否需要更新参数      send参数为01的时候进入判断并将send参数修改为0
    // if (config["motor"]["send"].as<int>())
    // {
    //   config["motor"]["send"] = 0;
    //   std::ofstream fout("../yaml/A1.yaml");
    //   fout << config;
    //   fout.close();
    //   kp=config["motor"]["kp"].as<float>();
    //   kd = config["motor"]["kd"].as<float>();
    //   pose=pose-config["motor"]["pose"].as<float>();;
    // }
    std::cout <<  "motor.q: " << RR_motor_0->data.q <<  std::endl;
    std::cout <<  "motor.dq: " << RR_motor_0->data.dq <<  std::endl;
    //UDP发送
    // std::string str = std::to_string(data_0.q)+","+std::to_string(cmd_0.q)+"\n";
    // const char *message = str.c_str();
    // sendto(client_fd, message, strlen(message), MSG_CONFIRM, (const struct sockaddr *)&server_addr, sizeof(server_addr));
    // usleep(2000);
  }
  // close(client_fd);
  return 0;
}