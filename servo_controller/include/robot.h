#ifndef _ROBOT_H_
#define _ROBOT_H_   
#include "JAKAZuRobot.h"
#include "jktypes.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <vector>
#include <cstring>
#include <functional>
//#include <nlohmann/json.hpp>
#include <fstream>



#define FRAME_HEAD1 0xEB
#define FRAME_HEAD2 0x90

#define FRAME_HEAD1_RT 0x90
#define FRAME_HEAD2_RT 0xEB

#define REGISTER_ANGLE_ACT 1546
#define REGISTER_FORCE_ACT 1582
#define REGISTER_TEMPLE 1618
#define CMD_HANDG3_READ 0x11  // 读三代手内部寄存器
#define CMD_HANDG3_WRITE 0x12 // 写三代手内部寄存器
#define ANGLE_SET 1486
#define SPEED_SET 1522

/**********modbusRTU********/

#define SLAVE_STATION_ADD 0x01
#define FUNCTION_READ_REGISTER 0x03
#define FUNCTION_SINGLE_REGISTER 0x06
#define FUNCTION_MULTI_REGISTER 0x10
/**********modbusRTU ANGLE_SET********/
#define ANGLE_SET_H 0x05
#define ANGLE_SET_L 0xCE

#define REGISTER_FORCE_GRIPPER 10


class Robot {
    public:
        Robot();
        ~Robot();
#if defined(ARCH_ARM64)
        errno_t login_in(const char *ip);
#else
        errno_t login_in(const char *ip, int port);
#endif
        errno_t power_on();
        errno_t power_off();
        errno_t enable_robot();
        errno_t disable_robot();

        errno_t set_user_frame_id(int id);
        errno_t clear_error();

        errno_t get_joint_position(JointValue *joint_pos);
        errno_t get_tcp_position(CartesianPose *cart_pos);
    	errno_t get_robot_state(RobotState *state);
        errno_t get_robot_status(RobotStatus *status);
        errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed);


        errno_t servo_move_enable(BOOL is_enable);
        errno_t servo_p(CartesianPose *cart_pos, MoveMode move_mode, BOOL is_block);
        errno_t change_mode();
        void Add_Signal();
        void write_SeriesPort_modbus(uint16_t angle_1, uint16_t angle_2, uint16_t angle_3, uint16_t angle_4, uint16_t angle_5, uint16_t angle_6, uint16_t Register_Number);
        void set_sim_mode(bool sim_mode);
#if defined(ARCH_ARM64)
        errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);
#else
        errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);
#endif
        errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode);
#if !defined(ARCH_ARM64)
        errno_t edg_servo_j(unsigned char robot_index, const JointValue *joint_pos, MoveMode move_mode);
        errno_t edg_send();
#endif
        errno_t servo_move_use_joint_LPF(double cutoffFreq);




    private:
        JAKAZuRobot jaka_robot_;
        bool sim_mode_ = false; 
        ModRtuComm hand;
        SignInfo sign_info_angles[8];
        JointValue joint_pos_current_sim_ = {0,0,0,0,0,0,0};
        CartesianPose cart_pos_current_sim_= {0,0,0,0,0,0};
};
#endif
