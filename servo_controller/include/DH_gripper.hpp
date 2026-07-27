#ifndef _DH_GRIPPER_HPP_
#define _DH_GRIPPER_HPP_

#include <string>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

class DH_Gripper
{
public:
    DH_Gripper(int id, std::string Portname, int Baudrate);
    DH_Gripper();
    ~DH_Gripper();

    // Connection management
    int open();
    void close();

    // Gripper control
    bool Initialization();
    bool SetTargetPosition(int refpos);
    bool SetTargetForce(int force);
    bool SetTargetSpeed(int speed);

    // Gripper status
    bool GetCurrentPosition(int &curpos);
    bool GetTargetPosition(int &tarpos);
    bool GetTargetForce(int &curTarforce);
    bool GetTargetSpeed(int &curTarpos);
    bool GetInitState(int &i_state);
    bool GetGripState(int &g_state);
    bool GetRunStates(int states[]);

    int GetGripperAxiNumber() { return gripper_axis; }

    enum S_INIT_STATES
    {
        S_INIT_NOT = 0,      // Need to be initialized
        S_INIT_FINISHED = 1, // Initialize finished
        S_INIT_DOING = 2,    // Initializing
    };

    enum S_GRIP_STATES
    {
        S_GRIP_MOVING = 0,  // gripper finger is moving
        S_GRIP_ARRIVED = 1, // gripper finger arrived target position
        S_GRIP_CAUGHT = 2,  // gripper caught a object
        S_GRIP_DROPPED = 3, // object dropped
    };

private:
    // Modbus communication
    bool WriteRegisterFunc(int index, int value);
    bool ReadRegisterFunc(int index, int &value);
    unsigned short CRC16(const unsigned char *nData, unsigned short wLength);

    // Device communication
    int serial_connect(std::string portname, int Baudrate);
    int tcp_connect(std::string ip_port);
    int connect_device(std::string portname, int parameter);
    void disconnect_device(int fd);
    int device_write(int fd, char *data, int len);
    int device_read(int fd, char *data, int data_len);
    int set_interface_attribs(int fd, int speed);
    void set_mincount(int fd, int mcount);

    // Member variables
    int gripper_axis;
    int _gripper_id;
    std::string _PortName;
    int _BaudRate;
    int _Serialhandle;
};

#endif //_DH_GRIPPER_HPP_