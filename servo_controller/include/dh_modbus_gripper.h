#ifndef __dh_modbus_gripper__
#define __dh_modbus_gripper__

#include "dh_gripper.h"
#include "dh_device.h"
#include <iostream>

class DH_Modbus_Gripper : public DH_Gripper
{
public:

    DH_Modbus_Gripper(int id, std::string Portname, int Baudrate);
    ~DH_Modbus_Gripper();

    // connect to the gripper
    int open();
    // disconnect
    void close();

    // initialization the gripper 
    bool Initialization();
    
    // set gripper target position 0-1000
    bool SetTargetPosition(int refpos);
    // set gripper target force 20-100 %
    bool SetTargetForce(int force);
    // set gripper target speed 1-100 %
    bool SetTargetSpeed(int speed);

    // get gripper current position 
    bool GetCurrentPosition(int *curpos);
    // get gripper current target force (Notice: Not actual force)
    bool GetCurrentTargetForce(int *curTarforce);
    // get gripper current target speed (Notice: Not actual speed)
    bool GetCurrentTargetSpeed(int *curTarpos);

    // get gripper initialization state
    bool GetInitState(int *i_state);
    // get gripper grip state
    bool GetGripState(int *g_state);

    void GetSystemTime(QString *system_time);

    QString Get_All_Counter();

protected:
    //Modbus WriteRegisterFunc
    //para      :   index : register address ;
    //              value : write value
    //return    :   false : write failed ;
    //              true  : write successed
    bool WriteRegisterFunc(int index, int value);
    //Modbus ReadRegisterFunc
    //para      :   index : register address ;
    //              value : readed value
    //return    :   false : readed failed ;
    //              true  : readed successed
    bool ReadRegisterFunc(int index,int *value);
    //Modbus CRC16 
    unsigned short CRC16(const unsigned char *nData, unsigned short wLength);

    bool VsmdWriteRegisterFunc(int index, int value);
    bool VsmdReadRegisterFunc(int index, int *value);

    int _gripper_id;
    std::string _PortName;
    int _BaudRate;
    int _Serialhandle;

    dh_device *_m_device;

    long write_counter;
    long write_error_counter;
    long write_check_counter;

    long read_counter;
    long read_error_counter;
    long read_check_counter;

    long write_rev_counter;
    long read_rev_counter;


};

#endif //__dh_modbus_gripper__
