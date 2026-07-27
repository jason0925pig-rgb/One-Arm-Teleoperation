#ifndef DH_VSMD_H
#define DH_VSMD_H

#include "dh_modbus_gripper.h"
#include <iostream>

class DH_VSMD: public DH_Modbus_Gripper
{
public:
    DH_VSMD(int id, std::string Portname, int Baudrate);
    ~DH_VSMD();

    // set gripper target rotation angle -32767~32767 degree
    bool SetTargetRotation(int angle);
    // set gripper target rotation torque  20~100  %
    bool SetTargetRotationTorque(int torque);
    // set gripper target rotation speed  1~100 %
    bool SetTargetRotationSpeed(int speed);

    // get gripper current rotation angle
    bool GetCurrentRotation(int *curAngle);
    // get gripper target rotation torque (Notice: Not actual force)
    bool GetCUrrentTargetRotationTorque(int *curTarRotTorque);
    // get gripper target rotation speed (Notice: Not actual speed)
    bool GetCurrentTargetRotationSpeed(int *curTarRotSpeed);

    // get gripper rotation state
    bool GetRotationState(int *r_state);
    // get gripper rotation initialization state
    bool GetRotationInitializationState(int *ri_state);


    enum S_ROTATION_STATES
    {
        S_ROT_MOVING = 0,       // Rotating
        S_ROT_ARRIVED = 1,      // Arrived target angle
        S_ROT_BLOCKED = 2,      // Rotation is blocked
        S_ROT_HAD_BLOCKED = 3,  // Had been blocked
    };

};

#endif // DH_VSMD_H
