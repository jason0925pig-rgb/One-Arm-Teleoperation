#include "dh_vsmd.h"

DH_VSMD::DH_VSMD(int id, std::string Portname, int Baudrate):
    DH_Modbus_Gripper(id,Portname, Baudrate)
{

}

DH_VSMD::~DH_VSMD()
{

}


bool DH_VSMD::SetTargetRotation(int angle)
{
     return VsmdWriteRegisterFunc(0x0105,angle);
}

bool DH_VSMD::SetTargetRotationTorque(int torque)
{
    return VsmdWriteRegisterFunc(0x0108,torque);
}

bool DH_VSMD::SetTargetRotationSpeed(int speed)
{
    return VsmdWriteRegisterFunc(0x0107,speed);
}


bool DH_VSMD::GetCurrentRotation(int *curAngle)
{
    return VsmdReadRegisterFunc(0x0208,curAngle);
}

bool DH_VSMD::GetCUrrentTargetRotationTorque(int *curTarRotTorque)
{
    return VsmdReadRegisterFunc(0x0108,curTarRotTorque);
}

bool DH_VSMD::GetCurrentTargetRotationSpeed(int *curTarRotSpeed)
{
    return VsmdReadRegisterFunc(0x0107,curTarRotSpeed);
}

bool DH_VSMD::GetRotationState(int *r_state)
{
    return VsmdReadRegisterFunc(0x020B,r_state);
}

bool DH_VSMD::GetRotationInitializationState(int *ri_state)
{
    return VsmdReadRegisterFunc(0x020A,ri_state);
}
