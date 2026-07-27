#ifndef HAND_MAIN_H
#define HAND_MAIN_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <sys/time.h>
#include <sys/select.h>
#include <time.h>

#include <iostream>
#include <thread>
#include <chrono>
#include "timespec.h"
#include <iomanip>


#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "DH_gripper.hpp"
#include "ZX_gripper.h"

//#include "nlohmann/json.hpp"

#define rad2deg(x) ((x) * 180.0 / M_PI)
#define deg2rad(x) ((x) * M_PI / 180.0)

#define kSend_Frame_Head1				0xEB
#define kSend_Frame_Head2				0x90


#define kRcv_Frame_Head1				0x90
#define kRcv_Frame_Head2				0xEB

//Direct control flag
#define kCmd_Handg3_Read  			       0x11  //read register
#define kCmd_Handg3_Write  			       0x12	 //write register
#define kCmd_Mc_Angle_Force                          0x14  //motor motion enable
#define kCmd_Mc_Force                                0x15  //motor motion disable
#define kCmd_Mc_Current                              0x16  //save parameters to flash
#define kCmd_Mc_All                                  0x17  //get drive status

//Number of data
#define kUart_Recv_Length                            256   //data length

#define rad2deg(x) ((x) * 180.0 / M_PI)
#define deg2rad(x) ((x) * M_PI / 180.0)

#define NUM_HAND_CONTROL 6

#define PrintInfo(format, ...)                                               \
    if (bPrint2Buffer)                                                       \
    {                                                                        \
        logQuit();                                                           \
        logOffset += snprintf(logBuffer + logOffset, nBufferLen - logOffset, \
                              format, ##__VA_ARGS__);                        \
    }                                                                        \
    else                                                                     \
        printf(format, ##__VA_ARGS__);

//using json = nlohmann::json;

struct Figure
{
      int16_t       m_current_angle;                   //手指当前角度
      int16_t       m_current_current;                 //手指当前电流值
      int16_t       m_current_forceact;                //手指实际受力值
      int8_t        m_error_code;                      //手指故障码
};

typedef struct
{
     struct Figure  tLittleFinger;
     struct Figure  tRingFinger;
     struct Figure  tMiddleFinger;
     struct Figure  tIndexFinger;
     struct Figure  tThumbBend;
     struct Figure  tThumbSide;
}ActuatorStatusTypeDef;

typedef struct _Uart_Data_Buff{
    uint8_t m_tx_len;
    uint8_t m_rec_array[128];     //Define receive data array
    uint8_t m_send_array[128];     //Define send data array
}UartDataBuff;


class HandControl
{

public:
    HandControl();
    ~HandControl();



public:

    ActuatorStatusTypeDef tHand;
    UartDataBuff tUartData = {0};

    volatile int fd_left, fd_right, fd_gripper, nByte;
    volatile int fd_hand;
    volatile uint8_t   sum,g_unHand_id = 0x01;                   //Hand ID
    volatile uint16_t  g_unNum = 50;                      //Define data length    
    double finger_value_act_[6];

    DH_Gripper *m_gripper;
    ZX_gripper *m_gripper_zx;


    bool init(std::string device_name);
    bool init_dh_gripper(std::string device_name);
    void exec_dh_gripper(double *finger_value_arr_, uint8_t hand_id, int fd);

    bool init_zx_gripper(std::string device_name);
    void exec_zx_gripper(double *finger_value_arr_, uint8_t hand_id, int fd);

    void setID();

    void setOpenLimit();

    void setClearError();

    void setMoveTGT(int n);

    void setMoveMax();

    void setMoveMin();

    void setMoveMinHold();

    void setStop();

    void setParam();

    double deg2Rad(double deg) {
        return deg * M_PI / 180.0;
    }

    void exec_inspire_hand(double *finger_value_arr_, uint8_t hand_id, int fd);
    void exec_inspire_gripper(double *finger_value_arr_, uint8_t hand_id, int fd);

    bool read_inspire_hand(uint8_t hand_id,int fd);
    bool read_inspire_hand_all(uint8_t hand_id,int fd);

    void print_hand_data();
    void printTimestamp(uint64_t timestamp);
    void printTimestampEx(uint64_t timestamp, const char *type, int32_t count);

    //json readJsonFile(const std::string &filename);

    void* ThreadEntry();

    /*********************************************************************************************************
    ** Function name:       void setTimer(int seconds, int mseconds, int com)
    ** Descriptions:        Timing function
    ** input parameters:    seconds: the seconds; mseconds: the micro seconds,com:the com
    ** output parameters:   none
    ** Returned value:      None
    *********************************************************************************************************/
    void setTimer(int seconds, int mseconds, int com);
    /*********************************************************************************************************
    ** Function name:       int set_opt(int ,int , int , char , int )
    ** Descriptions:        Serial port configuration function
    ** input parameters:    com,speed,bit,even,stop
    ** output parameters:   none
    ** Returned value:      None
    *********************************************************************************************************/
    int set_opt(int ,int , int , char , int );
    /*********************************************************************************************************
    ** Function name:       int Read_Hand_Data(uint8_t ID)
    ** Descriptions:        Read hand data
    ** input parameters:    Input ID of the hand
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Read_Hand_Data(u_int8_t ID);
    /*********************************************************************************************************
    ** Function name:       int Save_Hand_Data(uint8_t ID)
    ** Descriptions:        Save hand data
    ** input parameters:    Input ID of the hand
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Save_Hand_Data(u_int8_t ID);
    /*********************************************************************************************************
    ** Function name:       int Read_Hand_Register_Data(uint8_t ID,uint16_t val,uint16_t vallen)
    ** Descriptions:        Read hand register data
    ** input parameters:    ID:Input ID of the drive
                            val:Input starting register value
                            vallen:Input read register length
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Read_Hand_Register_Data(uint8_t ID,uint16_t val,uint16_t vallen);
    /*********************************************************************************************************
    ** Function name:       int Change_Hand_ID(uint8_t ID,uint8_t val)
    ** Descriptions:        Change hand ID
    ** input parameters:    ID:Input ID of the drive
                            val:Input another ID
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Change_Hand_ID(uint8_t ID,uint8_t val);
    /*********************************************************************************************************
    ** Function name:       int Change_Hand_Baud(uint8_t ID,uint8_t baud)
    ** Descriptions:        Change baud of the hand
    ** input parameters:    ID:ID of the current hand
                            baud:Representative number of baud rate,0--19200 1--57600 2--115200
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Change_Hand_Baud(uint8_t ID,uint8_t baud);
    /*********************************************************************************************************
    ** Function name:       int Clear_Hand_Error(uint8_t ID)
    ** Descriptions:        Clear drive error
    ** input parameters:    Input ID of the drive
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Clear_Hand_Error(uint8_t ID);
    /*********************************************************************************************************
    ** Function name:       int Reset_Hand(uint8_t ID)
    ** Descriptions:        Reset drive
    ** input parameters:    Input ID of the drive
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Reset_Hand(uint8_t ID);
    /*********************************************************************************************************
    ** Function name:       int Force_Sensor_Calibration(uint8_t ID)
    ** Descriptions:        Calibrate the force sensor
    ** input parameters:    Input ID of the drive
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Force_Sensor_Calibration(uint8_t ID);
    /*********************************************************************************************************
    ** Function name:       int Write_Hand_Angle(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
    ** Descriptions:        Set the angle of each finger
    ** input parameters:    ID:Input ID of the drive
                            val1:Little figure angle setting value
                            val2:Ring finger angle setting value
                            val3:Middle finger angle setting value
                            val4:Index finger angle setting value
                            val5:Thumb angle setting value
                            val6:Thumb swing angle setting value
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Write_Hand_Angle(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6);
    /*********************************************************************************************************
    ** Function name:       int Write_Hand_Drive_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
    ** Descriptions:        Through register set the position of each finger
    ** input parameters:    ID:Input ID of the drive
                            val1:Little figure position setting value
                            val2:Ring finger position setting value
                            val3:Middle finger position setting value
                            val4:Index finger position setting value
                            val5:Thumb angle position value
                            val6:Thumb swing position setting value
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Write_Hand_Drive_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6);
    /*********************************************************************************************************
    ** Function name:       int Write_Hand_Angle_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
    ** Descriptions:        Through register set the Angle of each finger
    ** input parameters:    ID:Input ID of the drive
                            val1:Little figure angle setting value
                            val2:Ring finger angle setting value
                            val3:Middle finger angle setting value
                            val4:Index finger angle setting value
                            val5:Thumb angle setting value
                            val6:Thumb swing angle setting value
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Write_Hand_Angle_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6);
    /*********************************************************************************************************
    ** Function name:       int Write_Hand_Force_Threshold(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
    ** Descriptions:        Through register set the force threshole of each finger
    ** input parameters:    ID:Input ID of the drive
                            val1:Little figure force threshole setting value
                            val2:Ring finger force threshole setting value
                            val3:Middle finger force threshole setting value
                            val4:Index finger force threshole setting value
                            val5:Thumb force threshole setting value
                            val6:Thumb swing force threshole setting value
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Write_Hand_Force_Threshold(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6);
    /*********************************************************************************************************
    ** Function name:       int Write_Hand_Speed(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
    ** Descriptions:        Through register set the speed of each finger
    ** input parameters:    Input ID of the drive
                            val1:Little figure speed setting value
                            val2:Ring finger speed setting value
                            val3:Middle finger speed setting value
                            val4:Index finger speed setting value
                            val5:Thumb speed setting value
                            val6:Thumb swing speed setting value
    ** output parameters:   Outout command array
    ** Returned value:      None
    *********************************************************************************************************/
    int Write_Hand_Speed(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6);
    /*********************************************************************************************************
    ** Function name:       int Uart_Rece_Data_Parsing()
    ** Descriptions:        Parse hand data
    ** input parameters:    None
    ** output parameters:   Hand data
    ** Returned value:      None
    *********************************************************************************************************/
    int Uart_Rece_Data_Parsing();
};
#endif // HAND_MAIN_H
