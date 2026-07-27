#include "hand_control.h"


HandControl::HandControl()
{

}


bool HandControl::init(std::string device_name) {
    //if ((fd_right = open(device_name.c_str(), O_RDWR | O_NOCTTY)) < 0)
    //{
    //    std::cout<<"open usb serial failed!"<<std::endl;
    //}
    //else
    //{
    //    set_opt(fd_right, 115200, 8, 'N', 1);
    //}
//
    //if ((fd_left = open(device_name.c_str(), O_RDWR | O_NOCTTY)) < 0)
    //{
    //    std::cout<<"open usb serial failed!"<<std::endl;
    //}
    //else
    //{
    //    set_opt(fd_left, 115200, 8, 'N', 1);
    //}

    if ((fd_hand = open(device_name.c_str(), O_RDWR | O_NOCTTY)) < 0)
    {
        std::cout<<"open usb serial failed!"<<std::endl;
    }
    else
    {
        set_opt(fd_hand, 115200, 8, 'N', 1);
    }


    setClearError();
    setID();
    setOpenLimit();

    return true;
}




void HandControl::setID() {
    uint8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x02;
    send_buffer[4] = 0x04;
    send_buffer[5] = 0x01;

    int len = send_buffer[3] + 5;
    for (int i = 2; i < len - 1; ++i) {
        check_sum += send_buffer[i];
    }
    send_buffer[6] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,7);
}

void HandControl::setOpenLimit() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x05;
    send_buffer[4] = 0x12;

    unsigned int temp_int1,temp_int2;
    temp_int1 = 1000;
    temp_int2 = 0;

    send_buffer[5] = (temp_int1 & 0xFF);
    send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
    send_buffer[7] = (temp_int2 & 0xFF);
    send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

    int len = send_buffer[3]+5;
    for(int i = 2; i < len - 1; ++i) {
        check_sum += send_buffer[i];
    }
    send_buffer[9] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,10);
}

void HandControl::setClearError() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x01;
    send_buffer[4] = 0x17;

    int len = send_buffer[3]+5;
    for(int i = 2; i < len - 1; ++i) {
        check_sum += send_buffer[i];
    }
    send_buffer[5] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,6);
}

void HandControl::setMoveTGT(int n) {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x03;
    send_buffer[4] = 0x54;

    unsigned int temp_int1;
    temp_int1 = n;

    send_buffer[5] = (temp_int1 & 0xFF);
    send_buffer[6] = ((temp_int1 >> 8) & 0xFF);

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[7] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,8);
}

void HandControl::setMoveMax() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x03;
    send_buffer[4] = 0x11;

    unsigned int temp_int1;
    temp_int1 = 1000;

    send_buffer[5] = (temp_int1 & 0xFF);
    send_buffer[6] = ((temp_int1 >> 8) & 0xFF);

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[7] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,8);
}
                                                                                                                                                                                                                                                             
void HandControl::setMoveMin() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x05;
    send_buffer[4] = 0x10;

    unsigned int temp_int1,temp_int2;
    temp_int1 = 1000;
    temp_int2 = 1000;

    send_buffer[5] = (temp_int1 & 0xFF);
    send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
    send_buffer[7] = (temp_int2 & 0xFF);
    send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[9] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,10);
}

void HandControl::setMoveMinHold() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x05;
    send_buffer[4] = 0x18;

    unsigned int temp_int1,temp_int2;
    temp_int1 = 1000;
    temp_int2 = 1000;

    send_buffer[5] = (temp_int1 & 0xFF);
    send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
    send_buffer[7] = (temp_int2 & 0xFF);
    send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[9] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,10);
}

void HandControl::setStop() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x01;
    send_buffer[4] = 0x16;

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[5] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,11);
}

void HandControl::setParam() {
    u_int8_t check_sum = 0;
    unsigned char send_buffer[64] = {0};
    send_buffer[0] = 0xEB;
    send_buffer[1] = 0x90;
    send_buffer[2] = 0x01;
    send_buffer[3] = 0x01;
    send_buffer[4] = 0x01;

    int len = send_buffer[3]+5;
    for(int i = 2;i < len - 1;++i)
    {
        check_sum += send_buffer[i];
    }
    send_buffer[5] = (check_sum & 0xFF);
    write(fd_hand,send_buffer,6);
}

/*seconds: the seconds; mseconds: the micro seconds*/
void HandControl::setTimer(int seconds, int mseconds, int com)
{
    struct timeval temp;

    temp.tv_sec = seconds;
    temp.tv_usec = mseconds * 1000;

    select(0, NULL, NULL, NULL, &temp);

    read(com, tUartData.m_rec_array, sizeof(*tUartData.m_rec_array));
    printf("%x\n", *tUartData.m_rec_array);
    memset(tUartData.m_rec_array, 0, sizeof(*tUartData.m_rec_array));

    return;
}
int HandControl::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }

    printf("串口设置完成!\n\r");
    return 0;
}
int HandControl::Read_Hand_Data(u_int8_t ID)
{
    u_int8_t i = 0;
    u_int8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0D;
    tUartData.m_send_array[4] = kCmd_Mc_All;
    for (i = 5; i < 17; i++)
    {
        tUartData.m_send_array[i] = 0xFF;
    }
    for (i = 2; i < 17; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[17] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 18;
    return 0;
}
int HandControl::Read_Hand_Register_Data(uint8_t ID, uint16_t val, uint16_t vallen)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Read;
    tUartData.m_send_array[5] = val & 0xFF;
    tUartData.m_send_array[6] = val >> 8;
    tUartData.m_send_array[7] = vallen;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Save_Hand_Data(uint8_t ID)
** Descriptions:        Save hand data
** input parameters:    Input ID of the hand
** output parameters:   Outout command array
** Returned value:      None
*********************************************************************************************************/

int HandControl::Save_Hand_Data(uint8_t ID)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xED;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = 0x01;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Change_Hand_ID(uint8_t ID,uint8_t val)
** Descriptions:        Change hand ID
** input parameters:    ID:Input ID of the drive
                        val:Input another ID
** output parameters:   Outout command array
** Returned value:      None
*********************************************************************************************************/

int HandControl::Change_Hand_ID(uint8_t ID, uint8_t val)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xE8;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = val;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Change_Hand_Baud(uint8_t ID,uint8_t baud)
** Descriptions:        Change baud of the hand
** input parameters:    ID:ID of the current hand
                        baud:Representative number of baud rate,0--19200 1--57600 2--115200
** output parameters:   Outout command array
** Returned value:      None
*********************************************************************************************************/

int HandControl::Change_Hand_Baud(uint8_t ID, uint8_t baud)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xE9;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = baud;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Clear_Hand_Error(uint8_t ID)
** Descriptions:        Clear drive error
** input parameters:    Input ID of the drive
** output parameters:   Outout command array
** Returned value:      None
*********************************************************************************************************/

int HandControl::Clear_Hand_Error(uint8_t ID)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xEC;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = 0x01;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Reset_Hand(uint8_t ID)
** Descriptions:        Reset drive
** input parameters:    Input ID of the drive
** output parameters:   Outout command array
** Returned value:      No
*********************************************************************************************************/

int HandControl::Reset_Hand(uint8_t ID)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xEE;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = 0x01;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Force_Sensor_Calibration(uint8_t ID)
** Descriptions:        Calibrate the force sensor
** input parameters:    Input ID of the drive
** output parameters:   Outout command array
** Returned value:      None
*********************************************************************************************************/

int HandControl::Force_Sensor_Calibration(uint8_t ID)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xF1;
    tUartData.m_send_array[6] = 0x03;
    tUartData.m_send_array[7] = 0x01;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 9;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Write_Hand_Angle(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
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

int HandControl::Write_Hand_Angle(uint8_t ID, uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4, uint16_t val5, uint16_t val6)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0D;
    tUartData.m_send_array[4] = kCmd_Mc_All;
    tUartData.m_send_array[5] = val1 & 0xFF;
    tUartData.m_send_array[6] = val1 >> 8;
    tUartData.m_send_array[7] = val2 & 0xFF;
    tUartData.m_send_array[8] = val2 >> 8;
    tUartData.m_send_array[9] = val3 & 0xFF;
    tUartData.m_send_array[10] = val3 >> 8;
    tUartData.m_send_array[11] = val4 & 0xFF;
    tUartData.m_send_array[12] = val4 >> 8;
    tUartData.m_send_array[13] = val5 & 0xFF;
    tUartData.m_send_array[14] = val5 >> 8;
    tUartData.m_send_array[15] = val6 & 0xFF;
    tUartData.m_send_array[16] = val6 >> 8;
    for (i = 2; i < 17; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[17] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 18;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Write_Hand_Drive_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
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

int HandControl::Write_Hand_Drive_Position(uint8_t ID, uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4, uint16_t val5, uint16_t val6)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0F;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xC2;
    tUartData.m_send_array[6] = 0x05;
    tUartData.m_send_array[7] = val1 & 0xFF;
    tUartData.m_send_array[8] = val1 >> 8;
    tUartData.m_send_array[9] = val2 & 0xFF;
    tUartData.m_send_array[10] = val2 >> 8;
    tUartData.m_send_array[11] = val3 & 0xFF;
    tUartData.m_send_array[12] = val3 >> 8;
    tUartData.m_send_array[13] = val4 & 0xFF;
    tUartData.m_send_array[14] = val4 >> 8;
    tUartData.m_send_array[15] = val5 & 0xFF;
    tUartData.m_send_array[16] = val5 >> 8;
    tUartData.m_send_array[17] = val6 & 0xFF;
    tUartData.m_send_array[18] = val6 >> 8;
    for (i = 2; i < 19; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[19] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 20;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Write_Hand_Angle_Position(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
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

int HandControl::Write_Hand_Angle_Position(uint8_t ID, uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4, uint16_t val5, uint16_t val6)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0F;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xCE;
    tUartData.m_send_array[6] = 0x05;
    tUartData.m_send_array[7] = val1 & 0xFF;
    tUartData.m_send_array[8] = val1 >> 8;
    tUartData.m_send_array[9] = val2 & 0xFF;
    tUartData.m_send_array[10] = val2 >> 8;
    tUartData.m_send_array[11] = val3 & 0xFF;
    tUartData.m_send_array[12] = val3 >> 8;
    tUartData.m_send_array[13] = val4 & 0xFF;
    tUartData.m_send_array[14] = val4 >> 8;
    tUartData.m_send_array[15] = val5 & 0xFF;
    tUartData.m_send_array[16] = val5 >> 8;
    tUartData.m_send_array[17] = val6 & 0xFF;
    tUartData.m_send_array[18] = val6 >> 8;
    for (i = 2; i < 19; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[19] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 20;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Write_Hand_Force_Threshold(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
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

int HandControl::Write_Hand_Force_Threshold(uint8_t ID, uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4, uint16_t val5, uint16_t val6)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0F;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xDA;
    tUartData.m_send_array[6] = 0x05;
    tUartData.m_send_array[7] = val1 & 0xFF;
    tUartData.m_send_array[8] = val1 >> 8;
    tUartData.m_send_array[9] = val2 & 0xFF;
    tUartData.m_send_array[10] = val2 >> 8;
    tUartData.m_send_array[11] = val3 & 0xFF;
    tUartData.m_send_array[12] = val3 >> 8;
    tUartData.m_send_array[13] = val4 & 0xFF;
    tUartData.m_send_array[14] = val4 >> 8;
    tUartData.m_send_array[15] = val5 & 0xFF;
    tUartData.m_send_array[16] = val5 >> 8;
    tUartData.m_send_array[17] = val6 & 0xFF;
    tUartData.m_send_array[18] = val6 >> 8;
    for (i = 2; i < 19; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[19] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 20;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Write_Hand_Speed(uint8_t ID,uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4,uint16_t val5,uint16_t val6)
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

int HandControl::Write_Hand_Speed(uint8_t ID, uint16_t val1, uint16_t val2, uint16_t val3, uint16_t val4, uint16_t val5, uint16_t val6)
{
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = ID;
    tUartData.m_send_array[3] = 0x0F;
    tUartData.m_send_array[4] = kCmd_Handg3_Write;
    tUartData.m_send_array[5] = 0xF2;
    tUartData.m_send_array[6] = 0x05;
    tUartData.m_send_array[7] = val1 & 0xFF;
    tUartData.m_send_array[8] = val1 >> 8;
    tUartData.m_send_array[9] = val2 & 0xFF;
    tUartData.m_send_array[10] = val2 >> 8;
    tUartData.m_send_array[11] = val3 & 0xFF;
    tUartData.m_send_array[12] = val3 >> 8;
    tUartData.m_send_array[13] = val4 & 0xFF;
    tUartData.m_send_array[14] = val4 >> 8;
    tUartData.m_send_array[15] = val5 & 0xFF;
    tUartData.m_send_array[16] = val5 >> 8;
    tUartData.m_send_array[17] = val6 & 0xFF;
    tUartData.m_send_array[18] = val6 >> 8;
    for (i = 2; i < 19; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[19] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 20;
    return 0;
}

/*********************************************************************************************************
** Function name:       void Uart_Rece_Data_Parsing()
** Descriptions:        Parse hand data
** input parameters:    None
** output parameters:   Hand data
** Returned value:      None
*********************************************************************************************************/

int HandControl::Uart_Rece_Data_Parsing()
{
    uint8_t i = 0, j = 0;
    uint8_t m_unChecksum = 0;
    uint8_t m_unRxdata_usart = 0;
    uint8_t m_unRx_cnt_uart = 0;
    uint8_t m_unRx_data_uartcom = 0;
    uint16_t m_unUart_rx_data_lens = 0;
    for (j = 0; j < g_unNum; j++)
    {
        m_unRxdata_usart = tUartData.m_rec_array[j];
        m_unRx_data_uartcom = m_unRxdata_usart;
        if (m_unRx_cnt_uart == 0)
        {
            if (m_unRx_data_uartcom == kRcv_Frame_Head1)
            {
                tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
                m_unRx_cnt_uart++;
            }
        }
        else if (m_unRx_cnt_uart == 1)
        {
            if (m_unRx_data_uartcom == kRcv_Frame_Head2)
            {
                tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
                m_unRx_cnt_uart++;
            }
            else if (m_unRx_data_uartcom == kRcv_Frame_Head1)
            {
                m_unRx_cnt_uart = 0;
                tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
                m_unRx_cnt_uart++;
            }
        }
        else if (m_unRx_cnt_uart == 2)
        {
            tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
            m_unRx_cnt_uart++;
        }
        else if (m_unRx_cnt_uart == 3)
        {
            tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
            m_unUart_rx_data_lens = m_unRx_data_uartcom;
            m_unRx_cnt_uart++;
        }
        else if (m_unRx_cnt_uart == (m_unUart_rx_data_lens + 4))
        {
            tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
            m_unRx_cnt_uart = 0;
            for (i = 2; i < (m_unUart_rx_data_lens + 4); i++)
            {
                m_unChecksum += tUartData.m_rec_array[i];
            }
            if (m_unChecksum == tUartData.m_rec_array[m_unUart_rx_data_lens + 4])
            {
                g_unHand_id = tUartData.m_rec_array[2];
                if (tUartData.m_rec_array[4] == kCmd_Handg3_Read)
                {
                    tHand.tLittleFinger.m_current_angle = (tUartData.m_rec_array[7] & 0xFF) + (tUartData.m_rec_array[8] << 8);
                    tHand.tRingFinger.m_current_angle = (tUartData.m_rec_array[9] & 0xFF) + (tUartData.m_rec_array[10] << 8);
                    tHand.tMiddleFinger.m_current_angle = (tUartData.m_rec_array[11] & 0xFF) + (tUartData.m_rec_array[12] << 8);
                    tHand.tIndexFinger.m_current_angle = (tUartData.m_rec_array[13] & 0xFF) + (tUartData.m_rec_array[14] << 8);
                    tHand.tThumbBend.m_current_angle = (tUartData.m_rec_array[15] & 0xFF) + (tUartData.m_rec_array[16] << 8);
                    tHand.tThumbSide.m_current_angle = (tUartData.m_rec_array[17] & 0xFF) + (tUartData.m_rec_array[18] << 8);
                    tHand.tLittleFinger.m_current_current = (tUartData.m_rec_array[55] & 0xFF) + (tUartData.m_rec_array[56] << 8);
                    tHand.tRingFinger.m_current_current = (tUartData.m_rec_array[57] & 0xFF) + (tUartData.m_rec_array[58] << 8);
                    tHand.tMiddleFinger.m_current_current = (tUartData.m_rec_array[59] & 0xFF) + (tUartData.m_rec_array[60] << 8);
                    tHand.tIndexFinger.m_current_current = (tUartData.m_rec_array[61] & 0xFF) + (tUartData.m_rec_array[62] << 8);
                    tHand.tThumbBend.m_current_current = (tUartData.m_rec_array[63] & 0xFF) + (tUartData.m_rec_array[64] << 8);
                    tHand.tThumbSide.m_current_current = (tUartData.m_rec_array[65] & 0xFF) + (tUartData.m_rec_array[66] << 8);
                    tHand.tLittleFinger.m_current_forceact = (tUartData.m_rec_array[43] & 0xFF) + (tUartData.m_rec_array[44] << 8);
                    tHand.tRingFinger.m_current_forceact = (tUartData.m_rec_array[45] & 0xFF) + (tUartData.m_rec_array[46] << 8);
                    tHand.tMiddleFinger.m_current_forceact = (tUartData.m_rec_array[47] & 0xFF) + (tUartData.m_rec_array[48] << 8);
                    tHand.tIndexFinger.m_current_forceact = (tUartData.m_rec_array[49] & 0xFF) + (tUartData.m_rec_array[50] << 8);
                    tHand.tThumbBend.m_current_forceact = (tUartData.m_rec_array[51] & 0xFF) + (tUartData.m_rec_array[52] << 8);
                    tHand.tThumbSide.m_current_forceact = (tUartData.m_rec_array[53] & 0xFF) + (tUartData.m_rec_array[54] << 8);
                    tHand.tLittleFinger.m_error_code = (tUartData.m_rec_array[67] & 0xFF);
                    tHand.tRingFinger.m_error_code = (tUartData.m_rec_array[68] & 0xFF);
                    tHand.tMiddleFinger.m_error_code = (tUartData.m_rec_array[69] & 0xFF);
                    tHand.tIndexFinger.m_error_code = (tUartData.m_rec_array[70] & 0xFF);
                    tHand.tThumbBend.m_error_code = (tUartData.m_rec_array[71] & 0xFF);
                    tHand.tThumbSide.m_error_code = (tUartData.m_rec_array[72] & 0xFF);
                }
                else if (tUartData.m_rec_array[4] == kCmd_Mc_Angle_Force)
                {
                    tHand.tLittleFinger.m_current_angle = (tUartData.m_rec_array[5] & 0xFF) + (tUartData.m_rec_array[6] << 8);
                    tHand.tRingFinger.m_current_angle = (tUartData.m_rec_array[9] & 0xFF) + (tUartData.m_rec_array[10] << 8);
                    tHand.tMiddleFinger.m_current_angle = (tUartData.m_rec_array[13] & 0xFF) + (tUartData.m_rec_array[14] << 8);
                    tHand.tIndexFinger.m_current_angle = (tUartData.m_rec_array[17] & 0xFF) + (tUartData.m_rec_array[18] << 8);
                    tHand.tThumbBend.m_current_angle = (tUartData.m_rec_array[21] & 0xFF) + (tUartData.m_rec_array[22] << 8);
                    tHand.tThumbSide.m_current_angle = (tUartData.m_rec_array[25] & 0xFF) + (tUartData.m_rec_array[26] << 8);
                    tHand.tLittleFinger.m_current_forceact = (tUartData.m_rec_array[7] & 0xFF) + (tUartData.m_rec_array[8] << 8);
                    tHand.tRingFinger.m_current_forceact = (tUartData.m_rec_array[11] & 0xFF) + (tUartData.m_rec_array[12] << 8);
                    tHand.tMiddleFinger.m_current_forceact = (tUartData.m_rec_array[15] & 0xFF) + (tUartData.m_rec_array[16] << 8);
                    tHand.tIndexFinger.m_current_forceact = (tUartData.m_rec_array[19] & 0xFF) + (tUartData.m_rec_array[20] << 8);
                    tHand.tThumbBend.m_current_forceact = (tUartData.m_rec_array[23] & 0xFF) + (tUartData.m_rec_array[24] << 8);
                    tHand.tThumbSide.m_current_forceact = (tUartData.m_rec_array[27] & 0xFF) + (tUartData.m_rec_array[28] << 8);
                }
                else if (tUartData.m_rec_array[4] == kCmd_Mc_Force)
                {

                    tHand.tLittleFinger.m_current_forceact = (tUartData.m_rec_array[7] & 0xFF) + (tUartData.m_rec_array[8] << 8);
                    tHand.tRingFinger.m_current_forceact = (tUartData.m_rec_array[9] & 0xFF) + (tUartData.m_rec_array[10] << 8);
                    tHand.tMiddleFinger.m_current_forceact = (tUartData.m_rec_array[11] & 0xFF) + (tUartData.m_rec_array[12] << 8);
                    tHand.tIndexFinger.m_current_forceact = (tUartData.m_rec_array[13] & 0xFF) + (tUartData.m_rec_array[14] << 8);
                    tHand.tThumbBend.m_current_forceact = (tUartData.m_rec_array[15] & 0xFF) + (tUartData.m_rec_array[16] << 8);
                    tHand.tThumbSide.m_current_forceact = (tUartData.m_rec_array[17] & 0xFF) + (tUartData.m_rec_array[18] << 8);
                }
                else if (tUartData.m_rec_array[4] == kCmd_Mc_Current)
                {
                    tHand.tLittleFinger.m_current_current = (tUartData.m_rec_array[7] & 0xFF) + (tUartData.m_rec_array[8] << 8);
                    tHand.tRingFinger.m_current_current = (tUartData.m_rec_array[9] & 0xFF) + (tUartData.m_rec_array[10] << 8);
                    tHand.tMiddleFinger.m_current_current = (tUartData.m_rec_array[11] & 0xFF) + (tUartData.m_rec_array[12] << 8);
                    tHand.tIndexFinger.m_current_current = (tUartData.m_rec_array[13] & 0xFF) + (tUartData.m_rec_array[14] << 8);
                    tHand.tThumbBend.m_current_current = (tUartData.m_rec_array[15] & 0xFF) + (tUartData.m_rec_array[16] << 8);
                    tHand.tThumbSide.m_current_current = (tUartData.m_rec_array[17] & 0xFF) + (tUartData.m_rec_array[18] << 8);
                }
                else if (tUartData.m_rec_array[4] == kCmd_Mc_All)
                {
                    tHand.tLittleFinger.m_current_angle = (tUartData.m_rec_array[5] & 0xFF) + (tUartData.m_rec_array[6] << 8);
                    tHand.tRingFinger.m_current_angle = (tUartData.m_rec_array[12] & 0xFF) + (tUartData.m_rec_array[13] << 8);
                    tHand.tMiddleFinger.m_current_angle = (tUartData.m_rec_array[19] & 0xFF) + (tUartData.m_rec_array[20] << 8);
                    tHand.tIndexFinger.m_current_angle = (tUartData.m_rec_array[26] & 0xFF) + (tUartData.m_rec_array[27] << 8);
                    tHand.tThumbBend.m_current_angle = (tUartData.m_rec_array[33] & 0xFF) + (tUartData.m_rec_array[34] << 8);
                    tHand.tThumbSide.m_current_angle = (tUartData.m_rec_array[40] & 0xFF) + (tUartData.m_rec_array[41] << 8);
                    tHand.tLittleFinger.m_current_current = (tUartData.m_rec_array[7] & 0xFF) + (tUartData.m_rec_array[8] << 8);
                    tHand.tRingFinger.m_current_current = (tUartData.m_rec_array[14] & 0xFF) + (tUartData.m_rec_array[15] << 8);
                    tHand.tMiddleFinger.m_current_current = (tUartData.m_rec_array[21] & 0xFF) + (tUartData.m_rec_array[22] << 8);
                    tHand.tIndexFinger.m_current_current = (tUartData.m_rec_array[28] & 0xFF) + (tUartData.m_rec_array[29] << 8);
                    tHand.tThumbBend.m_current_current = (tUartData.m_rec_array[35] & 0xFF) + (tUartData.m_rec_array[36] << 8);
                    tHand.tThumbSide.m_current_current = (tUartData.m_rec_array[42] & 0xFF) + (tUartData.m_rec_array[43] << 8);
                    tHand.tLittleFinger.m_current_forceact = (tUartData.m_rec_array[9] & 0xFF) + (tUartData.m_rec_array[10] << 8);
                    tHand.tRingFinger.m_current_forceact = (tUartData.m_rec_array[16] & 0xFF) + (tUartData.m_rec_array[17] << 8);
                    tHand.tMiddleFinger.m_current_forceact = (tUartData.m_rec_array[23] & 0xFF) + (tUartData.m_rec_array[24] << 8);
                    tHand.tIndexFinger.m_current_forceact = (tUartData.m_rec_array[30] & 0xFF) + (tUartData.m_rec_array[31] << 8);
                    tHand.tThumbBend.m_current_forceact = (tUartData.m_rec_array[37] & 0xFF) + (tUartData.m_rec_array[38] << 8);
                    tHand.tThumbSide.m_current_forceact = (tUartData.m_rec_array[44] & 0xFF) + (tUartData.m_rec_array[45] << 8);
                    tHand.tLittleFinger.m_error_code = (tUartData.m_rec_array[11] & 0xFF);
                    tHand.tRingFinger.m_error_code = (tUartData.m_rec_array[18] & 0xFF);
                    tHand.tMiddleFinger.m_error_code = (tUartData.m_rec_array[25] & 0xFF);
                    tHand.tIndexFinger.m_error_code = (tUartData.m_rec_array[32] & 0xFF);
                    tHand.tThumbBend.m_error_code = (tUartData.m_rec_array[39] & 0xFF);
                    tHand.tThumbSide.m_error_code = (tUartData.m_rec_array[46] & 0xFF);
                }
            }
        }
        else
        {
            tUartData.m_rec_array[m_unRx_cnt_uart] = m_unRx_data_uartcom;
            printf("%x\n", tUartData.m_rec_array[m_unRx_cnt_uart]);
            m_unRx_cnt_uart++;
        }
    }
    return 0;
}

HandControl::~HandControl()
{
    //::close(fd_left);
    //::close(fd_right);
    ::close(fd_hand);
}

void HandControl::exec_inspire_hand(double *finger_value_arr_, uint8_t hand_id, int fd)
{
    // rad
    // 四指关节角: 0 ~ 80, 
    // 大拇指弯曲关节角: 0 ~ 40
    // 大拇指侧摆关节角 0 ~ 80

    int val_four_finger_1;
    int val_four_finger_2;
    int val_four_finger_3;
    int val_four_finger_4;

    val_four_finger_1 = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 1.7) * finger_value_arr_[5])));
    val_four_finger_2 = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 1.7) * finger_value_arr_[4])));
    val_four_finger_3 = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 1.7) * finger_value_arr_[3])));
    val_four_finger_4 = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 1.7) * finger_value_arr_[2])));

    int bend_val_big_finger;
    int yaw_val_big_finger;

    bend_val_big_finger = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 0.55) * finger_value_arr_[1])));
    yaw_val_big_finger = 1000 - std::max(0, std::min(1000, static_cast<int>((1000 / 1.2) * finger_value_arr_[0])));
    //std::cout << "val_four_finger_1: " << val_four_finger_1 << std::endl;
    Write_Hand_Angle_Position(hand_id, val_four_finger_1, val_four_finger_2, val_four_finger_3, val_four_finger_4, bend_val_big_finger, yaw_val_big_finger);
    write(fd, tUartData.m_send_array, tUartData.m_tx_len);
    Read_Hand_Data(hand_id);


    //    qDebug("DataValue: ");
    //    qDebug("%d %d %d %d %d %d", val_four_finger_1, val_four_finger_2, val_four_finger_3, val_four_finger_4, bend_val_big_finger, yaw_val_big_finger);
    //    qDebug("DataFrame: ");
    //    qDebug("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //            tUartData.m_send_array[7], tUartData.m_send_array[8], tUartData.m_send_array[9],
    //            tUartData.m_send_array[10], tUartData.m_send_array[11], tUartData.m_send_array[12],
    //            tUartData.m_send_array[13], tUartData.m_send_array[14], tUartData.m_send_array[15],
    //            tUartData.m_send_array[16], tUartData.m_send_array[17], tUartData.m_send_array[18]);
    //Read_Hand_Data(hand_id);
    //std::cout<< "receive data: " << std::endl;
//
    //write(fd, tUartData.m_send_array, tUartData.m_tx_len);
    //read(fd,tUartData.m_rec_array,sizeof(*tUartData.m_rec_array));
    //for (int i = 0; i < sizeof(*tUartData.m_rec_array); i++) {
    //    std::cout << std::hex << static_cast<int>(tUartData.m_rec_array[i]) << " ";
    //}

}

void HandControl::exec_inspire_gripper(double *finger_value_arr_, uint8_t hand_id, int fd)
{

    int gripper_control = 1000 - static_cast<int>((1000 / 1.38) * finger_value_arr_[2]);
    gripper_control *= 1.33;
    // gripper_control += 200;

    if (gripper_control>500){
        setMoveMax();
        // setMoveMin();

    }
    else{
        setMoveMin();
    }
}


bool HandControl::init_dh_gripper(std::string device_name){

        m_gripper = new DH_Gripper(1, device_name, 115200);  //m_gripper(1, "/dev/ttyUSB0", 115200);
        if (m_gripper->open() < 0) {
            printf("Can not open gripper");
        //     RCLCPP_INFO(get_logger(), "无法找到CH340设备");
        }

        m_gripper->Initialization();
        // RCLCPP_INFO(get_logger(),"Gripper initialization sent.");

        int initstate = 0;
        int force = 100;
        int speed = 100;
        int status_counter = 0;
        while (initstate != DH_Gripper::S_INIT_FINISHED) {
            m_gripper->GetInitState(initstate);
            // printf("DH_STATUS: %d\n", initstate);
            if (status_counter > 1000){
                printf("Gripper Init State timeout : can not init gripper! %d\n", initstate);
                return false;
            }
            status_counter++;
        }

        m_gripper->SetTargetForce(force);
        // RCLCPP_INFO(get_logger(),"Set current grip force %f",force);

        m_gripper->SetTargetSpeed(speed);
        // RCLCPP_INFO(get_logger(),"Set current grip speed %f", speed);
        m_gripper->SetTargetPosition(655);
        m_gripper->SetTargetPosition(655);

}


void HandControl::exec_dh_gripper(double *finger_value_arr_, uint8_t hand_id, int fd)
{

    int gripper_control = 1000 - static_cast<int>((1000 / 1.38) * finger_value_arr_[2]);
    gripper_control *= 1.33;
    gripper_control += 200;

    if (gripper_control>500){
        m_gripper->SetTargetPosition(655);
        // setMoveMax();

    }
    else{
        m_gripper->SetTargetPosition(0);
        // setMoveMin();
    }
}


bool HandControl::init_zx_gripper(std::string device_name){

    // const char *PORT = "/dev/ttyUSB0"; // Adjust to your port
    // int SLAVE_ID = 1;

    // // m_gripper_zx = new ZX_gripper(PORT, SLAVE_ID, 115200, 0.5);
    // m_gripper_zx = new ZX_gripper(device_name.c_str(), SLAVE_ID, 115200, 0.5);    
    // m_gripper_zx->enable(true);
    // // close
    // ZX_gripper.temp_move(0, 100, 60, 2000, 2000, true);
    // std::string result = ZX_gripper.wait_until_pos_or_torque(10.0);
    // std::cout << "Move result: " << result << std::endl;

    // // open
    // ZX_gripper.temp_move(12000, 100, 60, 2000, 2000, true); //int position_mm, int speed_pct = 100, int force_pct = 60, int accel = 2000, int decel = 2000, bool trigger = true
    // std::string result = ZX_gripper.wait_until_pos_or_torque(10.0);
    // std::cout << "Move result: " << result << std::endl;

    // printf("ZX Gripper Init State: Done");

    int SLAVE_ID = 1;

    // 创建 ZX_gripper 实例，使用传入的 device_name
    m_gripper_zx = new ZX_gripper(device_name.c_str(), SLAVE_ID, 115200, 0.5);
    
    if (!m_gripper_zx) {
        std::cerr << "Failed to initialize ZX_gripper" << std::endl;
        return false;
    }
    
    m_gripper_zx->enable(true);
    
    // 关闭夹爪
    m_gripper_zx->temp_move(12000, 100, 60, 2000, 2000, true);
    std::string result = m_gripper_zx->wait_until_pos_or_torque(10.0);
    std::cout << "Close result: " << result << std::endl;

    // 打开夹爪
    // m_gripper_zx->temp_move(0, 100, 60, 2000, 2000, true);
    m_gripper_zx->temp_move(6000, 100, 60, 2000, 2000, true);    // half open
    result = m_gripper_zx->wait_until_pos_or_torque(10.0); // 
    std::cout << "Open result: " << result << std::endl;

    printf("ZX Gripper Init State: Done");
    return true; // 添加返回值

}


void HandControl::exec_zx_gripper(double *finger_value_arr_, uint8_t hand_id, int fd)
{

    int gripper_control = 1000 - static_cast<int>((1000 / 1.38) * finger_value_arr_[2]);
    gripper_control *= 1.33;
    gripper_control += 200;
    std::string result;
    if (gripper_control>500){
        // m_gripper->SetTargetPosition(1000);
        // setMoveMax();
        // open gripper
        // m_gripper_zx->temp_move(0, 100, 100, 2000, 2000, true);
        // m_gripper_zx->temp_move(6000, 100, 100, 4000, 4000, true); // half open
        m_gripper_zx->temp_move(6000, 100, 100, 4000, 4000, true); // half open

        // result = m_gripper_zx->wait_until_pos_or_torque(10.0); // 
        result = m_gripper_zx->wait_until_pos_or_torque(1.0); // 

    }
    else{
        // m_gripper->SetTargetPosition(0);
        // setMoveMin();
        // m_gripper_zx->temp_move(12000, 100, 100, 2000, 2000, true);
        m_gripper_zx->temp_move(12000, 100, 100, 4000, 4000, true);        
        // result = m_gripper_zx->wait_until_pos_or_torque(10.0);
        result = m_gripper_zx->wait_until_pos_or_torque(1.0);
        
    }
}



/**
 * 读取手部信息并解析返回数据
 * 
 * @param hand_id 手部ID
 * @param fd 串口文件描述符
 * @return 是否成功读取并解析数据
 */
bool HandControl::read_inspire_hand(uint8_t hand_id, int fd) {
    // 清空接收缓冲区
    //tcflush(fd, TCIFLUSH);
    
    uint8_t i = 0;
    uint8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = hand_id;
    tUartData.m_send_array[3] = 0x04;
    tUartData.m_send_array[4] = kCmd_Handg3_Read;
    tUartData.m_send_array[5] = 0x0A;
    tUartData.m_send_array[6] = 0x06;
    tUartData.m_send_array[7] = 0x0C;
    for (i = 2; i < 8; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[8] = m_unChecksum & 0xFF;
    //tUartData.m_send_array[8] = 0x32;
    tUartData.m_tx_len = 9;
    
    write(fd, tUartData.m_send_array, tUartData.m_tx_len);
    uint8_t *rev_data = new uint8_t[20];
    read(fd, rev_data, 20);
    
    // 打印接收到的数据（十六进制格式）
    // std::cout << "接收到的数据: ";
    // for (int i = 0; i < 20; i++) {
    //     std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(rev_data[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;  // 重置为十进制输出
    
    // 验证数据帧格式
    if (rev_data[0] != 0x90 || rev_data[1] != 0xEB) {
        //std::cout << "错误的包头" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证设备ID
    if (rev_data[2] != hand_id) {
        std::cout << "设备ID不匹配" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证数据长度
    if (rev_data[3] != 0x0F) {
        std::cout << "数据长度错误" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证命令类型
    if (rev_data[4] != 0x11) {
        std::cout << "非预期的命令类型" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 解析寄存器地址
    uint16_t reg_addr = (rev_data[6] << 8) | rev_data[5];
    //std::cout << "寄存器地址: 0x" << std::hex << reg_addr << std::dec << std::endl;
    
    // 解析数据值（6个16位整数）
    uint16_t values[6];
    for (int i = 0; i < 6; i++) {
        values[i] = (rev_data[i*2 + 8] << 8) | rev_data[i*2 + 7];
        //std::cout << "值 " << i + 1 << ": " << values[i] << std::endl;
    }
    
    // 计算校验和
    uint8_t checksum = 0;
    for (int i = 2; i < 19; i++) {
        checksum += rev_data[i];
    }
    
    // 验证校验和
    if (checksum != rev_data[19]) {
        std::cout << "校验和错误，计算值: 0x" << std::hex << static_cast<int>(checksum) 
                  << "，接收值: 0x" << static_cast<int>(rev_data[19]) << std::dec << std::endl;
        delete[] rev_data;
        return false;
    }
    
    //std::cout << "数据校验通过" << std::endl;
    
    // 更新手部状态数据
    // 这里需要根据具体的寄存器地址来确定数据的含义
    // 假设这些值代表角度信息
    if (reg_addr == 0x060A) {  // 根据实际协议确认寄存器地址的含义
        tHand.tLittleFinger.m_current_angle = values[0];
        tHand.tRingFinger.m_current_angle = values[1];
        tHand.tMiddleFinger.m_current_angle = values[2];
        tHand.tIndexFinger.m_current_angle = values[3];
        tHand.tThumbBend.m_current_angle = values[4];
        tHand.tThumbSide.m_current_angle = values[5];

        finger_value_act_[0] = 1.2 - static_cast<double>(values[0])/1000 * 1.2;
        finger_value_act_[1] = 0.55 - static_cast<double>(values[1])/1000 * 0.55;
        finger_value_act_[2] = 1.7 - static_cast<double>(values[2])/1000 * 1.7;
        finger_value_act_[3] = 1.7 - static_cast<double>(values[3])/1000 * 1.7;
        finger_value_act_[4] = 1.7 - static_cast<double>(values[4])/1000 * 1.7;
        finger_value_act_[5] = 1.7 - static_cast<double>(values[5])/1000 * 1.7;
    }
    
    delete[] rev_data;
    return true;
}



/**
 * 读取手部信息并解析返回数据
 * 
 * @param hand_id 手部ID
 * @param fd 串口文件描述符
 * @return 是否成功读取并解析数据
 */
bool HandControl::read_inspire_hand_all(uint8_t hand_id, int fd) {
    // 清空接收缓冲区
    //tcflush(fd, TCIFLUSH);
    
    u_int8_t i = 0;
    u_int8_t m_unChecksum = 0;
    tUartData.m_send_array[0] = kSend_Frame_Head1;
    tUartData.m_send_array[1] = kSend_Frame_Head2;
    tUartData.m_send_array[2] = hand_id;
    tUartData.m_send_array[3] = 0x0D;
    tUartData.m_send_array[4] = kCmd_Mc_All;
    for (i = 5; i < 17; i++)
    {
        tUartData.m_send_array[i] = 0xFF;
    }
    for (i = 2; i < 17; i++)
    {
        m_unChecksum += tUartData.m_send_array[i];
    }
    tUartData.m_send_array[17] = m_unChecksum & 0xFF;
    tUartData.m_tx_len = 18;
    
    write(fd, tUartData.m_send_array, tUartData.m_tx_len);
    uint8_t *rev_data = new uint8_t[128];
    //read(fd, rev_data, 128);
    read(fd,tUartData.m_rec_array,128);
    //打印接收到的数据（十六进制格式）
    std::cout << "接收到的数据: ";
    for (int i = 0; i < 128; i++) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(rev_data[i]) << " ";
    }
    std::cout << std::dec << std::endl;  // 重置为十进制输出
    Uart_Rece_Data_Parsing();
    print_hand_data();
    // 验证数据帧格式
    if (rev_data[0] != 0x90 || rev_data[1] != 0xEB) {
        std::cout << "错误的包头" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证设备ID
    if (rev_data[2] != hand_id) {
        std::cout << "设备ID不匹配" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证数据长度
    if (rev_data[3] != 0x0D) {
        std::cout << "数据长度错误" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 验证命令类型
    if (rev_data[4] != 0x17) {
        std::cout << "非预期的命令类型" << std::endl;
        delete[] rev_data;
        return false;
    }
    
    // 解析寄存器地址
    uint16_t reg_addr = (rev_data[6] << 8) | rev_data[5];
    //std::cout << "寄存器地址: 0x" << std::hex << reg_addr << std::dec << std::endl;
    
    // 解析数据值（6个16位整数）
    uint16_t values[6];
    for (int i = 0; i < 6; i++) {
        values[i] = (rev_data[i*2 + 8] << 8) | rev_data[i*2 + 7];
        //std::cout << "值 " << i + 1 << ": " << values[i] << std::endl;
    }
    
    // 计算校验和
    uint8_t checksum = 0;
    for (int i = 2; i < 19; i++) {
        checksum += rev_data[i];
    }
    
    // 验证校验和
    if (checksum != rev_data[19]) {
        std::cout << "校验和错误，计算值: 0x" << std::hex << static_cast<int>(checksum) 
                  << "，接收值: 0x" << static_cast<int>(rev_data[19]) << std::dec << std::endl;
        delete[] rev_data;
        return false;
    }
    
    std::cout << "数据校验通过" << std::endl;
    
    // 更新手部状态数据
    // 这里需要根据具体的寄存器地址来确定数据的含义
    // 假设这些值代表角度信息
    if (reg_addr == 0x060A) {  // 根据实际协议确认寄存器地址的含义
        tHand.tLittleFinger.m_current_angle = values[0];
        tHand.tRingFinger.m_current_angle = values[1];
        tHand.tMiddleFinger.m_current_angle = values[2];
        tHand.tIndexFinger.m_current_angle = values[3];
        tHand.tThumbBend.m_current_angle = values[4];
        tHand.tThumbSide.m_current_angle = values[5];
        finger_value_act_[0] = values[0];
        finger_value_act_[1] = values[1];
        finger_value_act_[2] = values[2];
        finger_value_act_[3] = values[3];
        finger_value_act_[4] = values[4];
        finger_value_act_[5] = values[5];
    }
    
    delete[] rev_data;
    return true;
}



/**
 * 打印手部数据信息
 */
void HandControl::print_hand_data() {
    std::cout << "========= 手部数据信息 =========" << std::endl;
    std::cout << "手部ID: " << static_cast<int>(g_unHand_id) << std::endl;
    
    // 打印各个手指的角度信息
    std::cout << "角度信息 (单位: 度):" << std::endl;
    std::cout << "  小指: " << tHand.tLittleFinger.m_current_angle << std::endl;
    std::cout << "  无名指: " << tHand.tRingFinger.m_current_angle << std::endl;
    std::cout << "  中指: " << tHand.tMiddleFinger.m_current_angle << std::endl;
    std::cout << "  食指: " << tHand.tIndexFinger.m_current_angle << std::endl;
    std::cout << "  拇指弯曲: " << tHand.tThumbBend.m_current_angle << std::endl;
    std::cout << "  拇指侧摆: " << tHand.tThumbSide.m_current_angle << std::endl;
    
    // 打印各个手指的力反馈信息
    std::cout << "力反馈信息:" << std::endl;
    std::cout << "  小指: " << tHand.tLittleFinger.m_current_forceact << std::endl;
    std::cout << "  无名指: " << tHand.tRingFinger.m_current_forceact << std::endl;
    std::cout << "  中指: " << tHand.tMiddleFinger.m_current_forceact << std::endl;
    std::cout << "  食指: " << tHand.tIndexFinger.m_current_forceact << std::endl;
    std::cout << "  拇指弯曲: " << tHand.tThumbBend.m_current_forceact << std::endl;
    std::cout << "  拇指侧摆: " << tHand.tThumbSide.m_current_forceact << std::endl;
    
    // 打印各个手指的电流信息
    std::cout << "电流信息 (单位: mA):" << std::endl;
    std::cout << "  小指: " << tHand.tLittleFinger.m_current_current << std::endl;
    std::cout << "  无名指: " << tHand.tRingFinger.m_current_current << std::endl;
    std::cout << "  中指: " << tHand.tMiddleFinger.m_current_current << std::endl;
    std::cout << "  食指: " << tHand.tIndexFinger.m_current_current << std::endl;
    std::cout << "  拇指弯曲: " << tHand.tThumbBend.m_current_current << std::endl;
    std::cout << "  拇指侧摆: " << tHand.tThumbSide.m_current_current << std::endl;
    
    // 打印各个手指的错误代码
    std::cout << "错误代码:" << std::endl;
    std::cout << "  小指: 0x" << std::hex << static_cast<int>(tHand.tLittleFinger.m_error_code) << std::endl;
    std::cout << "  无名指: 0x" << std::hex << static_cast<int>(tHand.tRingFinger.m_error_code) << std::endl;
    std::cout << "  中指: 0x" << std::hex << static_cast<int>(tHand.tMiddleFinger.m_error_code) << std::endl;
    std::cout << "  食指: 0x" << std::hex << static_cast<int>(tHand.tIndexFinger.m_error_code) << std::endl;
    std::cout << "  拇指弯曲: 0x" << std::hex << static_cast<int>(tHand.tThumbBend.m_error_code) << std::endl;
    std::cout << "  拇指侧摆: 0x" << std::hex << static_cast<int>(tHand.tThumbSide.m_error_code) << std::dec << std::endl;
    
    std::cout << "===============================" << std::endl;
}


void HandControl::printTimestamp(uint64_t timestamp)
{
    auto tp = std::chrono::time_point<std::chrono::system_clock,
                                      std::chrono::microseconds>(
        std::chrono::microseconds(timestamp));
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm *datetime = std::localtime(&tt);
}

void HandControl::printTimestampEx(uint64_t timestamp, const char *type, int32_t count)
{
    auto tp = std::chrono::time_point<std::chrono::system_clock,
                                      std::chrono::microseconds>(
        std::chrono::microseconds(timestamp));
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm *datetime = std::localtime(&tt);
}


//json HandControl::readJsonFile(const std::string &filename)
//{
//    std::ifstream file(filename);
//    if (!file.is_open())
//    {
//        return nullptr;
//    }

//    json config;
//    file >> config;
//    file.close();
//    return config;
//}

