#include "DH_gripper.hpp"

DH_Gripper::DH_Gripper(int id, std::string Portname, int Baudrate)
    :_gripper_id(id), _PortName(Portname), _BaudRate(Baudrate), _Serialhandle(-1)
{
    this->gripper_axis = 1;
}
DH_Gripper::DH_Gripper()
{
    // std::string config_path = std::string(CONFIG_DIR) + "/select_executor.yaml";
    // load_config(config_path);
}

DH_Gripper::~DH_Gripper()
{
    if (_Serialhandle >= 0)
        close();
}

int DH_Gripper::open()
{
    _Serialhandle = connect_device(_PortName, _BaudRate);

    if (_Serialhandle < 0)
    {
        std::cout << "DH_Gripper open failed" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "DH_Gripper open successful" << std::endl;
        return _Serialhandle;
    }
}

void DH_Gripper::close()
{
    disconnect_device(_Serialhandle);
    _Serialhandle = -1;
}

bool DH_Gripper::Initialization()
{
    return WriteRegisterFunc(0x0100, 0xA5);
}

bool DH_Gripper::SetTargetPosition(int refpos)
{
    return WriteRegisterFunc(0x0103, refpos);
}

bool DH_Gripper::SetTargetForce(int force)
{
    return WriteRegisterFunc(0x0101, force);
}

bool DH_Gripper::SetTargetSpeed(int speed)
{
    return WriteRegisterFunc(0x0104, speed);
}

bool DH_Gripper::GetCurrentPosition(int &curpos)
{
    return ReadRegisterFunc(0x0202, curpos);
}

bool DH_Gripper::GetTargetPosition(int &tarpos)
{
    return ReadRegisterFunc(0x0103, tarpos);
}

bool DH_Gripper::GetTargetForce(int &curTarforce)
{
    return ReadRegisterFunc(0x0101, curTarforce);
}

bool DH_Gripper::GetTargetSpeed(int &curTarpos)
{
    return ReadRegisterFunc(0x0104, curTarpos);
}

bool DH_Gripper::GetInitState(int &i_state)
{
    return ReadRegisterFunc(0x0200, i_state);
}

bool DH_Gripper::GetGripState(int &g_state)
{
    return ReadRegisterFunc(0x0201, g_state);
}

bool DH_Gripper::GetRunStates(int states[])
{
    if (this->GetInitState(states[0]))
        if (this->GetGripState(states[1]))
            if (this->GetCurrentPosition(states[2]))
                if (this->GetTargetPosition(states[3]))
                    if (this->GetTargetForce(states[4]))
                    {
                        return true;
                    }
    return false;
}

bool DH_Gripper::WriteRegisterFunc(int index, int value)
{
    unsigned char send_buf[8];
    send_buf[0] = _gripper_id;
    send_buf[1] = 0x06;
    send_buf[2] = (index >> 8) & 0xff;
    send_buf[3] = index & 0xff;
    send_buf[4] = (value >> 8) & 0xff;
    send_buf[5] = value & 0xff;
    unsigned short crc = CRC16(send_buf, sizeof(send_buf) - 2);
    send_buf[6] = crc & 0xff;
    send_buf[7] = (crc >> 8) & 0xff;

    bool ret = false;
    int retrycount = 3;
    do
    {
        ret = false;
        retrycount--;
        if (retrycount < 0)
            break;

        int wdlen = device_write(_Serialhandle, (char *)send_buf, sizeof(send_buf));
        if (sizeof(send_buf) != wdlen)
            continue;

        char rev_buf[32];
        int rdlen = device_read(_Serialhandle, rev_buf, sizeof(rev_buf) - 1);

        if (rdlen == sizeof(send_buf))
        {
            bool checkrev = true;
            for (int i = 0; i < rdlen; i++)
            {
                if (send_buf[i] != (unsigned char)rev_buf[i])
                {
                    checkrev = false;
                    break;
                }
            }
            if (checkrev)
                ret = true;
        }
    } while (!ret);

    return ret;
}

bool DH_Gripper::ReadRegisterFunc(int index, int &value)
{
    unsigned char send_buf[8];
    send_buf[0] = _gripper_id;
    send_buf[1] = 0x03;
    send_buf[2] = (index >> 8) & 0xff;
    send_buf[3] = index & 0xff;
    send_buf[4] = 0x00;
    send_buf[5] = 0x01;
    unsigned short crc = CRC16(send_buf, sizeof(send_buf) - 2);
    send_buf[6] = crc & 0xff;
    send_buf[7] = (crc >> 8) & 0xff;

    bool ret = false;
    int retrycount = 100;
    do
    {
        ret = false;
        retrycount--;
        if (retrycount < 0)
            break;

        int wdlen = device_write(_Serialhandle, (char *)send_buf, sizeof(send_buf));
        if (sizeof(send_buf) != wdlen)
            continue;

        char rev_buf[32];
        int rdlen = device_read(_Serialhandle, rev_buf, sizeof(rev_buf) - 1);

        if (rdlen == 7)
        {
            unsigned short crc = CRC16((unsigned char *)rev_buf, rdlen - 2);
            int revcrch = (unsigned int)(unsigned char)rev_buf[6];
            int revcrcl = (unsigned int)(unsigned char)rev_buf[5];
            unsigned short revcrc = revcrch * 256 + revcrcl;

            if (crc != revcrc)
                continue;
            if (rev_buf[0] != _gripper_id || rev_buf[1] != 0x03)
                continue;

            value = ((rev_buf[4] & 0xff) | (rev_buf[3] << 8));
            ret = true;
        }
    } while (!ret);
    return ret;
}

unsigned short DH_Gripper::CRC16(const unsigned char *nData, unsigned short wLength)
{
    static const unsigned short wCRCTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

    unsigned char nTemp;
    unsigned short wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}

int DH_Gripper::serial_connect(std::string portname, int Baudrate)
{
    int fd = -1;
    fd = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname.c_str(), strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    if (Baudrate == 115200)
    {
        set_interface_attribs(fd, B115200);
    }
    else if (Baudrate == 38400)
    {
        set_interface_attribs(fd, B38400);
    }
    else if (Baudrate == 19200)
    {
        set_interface_attribs(fd, B19200);
    }
    else if (Baudrate == 9600)
    {
        set_interface_attribs(fd, B9600);
    }
    return fd;
}

int DH_Gripper::tcp_connect(std::string ip_port)
{
    std::string servInetAddr = ip_port.substr(0, ip_port.find(":"));
    int PORT = atoi(ip_port.substr(ip_port.find(":") + 1, ip_port.size() - ip_port.find(":") - 1).c_str());

    /*创建socket*/
    struct sockaddr_in serv_addr;
    int sockfd = -1;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) != -1)
    {
        printf("Socket id = %d \n", sockfd);
        /*设置sockaddr_in 结构体中相关参数*/
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        inet_pton(AF_INET, servInetAddr.c_str(), &serv_addr.sin_addr);
        bzero(&(serv_addr.sin_zero), 8);
        /*调用connect 函数主动发起对服务器端的连接*/
        if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
        {
            printf("Connect failed!\n");
            return -1;
        }
        else
        {
            printf("connected\n");
            return sockfd;
        }
    }
    else
    {
        printf("Socket failed!\n");
        return -1;
    }
}

int DH_Gripper::connect_device(std::string portname, int parameter)
{
    int fd;
    if (portname.find(":") != portname.npos)
    {
        fd = tcp_connect(portname);
    }
    else
    {
        fd = serial_connect(portname, parameter);
    }
    return fd;
}

void DH_Gripper::disconnect_device(int fd)
{
    ::close(fd);
}

int DH_Gripper::device_write(int fd, char *data, int len)
{
    int wlen;
    wlen = write(fd, data, len);
    if (wlen == len)
    {
        return wlen;
    }
    else
    {
        tcflush(fd, TCOFLUSH);
        return 0;
    }
}

int DH_Gripper::device_read(int fd, char *data, int data_len)
{
    int len, fs_sel;
    fd_set fs_read;
    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 0;
    time.tv_usec = 200000;

    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if (fs_sel)
    {
        len = read(fd, data, data_len);
        return len;
    }
    else
    {
        return -1;
    }
}

int DH_Gripper::set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void DH_Gripper::set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5; /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}
