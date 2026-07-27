#include "dh_modbus_gripper.h"
#include <QDebug>
#include <QDateTime>

DH_Modbus_Gripper::DH_Modbus_Gripper(int id, std::string Portname, int Baudrate)
    :DH_Gripper(), _gripper_id(id),_PortName(Portname),_BaudRate(Baudrate),_Serialhandle(-1),
     write_counter(0),write_rev_counter(0),write_error_counter(0),write_check_counter(0),read_counter(0),read_rev_counter(0),read_error_counter(0)
{
    _m_device = new dh_device();
}

DH_Modbus_Gripper::~DH_Modbus_Gripper()
{

}

int DH_Modbus_Gripper::open()
{
    _Serialhandle = _m_device->connect_device(_PortName.c_str(), _BaudRate);

    if(_Serialhandle < 0)
    {
        std::cout << "open failed"<<std::endl;
        return -1;
    }
    else
    {
        std::cout << "open successful"<<std::endl;
        return 0;
    }
}

void DH_Modbus_Gripper::close()
{
    _m_device->disconnect_device();
}


bool DH_Modbus_Gripper::Initialization()
{
   return WriteRegisterFunc(0x0100,0xA5);
//    return VsmdWriteRegisterFunc(0x0100,0xA5);
}


bool DH_Modbus_Gripper::SetTargetPosition(int refpos)
{
    return WriteRegisterFunc(0x0103,refpos);
//    return VsmdWriteRegisterFunc(0x0103,refpos);
}

bool DH_Modbus_Gripper::SetTargetForce(int force)
{
    return WriteRegisterFunc(0x0101,force);
//    return VsmdWriteRegisterFunc(0x0101,force);
}

bool DH_Modbus_Gripper::SetTargetSpeed(int speed)
{
    return WriteRegisterFunc(0x0104,speed);
//    return VsmdWriteRegisterFunc(0x0104,speed);
}


bool DH_Modbus_Gripper::GetCurrentPosition(int *curpos)
{
    return ReadRegisterFunc(0x0202,curpos);
//    return VsmdReadRegisterFunc(0x0202,curpos);
}

bool DH_Modbus_Gripper::GetCurrentTargetForce(int *curTarforce)
{
    return ReadRegisterFunc(0x0101,curTarforce);
//    return VsmdReadRegisterFunc(0x0101,curTarforce);
}

bool DH_Modbus_Gripper::GetCurrentTargetSpeed(int *curTarpos)
{
    return ReadRegisterFunc(0x0104,curTarpos);
//    return VsmdReadRegisterFunc(0x0104,curTarpos);
}


bool DH_Modbus_Gripper::GetInitState(int *i_state)
{
     return ReadRegisterFunc(0x0200,i_state);
//     return VsmdReadRegisterFunc(0x0200,i_state);
}

bool DH_Modbus_Gripper::GetGripState(int *g_state)
{
     return ReadRegisterFunc(0x0201,g_state);
//    return VsmdReadRegisterFunc(0x0201,g_state);
}

bool DH_Modbus_Gripper::VsmdWriteRegisterFunc(int index, int value)
{
    bool ret = false;
    int retrycount = 3;
    int dev_id = _gripper_id;
    do
    {
        ret = false;
        QString command = "";
        retrycount -- ;
        if(retrycount<0)
        {
            break;
        }
        
        int wdlen = 0;
        write_counter ++ ;
        if(index == 0x0100)
        {
            command = "zero start";
            wdlen = _m_device->vsmd_write(dev_id, command);
        }
        if(index == 0x0101)
        {
            command = "cfg crh=";
            wdlen = _m_device->vsmd_write(dev_id, command, value);
        }
        if(index == 0x0103)
        {
            command = "pos ";
            wdlen = _m_device->vsmd_write(dev_id, command, value);
        }
        if(index == 0x0104)
        {
            command = "cfg spd=";
            wdlen = _m_device->vsmd_write(dev_id, command, value);
        }

        QByteArray rev_buf;
        int rdlen = _m_device->vsmd_read(rev_buf);
//        qDebug() << "write_return: " << rdlen;
        if (rdlen == 10) {
            write_rev_counter ++ ;
            bool checkrev = true;
//            for(int i=0;i<rdlen;i++)
//            {
//                if(send_buf[i] != (unsigned char)rev_buf.at(i))
//                {
//                     qDebug() << "write check error! "<<rev_buf.toHex();
//                    write_check_counter++;
//                    checkrev = false;
//                    break;
//                }
//            }
            if(checkrev)
                ret = true;
        }

        else
        {
            qDebug() << "write error!  get "<< rev_buf.toHex();
            write_error_counter++;
        }

    } while(!ret);
    return ret;
}

bool DH_Modbus_Gripper::VsmdReadRegisterFunc(int index, int *value)
{
    bool ret = false;
    int retrycount = 3;
    int dev_id = _gripper_id;
    do
    {
        ret = false;
        retrycount -- ;
        if(retrycount<0)
        {
            break;
        }

        read_counter++;
        _m_device->vsmd_write(dev_id,"sts");

        QByteArray rev_buf;
        int rdlen = _m_device->vsmd_read(rev_buf);
//        qDebug() << "rev_buf"<< rev_buf;

        if (rdlen == 10) {
            read_rev_counter++;
            bool checkrev = true;
            if(rev_buf.at(1) != _gripper_id || rev_buf.at(2) != 0x02)
                checkrev = false;

            if(checkrev)
            {
                if(index == 0x0200)
                    *value = rev_buf[3]&0xff;
                if(index == 0x0201)
                    *value = rev_buf[4]&0xff;
                if(index == 0x0202)
                    *value = ((rev_buf[6]&0x7f)|(rev_buf[5]<<7));

                // std::cout << "value "<< *value << std::endl;
//                qDebug << "value "<< value;
                ret = true;
            }
        }
        else
        {
            qDebug() << "read error!  get "<< rev_buf.toHex();
            read_error_counter++;
        }

    } while(!ret);

    return ret;
}

 bool DH_Modbus_Gripper::WriteRegisterFunc(int index, int value)
 {
     unsigned char send_buf[8];
     send_buf[0] = _gripper_id;
     send_buf[1] = 0x06;
     send_buf[2] = (index>>8)&0xff;
     send_buf[3] = index&0xff;
     send_buf[4] = (value>>8)&0xff;
     send_buf[5] = value&0xff;
     unsigned short crc = CRC16(send_buf,sizeof(send_buf)-2);
     send_buf[6] = crc&0xff;
     send_buf[7] = (crc>>8)&0xff;
    
     QByteArray send_temp((char *)send_buf,8);

     bool ret = false;
     int retrycount = 3;
     do
     {
         ret = false;
         retrycount -- ;
         if(retrycount<0)
         {
             break;
         }

         write_counter ++ ;
         int wdlen = _m_device->device_wrire(send_temp);
         qDebug()<<"size "<< send_temp.size()<< " "<< wdlen;
         if(send_temp.size() != wdlen)
         {
             qDebug() << "write error! "<<send_temp.toHex();
             write_error_counter++;
             continue;
         }

         QByteArray rev_buf;
         int rdlen = _m_device->device_read(rev_buf);
        
         if (rdlen == sizeof(send_buf)) {
              bool checkrev = true;
             for(int i=0;i<rdlen;i++)
             {
                 if(send_buf[i] != (unsigned char)rev_buf.at(i))
                 {
                      qDebug() << "write check error! "<<rev_buf.toHex();
                     write_check_counter++;
                     checkrev = false;
                     break;
                 }
             }
             if(checkrev)
                 ret = true;
         }
        
     } while(!ret);
    
     return ret;
 }


 bool DH_Modbus_Gripper::ReadRegisterFunc(int index,int *value)
 {
     unsigned char send_buf[8];
     send_buf[0] = _gripper_id;
     send_buf[1] = 0x03;
     send_buf[2] = (index>>8)&0xff;
     send_buf[3] = index&0xff;
     send_buf[4] = 0x00;
     send_buf[5] = 0x01;
     unsigned short crc = CRC16(send_buf,sizeof(send_buf)-2);
     send_buf[6] = crc&0xff;
     send_buf[7] = (crc>>8)&0xff;
    
     QByteArray send_temp((char *)send_buf,8);


     bool ret = false;
     int retrycount = 3;
     do
     {
         ret = false;
         retrycount -- ;
         if(retrycount<0)
         {
             break;
         }

         read_counter++;

         int wdlen = _m_device->device_wrire(send_temp);
         if(sizeof(send_buf) != wdlen)
             continue;

         QByteArray rev_buf;
         int rdlen = _m_device->device_read(rev_buf);
        

         if (rdlen == 7) {


             bool checkrev = true;
             if(rev_buf.at(0) != _gripper_id || rev_buf.at(1) != 0x03)
                 checkrev = false;
        
             unsigned short crc = CRC16((unsigned char *)(rev_buf.toStdString().c_str()),rdlen-2);

             int revcrch = (unsigned int) (unsigned char)rev_buf[6];
             int revcrcl = (unsigned int) (unsigned char)rev_buf[5];

             unsigned short revcrc =  revcrch * 256 + revcrcl;

//             if(crc != revcrc)
//             {
//                 qDebug() << "CRC error~ cacu:"<< crc <<" rev: "<< revcrc << "get "<< rev_buf.toHex();
//                 read_crc_counter++;
//                 checkrev = false;
//             }

            // std::cout <<"get";
            // for(int i=0;i<7;i++)
            //     std::cout<< std::hex<<(unsigned int)(unsigned char)  rev_buf[i] << " ";
            // std::cout << std::endl;

            if(checkrev)
            {
                *value = ((rev_buf[4]&0xff)|(rev_buf[3]<<8));
                // std::cout << "value "<< *value << std::endl;
                ret = true;
            }
        }
        else
        {
            qDebug() << "read error!  get "<< rev_buf.toHex();
             read_error_counter++;
         }
        
     } while(!ret);
    
     return ret;
 }

QString DH_Modbus_Gripper::Get_All_Counter()
{
//    return QString("W %1 Werr %2 Wck %3 R %4 Rerr %5 Rcrc %6")
//            .arg(QString::number(write_counter))
//            .arg(QString::number(write_error_counter))
//            .arg(QString::number(write_check_counter))
//            .arg(QString::number(read_counter))
//            .arg(QString::number(read_error_counter))
//            .arg(QString::number(read_crc_counter));

    return QString("W %1 Wr %2 Werr %3 R %4 Rr %5 Rerr %6")
            .arg(QString::number(write_counter))
            .arg(QString::number(write_rev_counter))
            .arg(QString::number(write_error_counter))
            .arg(QString::number(read_counter))
            .arg(QString::number(read_rev_counter))
            .arg(QString::number(read_error_counter));
}

void DH_Modbus_Gripper::GetSystemTime(QString *system_time){
    QDateTime dateTime = QDateTime::currentDateTime();
    // 字符串格式化
    QString timestamp = dateTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    *system_time = timestamp;
    // 获取毫秒值
//    int ms = dateTime.time().msec();
    // 转换成时间戳
//    qint64 epochTime = dateTime.toMSecsSinceEpoch();
}

unsigned short DH_Modbus_Gripper::CRC16 (const unsigned char *nData, unsigned short wLength)
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
   0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

unsigned char nTemp;
unsigned short wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;

} // End: CRC16


