#include <dh_device.h>
#include <QDebug>
dh_device::dh_device(QObject *parent) :
    QObject(parent)
{
    _m_serial = new QSerialPort(this);
}

dh_device::~dh_device()
{
    if(_m_serial)
        _m_serial->deleteLater();
}

int dh_device::connect_device(QString portname, int Baudrate)
{

    if(!_m_serial)
        return -1;

    int ret;
    _m_serial->setPortName(portname);
    _m_serial->setBaudRate(Baudrate);
    _m_serial->setDataBits(QSerialPort::Data8);
    _m_serial->setParity(QSerialPort::NoParity);
    _m_serial->setStopBits(QSerialPort::OneStop);
    _m_serial->setFlowControl(QSerialPort::NoFlowControl);
    if (_m_serial->open(QIODevice::ReadWrite)) {
        qDebug()<< "Opened ";
        ret = 0;
    } else {
        qDebug()<< "Open error";
        ret = -1;
    }
    return ret;
}


void dh_device::disconnect_device()
{
    if(!_m_serial)
        return;

    _m_serial->close();
}


 int dh_device::device_wrire(QByteArray data)
 {
     if(!_m_serial)
         return -1;

     int wlen;
     wlen = _m_serial->write(data.toStdString().c_str(),data.length());
     if (wlen == data.length() )
 	{
 		return wlen;
 	}
     else
 	{
         qDebug()<< "write error! send data is " << data;
 		return 0;
 	}
 }

 int dh_device::device_read(QByteArray &data)
 {

     if(!_m_serial)
         return -1;

     QByteArray responseData;
     _m_serial->waitForReadyRead(500);
      qDebug()<<"get "<<_m_serial->bytesAvailable();
     if (_m_serial->bytesAvailable()>=1) {
         responseData = _m_serial->readAll();
         while (_m_serial->waitForReadyRead(30))
             responseData += _m_serial->readAll();

         data = responseData;

         return responseData.size();
     }
     else
     {
         qDebug()<< "read timeout";
         return -1;
     }
 }

int dh_device::vsmd_write(int number, const QString &function, int value)
{
    if(!_m_serial)
        return -1;

    QString command = QString("%1 %2%3%4").arg(number).arg(function).arg(value).arg("\n");
    QByteArray byte = command.toLatin1();

    int wlen;
    wlen = _m_serial->write(byte,byte.length());
    if (wlen == byte.length() )
    {
        return wlen;
    }
    else
    {
        qDebug()<< "write error! send data is " << byte;
        return 0;
    }
}

int dh_device::vsmd_write(int number, const QString &function)
{
    if(!_m_serial)
        return -1;

    QString command = QString("%1 %2%3").arg(number).arg(function).arg("\n");
    QByteArray byte = command.toLatin1();

     _m_serial->write(byte);
    int wlen;
    wlen = _m_serial->write(byte,byte.length());
    if (wlen == byte.length() )
    {
        return wlen;
    }
    else
    {
        qDebug()<< "write error! send data is " << byte;
        return 0;
    }
}

int dh_device::vsmd_read(QByteArray &data)
{
    if(!_m_serial)
        return -1;

    QByteArray responseData;
    _m_serial->waitForReadyRead(500);
//    qDebug()<<"get "<<_m_serial->bytesAvailable();
    if (_m_serial->bytesAvailable()>=1) {
        responseData = _m_serial->readAll();
        while (_m_serial->waitForReadyRead(30))
            responseData += _m_serial->readAll();

//        qDebug()<< "read buf" << responseData.toHex();
//        qDebug()<< "read buf" << responseData;
        data = responseData;

        return responseData.size();
    }
    else
    {
        qDebug()<< "read timeout";
        return -1;
    }
}




