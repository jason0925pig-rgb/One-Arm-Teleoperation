#ifndef __dh_device__
#define __dh_device__

#include <QSerialPort>

class dh_device:public QObject
{
    Q_OBJECT
public:
    dh_device(QObject *parent = nullptr);
    ~dh_device();

    // connect uart
    int connect_device(QString portname, int Baudrate);

    //disconnect
    void disconnect_device();

    //write data
    int device_wrire(QByteArray data);

    //read data
    int device_read(QByteArray &data);

    //VSMD write data
    int vsmd_write(int number, const QString &function, int value);
    int vsmd_write(int number, const QString &function);

    //VSMD read data
    int vsmd_read(QByteArray &data);

private:
    QSerialPort *_m_serial;
};

#endif //__dh_device__
