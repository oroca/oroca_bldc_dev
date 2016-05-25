#include <QCoreApplication>
#include "serialportwriter.h"
#include "serialportreader.h"
#include <QtSerialPort/QSerialPort>

#include <mavlink/common/mavlink.h>

void send();
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    /*
    QSerialPort serialPort_w;
    serialPort_w.setPortName("COM3");
    serialPort_w.setBaudRate(QSerialPort::Baud9600);
*/

    QSerialPort serialPort_r;
    serialPort_r.setPortName("COM4");
    serialPort_r.setBaudRate(QSerialPort::Baud9600);


    //serialPort_w.open(QIODevice::WriteOnly);
    serialPort_r.open(QIODevice::ReadWrite);

    //SerialPortWriter serialPortWriter(&serialPort_w);
    //serialPortWriter.write(writeData);

    SerialPortReader serialPortReader(&serialPort_r);

    //send();

    return a.exec();
}
