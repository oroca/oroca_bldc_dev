#include "dialog.h"
#include "ui_dialog.h"
#include "qextserialport.h"
#include "qextserialenumerator.h"
#include <QtCore>
#include <QFileDialog>
#include <QTextCodec>
#include <QCoreApplication>
#include <QMessageBox>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QDebug>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/time.h>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    //! [0]
    foreach (QextPortInfo info, QextSerialEnumerator::getPorts())
        ui->portBox->addItem(info.portName);
    //make sure user can input their own port name!
    ui->portBox->setEditable(true);

    ui->baudRateBox->addItem("1200", BAUD1200);
    ui->baudRateBox->addItem("2400", BAUD2400);
    ui->baudRateBox->addItem("4800", BAUD4800);
    ui->baudRateBox->addItem("9600", BAUD9600);
    ui->baudRateBox->addItem("19200", BAUD19200);
    ui->baudRateBox->addItem("115200", BAUD115200);
    ui->baudRateBox->setCurrentIndex(5);

    ui->parityBox->addItem("NONE", PAR_NONE);
    ui->parityBox->addItem("ODD", PAR_ODD);
    ui->parityBox->addItem("EVEN", PAR_EVEN);

    ui->dataBitsBox->addItem("5", DATA_5);
    ui->dataBitsBox->addItem("6", DATA_6);
    ui->dataBitsBox->addItem("7", DATA_7);
    ui->dataBitsBox->addItem("8", DATA_8);
    ui->dataBitsBox->setCurrentIndex(3);

    ui->stopBitsBox->addItem("1", STOP_1);
    ui->stopBitsBox->addItem("2", STOP_2);

    ui->queryModeBox->addItem("Polling", QextSerialPort::Polling);
    ui->queryModeBox->addItem("EventDriven", QextSerialPort::EventDriven);
    //! [0]

    timer = new QTimer(this);
    timer->setInterval(40);

    timer_colck = new QTimer(this);
    connect(timer_colck, SIGNAL(timeout()), this, SLOT(onClockLabelUpdate()));
    timer_colck->start(1000);

    //! [1]
    PortSettings settings = {BAUD9600, DATA_8, PAR_NONE, STOP_1, FLOW_OFF, 10};
    port = new QextSerialPort(ui->portBox->currentText(), settings, QextSerialPort::Polling);
    //! [1]

    enumerator = new QextSerialEnumerator(this);
    enumerator->setUpNotifications();

    connect(ui->baudRateBox, SIGNAL(currentIndexChanged(int)), SLOT(onBaudRateChanged(int)));
    connect(ui->parityBox, SIGNAL(currentIndexChanged(int)), SLOT(onParityChanged(int)));
    connect(ui->dataBitsBox, SIGNAL(currentIndexChanged(int)), SLOT(onDataBitsChanged(int)));
    connect(ui->stopBitsBox, SIGNAL(currentIndexChanged(int)), SLOT(onStopBitsChanged(int)));
    connect(ui->queryModeBox, SIGNAL(currentIndexChanged(int)), SLOT(onQueryModeChanged(int)));
    connect(ui->timeoutBox, SIGNAL(valueChanged(int)), SLOT(onTimeoutChanged(int)));
    connect(ui->portBox, SIGNAL(editTextChanged(QString)), SLOT(onPortNameChanged(QString)));
    connect(ui->openCloseButton, SIGNAL(clicked()), SLOT(onOpenCloseButtonClicked()));

    connect(timer, SIGNAL(timeout()), SLOT(onReadyRead()));
    connect(port, SIGNAL(readyRead()), SLOT(onReadyRead()));

    connect(enumerator, SIGNAL(deviceDiscovered(QextPortInfo)), SLOT(onPortAddedOrRemoved()));
    connect(enumerator, SIGNAL(deviceRemoved(QextPortInfo)), SLOT(onPortAddedOrRemoved()));

    QDateTime local(QDateTime::currentDateTime());
    ui->label_13->setText(local.toString());

    setWindowTitle("OROCA BLCD GUI v1.0");
}

Dialog::~Dialog()
{
    delete ui;
}
void Dialog::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void Dialog::onPortNameChanged(const QString & /*name*/)
{
    if (port->isOpen()) {
        port->close();
    }
}
//! [2]
void Dialog::onBaudRateChanged(int idx)
{
    port->setBaudRate((BaudRateType)ui->baudRateBox->itemData(idx).toInt());
}

void Dialog::onParityChanged(int idx)
{
    port->setParity((ParityType)ui->parityBox->itemData(idx).toInt());
}

void Dialog::onDataBitsChanged(int idx)
{
    port->setDataBits((DataBitsType)ui->dataBitsBox->itemData(idx).toInt());
}

void Dialog::onStopBitsChanged(int idx)
{
    port->setStopBits((StopBitsType)ui->stopBitsBox->itemData(idx).toInt());
}

void Dialog::onQueryModeChanged(int idx)
{
    port->setQueryMode((QextSerialPort::QueryMode)ui->queryModeBox->itemData(idx).toInt());
}

void Dialog::onTimeoutChanged(int val)
{
    port->setTimeout(val);
}
//! [2]
//! [3]
void Dialog::onOpenCloseButtonClicked()
{
    if (!port->isOpen())
    {
        if(ui->portBox->currentText() != NULL )
        {
            port->setPortName(ui->portBox->currentText());
            port->open(QIODevice::ReadWrite);
        }
        else
        {
        }
    }
    else
    {
        port->close();
    }

    //If using polling mode, we need a QTimer
    if (port->isOpen() && port->queryMode() == QextSerialPort::Polling)
        timer->start();
    else
        timer->stop();
}
//! [3]
//! [4]

void Dialog::onReadyRead()
{
    /*if (port->bytesAvailable()) {
        QByteArray ba = port->readAll();
        ui->textEdit_Log->moveCursor(QTextCursor::End);
        ui->textEdit_Log->insertPlainText(QString::fromLatin1(ba));
    }*/

    receiveFlag = true;
}

void Dialog::onClockLabelUpdate()
{
    QDateTime local(QDateTime::currentDateTime());
    ui->label_13->setText(local.toString("hh:mm:ss A"));
    //onTextBoxLogPrint("test\r\n");
}

void Dialog::onPortAddedOrRemoved()
{
    QString current = ui->portBox->currentText();

    ui->portBox->blockSignals(true);
    ui->portBox->clear();
    foreach (QextPortInfo info, QextSerialEnumerator::getPorts())
        ui->portBox->addItem(info.portName);

    ui->portBox->setCurrentIndex(ui->portBox->findText(current));

    ui->portBox->blockSignals(false);
}


void Dialog::msg_send(uint8_t chan, mavlink_message_t *p_msg)
{
  uint8_t  buf[1024];
  uint16_t len;
  uint16_t write_len;
QString textPrint;

  len = mavlink_msg_to_send_buffer(buf, p_msg);

  switch(chan)
  {
    case 0:
      write_len = write_bytes((char *)buf, (uint32_t)len);
      break;

    case 1:
      break;
  }

  //ui->led_Tx->turnOff();
}


BOOL Dialog::msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status )
{
  BOOL ret = FALSE;
  if(chan == 0)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, data, p_msg, p_status) == MAVLINK_FRAMING_OK)
    {
      ret = TRUE;
    }
  }
  else
  {
    if (mavlink_parse_char(MAVLINK_COMM_1, data, p_msg, p_status) == MAVLINK_FRAMING_OK)
    {
      ret = TRUE;
    }
  }

  return ret;
}


BOOL Dialog::msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout)
{
  BOOL ret = FALSE;
  int  ch_ret;
  uint8_t ch;
  static mavlink_message_t msg[MSG_CH_MAX];
  static mavlink_status_t status[MSG_CH_MAX];
  uint32_t retry = timeout;
    int  data_cnt;
 int  time_out_cnt;

    port->waitForReadyRead(timeout);

    if(receiveFlag == true && port->bytesAvailable())
    {
        QByteArray ch_ret = port->readAll();
        for(data_cnt=0;data_cnt< ch_ret.length();data_cnt++)
        {
           ch = (int)ch_ret.at(data_cnt) ;

            ret = msg_recv( chan, ch, &msg[chan], &status[chan] );

            if( ret == TRUE )
            {
                *p_msg = msg[chan];
                break;
            }
        }
    }

    for(time_out_cnt=0;time_out_cnt<10;time_out_cnt++)
    {
        if( ret != TRUE)
        {
           // port->waitForReadyRead(10000);
            QThread::sleep(1);
            if(port->bytesAvailable())
            {
                QByteArray ch_ret = port->readAll();
                for(data_cnt=0;data_cnt< ch_ret.length();data_cnt++)
                {
                   ch = (int)ch_ret.at(data_cnt) ;

                    ret = msg_recv( chan, ch, &msg[chan], &status[chan] );

                    if( ret == TRUE )
                    {
                        *p_msg = msg[chan];
                        break;
                    }
                }
            }
       }
       else
       {
            break;
       }
    }
    receiveFlag =false;

   // ui->led_Rx->g->turnOff();
    return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
---------------------------------------------------------------------------*/
void Dialog::ser_set_timeout_ms(long val )
{
    port->setTimeout(val);

    return;
}

/*---------------------------------------------------------------------------
     TITLE   : read_byte
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::read_byte( void )
{
    //return ser_read_byte( stm32_ser_id );
    char byte;

    port->read(&byte,1);

    return (int)byte;
}


/*---------------------------------------------------------------------------
     TITLE   : write_bytes
     WORK    :
---------------------------------------------------------------------------*/
int Dialog::write_bytes( char *p_data, int len )
{
    int written_len;

    //written_len = ser_write( stm32_ser_id, (const u8 *)p_data, len );
    written_len = port->write(p_data,len);

    return written_len;
}

long Dialog::iclock()
{
   // struct timeval tv;
   // gettimeofday (&tv, NULL);
   // return (tv.tv_sec * 1000 + tv.tv_usec / 1000);

    QDateTime local(QDateTime::currentDateTime());
    return local.toMSecsSinceEpoch();

}
