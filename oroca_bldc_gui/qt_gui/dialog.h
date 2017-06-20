#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "./msg/def.h"

#define GET_CALC_TIME(x)	( (int)(x / 1000) + ((float)(x % 1000))/1000 )

#define FLASH_TX_BLOCK_LENGTH	(8*1024)
#define FLASH_RX_BLOCK_LENGTH	(128)
#define FLASH_PACKET_LENGTH   	128

#define MSG_CH_MAX	1

typedef struct
{
  uint8_t ch;
  mavlink_message_t *p_msg;
} msg_t;

namespace Ui {
class Dialog;
}
class QTimer;
class QextSerialPort;
class QextSerialEnumerator;

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
protected:
    void changeEvent(QEvent *e);

private slots:

    void onPortNameChanged(const QString &name);
    void onBaudRateChanged(int idx);
    void onParityChanged(int idx);
    void onDataBitsChanged(int idx);
    void onStopBitsChanged(int idx);
    void onQueryModeChanged(int idx);
    void onTimeoutChanged(int val);
    void onOpenCloseButtonClicked();

    void onReadyRead();
    void onClockLabelUpdate();
    void onPortAddedOrRemoved();

private:
    Ui::Dialog *ui;
    QTimer *timer;
    QTimer *timer_colck;
    QextSerialPort *port;
    QextSerialEnumerator *enumerator;

public:
    QStringList fileNames;
    FILE      *opencr_fp;
    uint32_t   opencr_fpsize;

    bool receiveFlag;

    void msg_send(uint8_t chan, mavlink_message_t *p_msg);
    BOOL msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status );
    BOOL msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout);

    void ser_set_timeout_ms(long val );
    int read_byte( void );
    int write_bytes( char *p_data, int len );

    long iclock();
};





#endif // DIALOG_H
