/*
	Copyright 2012-2015 OROCA ESC Project 	www.oroca.org

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * mavlink_proc.c
 *
 *  Created on: 18 apr 2014
 *      Author: bakchajang
 */


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <math.h>
//#include <chthreads.h>
//#include <Chvt.h>

#include "timeout.h"
#include "utils.h"

#include "comm_usb.h"
#include "comm_usb_serial.h"

#include "mc_interface.h"
#include "mavlink_proc.h"

const uint8_t  *board_name   = "BLDC R10";
uint32_t boot_version        = 0x17020800;
uint32_t boot_revision       = 0x00000000;

#define MSG_CH_MAX	1
msg_handle_t	gMsg;


/*---------------------------------------------------------------------------
     TITLE   : resp_ack
     WORK    :
---------------------------------------------------------------------------*/
void resp_ack( uint8_t ch, mavlink_ack_t *p_ack )
{
  mavlink_message_t mav_msg;

  mavlink_msg_ack_pack_chan(0, 0, ch, &mav_msg, p_ack->msg_id, p_ack->err_code, p_ack->length, p_ack->data);

  mavlink_msg_send( ch, &mav_msg);
}

/*---------------------------------------------------------------------------
     TITLE   : cmd_send_error
     WORK    :
---------------------------------------------------------------------------*/
void cmd_send_error( msg_handle_t *p_msg, err_code_t err_code )
{

  mavlink_ack_t     mav_ack;
  mavlink_read_version_t mav_data;


  mavlink_msg_read_version_decode(p_msg->p_msg, &mav_data);

  mav_ack.msg_id   = p_msg->p_msg->msgid;
  mav_ack.err_code = err_code;
  resp_ack(p_msg->ch, &mav_ack);
}



/*---------------------------------------------------------------------------
     TITLE   : cmd_read_version
     WORK    :
---------------------------------------------------------------------------*/
void cmd_read_version( msg_handle_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_version_t mav_data;

  mavlink_msg_read_version_decode(p_msg->p_msg, &mav_data);

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;
    mav_ack.data[0] = boot_version;
    mav_ack.data[1] = boot_version>>8;
    mav_ack.data[2] = boot_version>>16;
    mav_ack.data[3] = boot_version>>24;
    mav_ack.data[4] = boot_revision;
    mav_ack.data[5] = boot_revision>>8;
    mav_ack.data[6] = boot_revision>>16;
    mav_ack.data[7] = boot_revision>>24;
    mav_ack.length  = 8;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_board_name
     WORK    :
---------------------------------------------------------------------------*/
void cmd_read_board_name( msg_handle_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_board_name_t mav_data;
  uint8_t i;

  mavlink_msg_read_board_name_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;


    for( i=0; i<strlen(board_name); i++ )
    {
      mav_ack.data[i] = board_name[i];
    }
    mav_ack.data[i] = 0;
    mav_ack.length  = i;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : cmd_read_tag
     WORK    :
---------------------------------------------------------------------------*/
void cmd_read_tag( msg_handle_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_tag_t mav_data;

  mavlink_msg_read_tag_decode(p_msg->p_msg, &mav_data);



  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;

    mav_ack.length  = 0;
    resp_ack(p_msg->ch, &mav_ack);
  }
}



int cmd_velocity( uint16_t data )
{
    mavlink_message_t msg; 

  //  mavlink_msg_set_velocity_pack( 9, 121, &msg, data );

   // mavlink_bytes_send( &msg);

    return 0;
}



void cmd_set_mcconf( msg_handle_t *p_msg )
{
	err_code_t err_code = OK;
	mavlink_ack_t     mav_ack;
	mavlink_set_mcconf_t mav_data;
	mc_configuration mcconf;

	mavlink_msg_set_mcconf_decode(p_msg->p_msg,&mav_data);

	if( mav_data.resp == 1 )
	{
		mav_ack.msg_id   = p_msg->p_msg->msgid;
		mav_ack.err_code = err_code;

		mav_ack.length  = 0;
		resp_ack(p_msg->ch, &mav_ack);
	}

	mcconf = *mc_interface_get_configuration();


	mcconf.vdd 		= mav_data.uVDD; /*< x10^1 */
	mcconf.rshunt 	= mav_data.uRSHUNT; /*< x10^3 */
	mcconf.pwmFreq	= mav_data.uPWMFREQUENCY; /*< x10^3 */
	mcconf.dkp 		= mav_data.uDKP; /*< x10^3 */
	mcconf.dki		= mav_data.uDKI; /*< x10^3 */
	mcconf.dkc		= mav_data.uDKC; /*< x10^3 */
	mcconf.dout_max	= mav_data.uDOUTMAX; /*< x10^3 */
	mcconf.qkp		= mav_data.uQKP; /*< x10^3 */
	mcconf.qki		= mav_data.uQKI; /*< x10^3 */
	mcconf.qkc		= mav_data.uQKC; /*< x10^3 */
	mcconf.qout_max	= mav_data.uQOUTMAX; /*< x10^3 */
	mcconf.wkp		= mav_data.uWKP; /*< x10^3 */
	mcconf.wki		= mav_data.uWKI; /*< x10^3 */
	mcconf.wkc		= mav_data.uWKC; /*< x10^3 */
	mcconf.wout_max	= mav_data.uWOUTMAX; /*< x10^3 */
	mcconf.pll_kp	= mav_data.uPLLKP; /*< x10^3 */
	mcconf.pll_ki	= mav_data.uPLLKI; /*< x10^3 */
	mcconf.pll_kc	= mav_data.uPLLKC; /*< x10^3 */
	mcconf.pllout_max= mav_data.uPLLOUTMAX; /*< x10^3 */

mc_interface_set_configuration(&mcconf);
chThdSleepMilliseconds(200);


}

void cmd_set_appconf( msg_handle_t *p_msg )
{
  err_code_t err_code = OK;
  mavlink_ack_t     mav_ack;
  mavlink_read_version_t mav_data;

  mavlink_msg_read_version_decode(p_msg->p_msg, &mav_data);

  if( mav_data.resp == 1 )
  {
    mav_ack.msg_id   = p_msg->p_msg->msgid;
    mav_ack.err_code = err_code;

    mav_ack.length  = 0;
    resp_ack(p_msg->ch, &mav_ack);
  }
}


void mavlink_dbgString( uint8_t ch, char* dbg_str )
{
    mavlink_message_t mav_msg;

	mavlink_msg_debug_string_pack_chan(0, 0, ch, &mav_msg, dbg_str);

    mavlink_msg_send( ch, &mav_msg);
}

//------------------------------------------------------------------------------------------
// CMD func
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// MAVLink
//------------------------------------------------------------------------------------------



void mavlink_msg_send(uint8_t ch, mavlink_message_t *p_msg)

{
	uint8_t buf[1024];	  

	int len = mavlink_msg_to_send_buffer(buf, p_msg);
	
	switch(ch)
	{
	  case 0:		usb_serial_send(buf,len);		break;
	  case 1:		break;
	}
	
	return;
}


bool mavlink_msg_recv( uint8_t ch, uint8_t data , msg_handle_t *p_msg )
{
	bool ret = FALSE;
	static mavlink_message_t msg[MSG_CH_MAX];
	mavlink_status_t status[MSG_CH_MAX];

	p_msg->ch = ch;

	if(ch == 0)
	{
		if (mavlink_parse_char(MAVLINK_COMM_0, data, &msg[ch], &status[ch]) == MAVLINK_FRAMING_OK)
		{
			p_msg->p_msg = &msg[ch];
			ret = TRUE;
		}
	}
	else
	{
		if (mavlink_parse_char(MAVLINK_COMM_1, data, &msg[ch], &status[ch]) == MAVLINK_FRAMING_OK)
		{
			p_msg->p_msg = &msg[ch];
			ret = TRUE;
		}
	}
	return ret;

}



void  mavlink_msg_process_vcp( msg_handle_t* p_msg)
{

      switch( p_msg->p_msg->msgid )
      {
		case MAVLINK_MSG_ID_READ_VERSION:		cmd_read_version(p_msg);					break;
		case MAVLINK_MSG_ID_READ_BOARD_NAME:	cmd_read_board_name(p_msg);					break;
		case MAVLINK_MSG_ID_READ_TAG:			cmd_read_tag(p_msg);						break;
		case MAVLINK_MSG_ID_SET_MCCONF: 		cmd_set_mcconf(p_msg); 						break;
		case MAVLINK_MSG_ID_SET_APPCONF : 		cmd_set_appconf(p_msg); 					break;
		default:								cmd_send_error(p_msg, ERR_INVALID_CMD);		break;
      }

}

