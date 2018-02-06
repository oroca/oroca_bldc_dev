/*
	Copyright 2012-2014 OROCA ESC Project 	www.oroca.org

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
 * app.h
 *
 *  Created on: 18 apr 2014
 *      Author: bakchajang
 */

#ifndef __MAVLINK_PROC_H__
#define __MAVLINK_PROC_H__

#include <chtypes.h>

#include "../mavlink/oroca_bldc/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t ch;
  mavlink_message_t *p_msg;
} msg_handle_t;

extern msg_handle_t	gMsg;

typedef uint16_t err_code_t;

#define OK                                  0x0000
#define ERR_INVALID_CMD                     0x0001


void mavlink_dbgString( uint8_t ch, char* dbg_str );

void mavlink_msg_send(uint8_t ch, mavlink_message_t *p_msg);
bool mavlink_msg_recv( uint8_t ch, uint8_t data , msg_handle_t *p_msg );
void  mavlink_msg_process_vcp( msg_handle_t* p_msg);


#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
