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

#ifndef __MAVLINK_UART_PROC_H__
#define __MAVLINK_UART_PROC_H__

#include <chtypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EVT_UART_RX EVENT_MASK(1)

void mavlink_uart_proc_configure(void);
void mavlink_uart_proc_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
