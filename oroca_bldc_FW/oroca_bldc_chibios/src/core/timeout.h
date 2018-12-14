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
 * timeout.h
 *
 *  Created on: 20 sep 2014
 *      Author: bakchajang
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "chtypes.h"

// Functions
void timeout_init(void);
void timeout_configure(systime_t timeout);
void timeout_reset(void);
bool timeout_has_timeout(void);
systime_t timeout_get_timeout_msec(void);

#endif /* TIMEOUT_H_ */
