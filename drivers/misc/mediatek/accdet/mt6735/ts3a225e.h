/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
/*
 * Definitions for TS3A225E Audio Switch chip.
 */
#ifndef __TS3A225E_H__
#define __TS3A225E_H__

int ts3a225e_read_byte(unsigned char cmd, unsigned char *returnData);
int ts3a225e_write_byte(unsigned char cmd, unsigned char writeData);

#endif
