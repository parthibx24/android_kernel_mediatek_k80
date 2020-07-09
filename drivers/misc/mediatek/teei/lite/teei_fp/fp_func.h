/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef TEEI_FUNC_H
#define TEEI_FUNC_H

#define FP_LIMIT_SIZE	0x80000
#define FP_MAJOR		254
#define DEV_NAME		"teei_fp"

#define FINGERPRINT_PORT "com.android.trusty.fingerprint"

#define FINGERPRINT_MAX_BUFFER_LENGTH 4096
#define MAX_MESSAGE_SIZE	(4096 - 64)

#define TEEI_IOC_MAGIC 'T'
#define CMD_FP_MEM_CLEAR	_IO(TEEI_IOC_MAGIC, 0x1)
#define CMD_FP_CMD			_IO(TEEI_IOC_MAGIC, 0x2)

enum fingerprint_command {
	FP_RESP_BIT = 1,
	FP_REQ_BIT = 1,
	FP_STOP_BIT = 2,
	FP_REQ_SHIFT = 2,
	FP_REQ = (0 << FP_REQ_SHIFT),
};

struct iovec {
	void *iov_base;
	size_t iov_len;
};

#define iovec_t struct iovec

struct fingerprint_message {
	uint32_t cmd;
	uint8_t payload[0];
};

struct TEEC_UUID {
	uint32_t timeLow;
	uint16_t timeMid;
	uint16_t timeHiAndVersion;
	uint8_t clockSeqAndNode[8];
};

extern struct TEEC_UUID uuid_fp;
extern int tipc_k_connect(tipc_k_handle *h, const char *port);
extern ssize_t tipc_k_write(tipc_k_handle h, void *buf,
				size_t len, unsigned int flags);
extern ssize_t tipc_k_read(tipc_k_handle h, void *buf, size_t buf_len,
				unsigned int flags);
extern int tipc_k_disconnect(tipc_k_handle h);

#endif /* end of TEEI_FUNC_H */
