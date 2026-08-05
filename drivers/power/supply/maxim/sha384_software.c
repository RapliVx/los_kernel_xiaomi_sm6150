/*
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 */

/* SHA3_HMAC - HMAC using SHA3-256 */
#include <linux/slab.h>
#include <linux/string.h>

#include "ucl_sha3.h"

#define SHA3_256_HMAC
#include "sha384_software.h"

struct sha3_buffers {
	unsigned char thash[256];
	unsigned char tmac[256];
	unsigned char cat_input_thash[1024];
	unsigned char cat_input_final[1024];
	unsigned char opad[136];
	unsigned char ipad[136];
};

/**
 * sha3_256_hmac() - Compute HMAC using SHA3-256.
 * @key:	buffer for key
 * @key_len:	length of key
 * @message:	buffer for message
 * @msg_len:	length of message
 * @mac:	32 byte output mac
 *
 * Restrictions:
 * Key length limited to 32 bytes.
 * Message Length is limited to 512 bytes.
 *
 * Return: 1 on command successful, 0 on command failed.
 */
int sha3_256_hmac(unsigned char *key, int key_len, unsigned char *message,
		  int msg_len, unsigned char *mac)
{
	struct sha3_buffers *bufs;
	const int blocksize = 136;
	const int hashsize = 32;
	int i;

	/* Check to see if key is larger than blocksize */
	if (key_len > blocksize)
		return 0;

	/* Check for blocks too big */
	if (msg_len > 512)
		return 0;

	bufs = kzalloc(sizeof(*bufs), GFP_KERNEL);
	if (!bufs)
		return 0;

	memset(bufs->opad, 0x5C, blocksize);
	memset(bufs->ipad, 0x36, blocksize);

	/* Loop through bytes of ipad/opad and XOR with key */
	for (i = 0; i < key_len; i++) {
		bufs->ipad[i] ^= key[i];
		bufs->opad[i] ^= key[i];
	}

	/* thash = hash(ipad || message) */
	memcpy(bufs->cat_input_thash, bufs->ipad, blocksize);
	memcpy(&bufs->cat_input_thash[blocksize], message, msg_len);

	ucl_sha3_256(bufs->thash, bufs->cat_input_thash, blocksize + msg_len);

	/* Return hash(opad || thash) */
	memcpy(bufs->cat_input_final, bufs->opad, blocksize);
	memcpy(&bufs->cat_input_final[blocksize], bufs->thash, hashsize);

	ucl_sha3_256(bufs->tmac, bufs->cat_input_final, blocksize + hashsize);

	memcpy(mac, bufs->tmac, hashsize);

	kfree(bufs);

	return 1;
}
