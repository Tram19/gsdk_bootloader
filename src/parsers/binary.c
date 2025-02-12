/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

#include "binary.h"


void* binary_init() {
	return calloc(sizeof(binary_t), 1);
}

parser_err_t binary_open(void *storage, const char *filename, const char write) {
	binary_t *st = (binary_t *)storage;
	if (write) {
		if (filename[0] == '-')
			st->fd = 1;
		else
			st->fd = open(
				filename,
#ifndef __WIN32__
				O_WRONLY | O_CREAT | O_TRUNC,
#else
				O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
#endif
#ifndef __WIN32__
				S_IRUSR  | S_IWUSR | S_IRGRP | S_IROTH
#else
				0
#endif
			);
		st->stat.st_size = 0;
	} else {
		if (filename[0] == '-') {
			st->fd = 0;
		} else {
			if (stat(filename, &st->stat) != 0)
				return PARSER_ERR_INVALID_FILE;
			st->fd = open(filename,
#ifndef __WIN32__
				O_RDONLY
#else
				O_RDONLY | O_BINARY
#endif
			);
		}
	}

	st->write = write;
	return st->fd == -1 ? PARSER_ERR_SYSTEM : PARSER_ERR_OK;
}

parser_err_t binary_close(void *storage) {
	binary_t *st = (binary_t *)storage;

	if (st->fd) close(st->fd);
	free(st);
	return PARSER_ERR_OK;
}

unsigned int binary_size(void *storage) {
	binary_t *st = (binary_t *)storage;
	return st->stat.st_size;
}

parser_err_t binary_read(void *storage, void *data, unsigned int *len) {
	binary_t *st = (binary_t *)storage;
	unsigned int left = *len;
	if (st->write) return PARSER_ERR_WRONLY;

	ssize_t r;
	while(left > 0) {
		r = read(st->fd, data, left);
		/* If there is no data to read at all, return OK, but with zero read */
		if (r == 0 && left == *len) {
			*len = 0;
			return PARSER_ERR_OK;
		}
		if (r <= 0) return PARSER_ERR_SYSTEM;
		left -= r;
		data += r;
	}

	*len = *len - left;
	return PARSER_ERR_OK;
}

parser_err_t binary_write(void *storage, void *data, unsigned int len) {
	binary_t *st = (binary_t *)storage;
	if (!st->write) return PARSER_ERR_RDONLY;

	ssize_t r;
	while(len > 0) {
		r = write(st->fd, data, len);
		if (r < 1) return PARSER_ERR_SYSTEM;
		st->stat.st_size += r;

		len  -= r;
		data += r;
	}

	return PARSER_ERR_OK;
}

parser_err_t binary_crc32(void *storage, uint32_t *crc32)
{
	binary_t *st = (binary_t *)storage;
	if (st->write) return PARSER_ERR_WRONLY;

	if (lseek(st->fd, 0, SEEK_SET) == (off_t)-1) {
		perror("lseek error");
	}
	*crc32 = 0xFFFFFFFF;
	uint32_t offset = 0;
	uint32_t total_size = (uint32_t)st->stat.st_size;
	printf("Total size: %d\n", total_size);
	uint16_t max_data_length = 1024;
	unsigned char buf[1024];
	while (offset < total_size)
	{
		uint32_t left	= total_size - offset;
        uint32_t len 	= max_data_length > left ? left : max_data_length;
        uint32_t len_t 	= len;

		if(binary_read(storage,buf,&len) != PARSER_ERR_OK)
		{
			return PARSER_ERR_SYSTEM;
		}
        if (len == 0 && offset <= total_size)
        {
            return PARSER_ERR_SYSTEM;
        }
		*crc32 = xcrc32(buf, len, *crc32);
		offset += len;
	}
	printf("Offset: %d\n", offset);
	if (lseek(st->fd, 0, SEEK_SET) == (off_t)-1) {
		perror("lseek error");
	}
	*crc32 = ~*crc32;
	return PARSER_ERR_OK;
}