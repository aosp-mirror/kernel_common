/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



 void file_close(struct file* file);
 int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
 int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
 struct file* file_open(const char* path, int flags, int rights);

 