/* Copyright (C) 2016 Caio Oliveira <caiooliveirafarias0@gmail.com>
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

#include <asm/setup.h>
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <ram_console.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static char bootreason[128];

int __init msm_boot_reason(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(bootreason, sizeof(bootreason), "Boot info:\n"
						     "Last boot reason: %s\n",
		     s);
	bootreason[n] = '\0';
	return 1;
}
__setup("bootreason", msm_boot_reason);

struct ram_console_platform_data ram_console_pdata = {
    .bootinfo = bootreason,
};
#endif
