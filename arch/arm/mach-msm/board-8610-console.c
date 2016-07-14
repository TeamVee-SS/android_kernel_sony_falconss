/* arch/arm/mach-msm/board-8610-console.c
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 * Copyright (C) 2016 Caio Oliveira <caiooliveirafarias0@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/string.h>
#include <ram_console.h>
#include <asm/setup.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static char bootreason[128];

int __init msm_boot_reason(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(bootreason, sizeof(bootreason),
		 "Boot info:\n"
		 "Last boot reason: %s\n", s);
	bootreason[n] = '\0';
	return 1;
}
__setup("bootreason", msm_boot_reason);

struct ram_console_platform_data ram_console_pdata = {
	.bootinfo = bootreason,
};
#endif
