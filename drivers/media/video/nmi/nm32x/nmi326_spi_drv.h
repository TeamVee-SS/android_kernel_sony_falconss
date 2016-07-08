/*****************************************************************************
 Copyright(c) 2012 NMI Inc. All Rights Reserved

 File name : nmi326_spi_drv.h

 Description : NM326 SPI interface

 History :
 ----------------------------------------------------------------------
 2012/11/27 	ssw		initial
*******************************************************************************/

#ifndef __SPI_DRV_H
#define __SPI_DRV_H

#define CONFIG_VIDEO_SPI_DEBUG
#define NMI326_HW_CHIP_ID_CHECK

/* Debug macro */
#define SPI_DEBUG(fmt, ...)                                     \
	do {                                                    \
	        pr_info("%s: " fmt, __func__, ##__VA_ARGS__);   \
	} while(0)

#ifdef CONFIG_VIDEO_SPI_DEBUG
#define spi_dbg(fmt, ...)               SPI_DEBUG(fmt, ##__VA_ARGS__)
#else
#define spi_dbg(fmt, ...)
#endif

unsigned long check_dtv_chip_id(void);
unsigned long nmi326_spi_read_chip_id(void);

int nmi326_spi_read(u8 *buf, size_t len);
int nmi326_spi_write(u8 *buf, size_t len);

#endif	//__SPI_DRV_H
