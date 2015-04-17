/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SPI_H
#define SWIFTNAV_SPI_H

#include <libswiftnav/common.h>
#include <libopencm3/stm32/spi.h>

#define dma2_stream0_isr Vector120
#define dma2_stream3_isr Vector12C

/** \addtogroup spi
 * \{ */

#define SPI_SLAVE_FPGA     0x01 /**< SwiftNAP FPGA */
#define SPI_SLAVE_FLASH    0x02 /**< M25 configuration flash */
#define SPI_SLAVE_FRONTEND 0x03 /**< MAX2769 front-end */
#define SPI_SLAVE_FRONTEND_ALT 0x04

#define SPI_BUS_FLASH    SPI2 /**< SPI bus that the M25 flash is on. */
#define SPI_BUS_FPGA     SPI1 /**< SPI bus that the FPGA is on. */
#define SPI_BUS_FRONTEND SPI2 /**< SPI bus that the MAX2769 is on. */
#define SPI_BUS_FRONTEND_ALT SPI1

/** \} */

void spi_setup(void);
void spi_deactivate(void);
void spi_slave_select(u8 slave);
void spi_slave_deselect(void);
void spi1_dma_setup(void);
void spi1_xfer_dma(u16 n_bytes, u8 data_in[], const u8 data_out[]);

#endif

