/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SWIFTNAV_SWIFT_NAP_H
#define SWIFTNAV_SWIFT_NAP_H

#include <libopencm3/cm3/common.h>

#include "main.h"

#define SPI_ID_IRQ 0x00
#define SPI_ID_ERROR 0x01
#define SPI_ID_DECEASED_COW 0xFF

#define SPI_ID_CW_BASE 0x02
#define SPI_ID_CW_INIT        (SPI_ID_CW_BASE+0x00)
#define SPI_ID_CW_LOAD_ENABLE (SPI_ID_CW_BASE+0x01)
#define SPI_ID_CW_CORR        (SPI_ID_CW_BASE+0x02)

#define SPI_ID_IIR 0x05

#define SPI_ID_ACQ_BASE 0x06
#define SPI_ID_ACQ_INIT        (SPI_ID_ACQ_BASE+0x00)
#define SPI_ID_ACQ_LOAD_ENABLE (SPI_ID_ACQ_BASE+0x01)
#define SPI_ID_ACQ_CORR        (SPI_ID_ACQ_BASE+0x02)
#define SPI_ID_ACQ_CODE        (SPI_ID_ACQ_BASE+0x03)

#define SPI_ID_TRACK_BASE   0x0A
#define TRACK_SIZE          5
#define TRACK_INIT_OFFSET   0x00
#define TRACK_UPDATE_OFFSET 0x01
#define TRACK_CORR_OFFSET   0x02
#define TRACK_PHASE_OFFSET  0x03
#define TRACK_CODE_OFFSET   0x04

#define IRQ_ACQ_DONE      (1<<31)
#define IRQ_ACQ_LOAD_DONE (1<<30)
#define IRQ_CW_DONE       (1<<29)
#define IRQ_CW_LOAD_DONE  (1<<28)
#define IRQ_TRACK_MASK    (~(IRQ_ACQ_DONE|IRQ_ACQ_LOAD_DONE|IRQ_CW_DONE|IRQ_CW_LOAD_DONE))

#define ACQ_N_TAPS 15
#define ACQ_CODE_PHASE_WIDTH 12
#define ACQ_CODE_PHASE_UNITS_PER_CHIP (1<<(ACQ_CODE_PHASE_WIDTH-10))
#define ACQ_CARRIER_FREQ_WIDTH 20
#define ACQ_CARRIER_FREQ_UNITS_PER_HZ ((1<<ACQ_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)

#define TRACK_N_CHANNELS 5
#define TRACK_INIT_CODE_PHASE_WIDTH 14
#define TRACK_INIT_CODE_PHASE_UNITS_PER_CHIP (1<<(TRACK_INIT_CODE_PHASE_WIDTH-10))
#define TRACK_CARRIER_FREQ_WIDTH 24
#define TRACK_CARRIER_FREQ_UNITS_PER_HZ ((1<<TRACK_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)
#define TRACK_CODE_PHASE_RATE_WIDTH 29
#define TRACK_NOMINAL_CODE_PHASE_RATE (1<<(TRACK_CODE_PHASE_RATE_WIDTH-1))
#define TRACK_CODE_PHASE_RATE_UNITS_PER_HZ (TRACK_NOMINAL_CODE_PHASE_RATE / 1.023e6)
#define TRACK_CODE_PHASE_UNITS_PER_CHIP ((u64)1<<32)

#define CW_CARRIER_FREQ_WIDTH 20
#define CW_CARRIER_FREQ_UNITS_PER_HZ ((1<<CW_CARRIER_FREQ_WIDTH) / (float)SAMPLE_FREQ)

typedef struct {
  s32 I;
  s32 Q;
} corr_t;

void swift_nap_setup();
void swift_nap_reset();
void swift_nap_xfer_blocking(u8 spi_id, u8 n_bytes, u8 data_in[], u8 data_out[]);

void exti_setup();
u32 last_exti_count();
void wait_for_exti();

void timing_strobe_setup();
u32 timing_count();
void timing_strobe(u32 falling_edge_count);

void spi_dma_setup();

u32 swift_nap_read_irq_blocking();
u32 swift_nap_read_error_blocking();

void acq_set_load_enable_blocking();
void acq_clear_load_enable_blocking();
void acq_write_init_blocking(u8 prn, u16 code_phase, s16 carrier_freq);
void acq_disable_blocking();
void acq_read_corr_blocking(corr_t corrs[]);
void acq_write_code_blocking(u8 prn);

void track_write_init_blocking(u8 channel, u8 prn, s32 carrier_phase, u16 code_phase);
void track_write_update_blocking(u8 channel, s32 carrier_freq, u32 code_phase_rate);
void track_read_corr_blocking(u8 channel, u16* sample_count, corr_t corrs[]);
void track_read_phase_blocking(u8 channel, u32* carrier_phase, u64* code_phase);
void track_read_corr_dma(u8 channel);
void track_write_code_blocking(u8 channel, u8 prn);

void cw_set_load_enable_blocking();
void cw_clear_load_enable_blocking();
void cw_write_init_blocking(s32 carrier_freq);
void cw_disable_blocking();
void cw_read_corr_blocking(corr_t* corrs);

#endif
