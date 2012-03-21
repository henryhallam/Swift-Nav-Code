/*
 * Copyright (C) 2012 Colin Beighley <colinbeighley@gmail.com>
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

#ifndef SWIFTNAV_CW_H
#define SWIFTNAV_CW_H

#include <libopencm3/cm3/common.h>

#define SPECTRUM_LEN 301
//#define SPECTRUM_LEN 501

typedef enum {
  CW_DISABLED = 0,
  CW_LOADING,
  CW_LOADING_DONE,
  CW_RUNNING,
  CW_RUNNING_DONE
} cw_status_t;

typedef struct {
  cw_status_t state;
  s32 cf_step, cf_min, cf_max;
  s32 carrier_freq;
  u64 power_acc;
  u64 best_power;
  s32 best_freq;
	u16 count;
	u64 spectrum_power[SPECTRUM_LEN];
	s32 spectrum_freq[SPECTRUM_LEN];
} cw_state_t;

void cw_schedule_load(u32 count);
void cw_service_load_done();
u8 cw_get_load_done();
u8 cw_get_running_done();

void cw_start(float cf_min, float cf_max, float cf_bin_width);
void cw_service_irq();
void cw_get_results(float* cf, float* snr);
void cw_get_spectrum_point(float* freq, u64* power, u16 index);
//void cw_get_spectrum_point(float* freq, float* power, u16 index);
void do_one_cw(s32 carrier_freq, corr_t* corrs);

#endif
