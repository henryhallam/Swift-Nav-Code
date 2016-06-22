/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap_conf.h"
#include "nap_common.h"
#include "track_channel.h"

#include "libswiftnav/prns.h"

/** \addtogroup nap
 * \{ */

/** \defgroup track_channel Track Channel
 * Interface to the SwiftNAP track channels.
 * \{ */

/* SPI register IDs */
#define NAP_REG_TRACK_BASE           0x0A
#define NAP_TRACK_N_REGS             3
#define NAP_REG_TRACK_CODE_OFFSET    0x00
#define NAP_REG_TRACK_UPDATE_OFFSET  0x01
#define NAP_REG_TRACK_CORR_OFFSET    0x02


/** Number of tracking channels.
 * Actual number of track channels NAP configuration was built with. Read from
 * configuration flash at runtime with nap_conf_rd_parameters().
 */
u8 nap_track_n_channels;

/** Pack data for writing to a NAP track channel's UPDATE register.
 *
 * \param pack            Array of u8 to pack data into.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 */
void nap_track_update_pack(u8 pack[], s32 carrier_freq, u32 code_phase_rate,
                           u8 rollover_count)
{
  pack[0] = (code_phase_rate & 0x80000000) >> 31;
  pack[1] = (code_phase_rate & 0x7F800000) >> 23;
  pack[2] = (code_phase_rate & 0x007F8000) >> 15;
  pack[3] = (code_phase_rate & 0x00007F80) >> 7;
  pack[4] = ((code_phase_rate & 0x0000007F) << 1) | ((carrier_freq >> 16) & 0x01);
  pack[5] = carrier_freq >> 8;
  pack[6] = carrier_freq;
  pack[7] = rollover_count;
}

/** Write to a NAP track channel's UPDATE register.
 * Write new carrier frequency and code phase rate to a NAP track channel's
 * UPDATE register, which will be used to accumulate the channel's carrier and
 * code phases during the next correlation period.
 *
 * \note This must be called in addition to nap_track_init_wr_blocking when a
 *       new track channel is being set up, before the NAP's internal timing
 *       strobe goes low.
 *
 * \note If two track channel IRQ's occur without a write to the tracking
 *       channel's UPDATE register between them, the error bit for the track
 *       channel in the NAP error register will go high.
 *
 * \param channel         NAP track channel whose UPDATE register to write.
 * \param carrier_freq    Next correlation period's carrier frequency.
 * \param code_phase_rate Next correlation period's code phase rate.
 */
void nap_track_update_wr_blocking(u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count)
{
  u8 temp[8] = { 0 };

  nap_track_update_pack(temp, carrier_freq, code_phase_rate,
                        rollover_count);
  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_UPDATE_OFFSET, 8, 0, temp);
}
/** Unpack data read from a NAP track channel's CORR register.
 *
 * \param packed       Array of u8 data read from channnel's CORR register.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void nap_track_corr_unpack(u8 packed[], u32* sample_count, corr_t corrs[])
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */

  struct { s32 xtend : 24; } sign;

  *sample_count = (packed[0] << 16) | (packed[1] << 8) | packed[2];

  for (u8 i = 0; i < 5; i++) {

    sign.xtend  = (packed[6 * (5 - i - 1) + 3] << 16) /* MSB */
                | (packed[6 * (5 - i - 1) + 4] << 8)  /* Middle byte */
                | (packed[6 * (5 - i - 1) + 5]);      /* LSB */

    corrs[i].Q = sign.xtend;  /* Sign extend! */

    sign.xtend  = (packed[6 * (5 - i - 1) + 6] << 16) /* MSB */
                | (packed[6 * (5 - i - 1) + 7] << 8)  /* Middle byte */
                | (packed[6 * (5 - i - 1) + 8]);      /* LSB */

    corrs[i].I = sign.xtend;  /* Sign extend! */
  }
}

/** Read data from a NAP track channel's CORR register.
 *
 * \param channel      NAP track channel whose CORR register to read.
 * \param sample_count Number of sample clock cycles in correlation period.
 * \param corrs        Array of E,P,L correlations from correlation period.
 */
void nap_track_corr_rd_blocking(u8 channel, u32* sample_count, corr_t corrs[])
{
  /* 2 (I or Q) * 3 (E, P or L) * 3 (24 bits / 8) + 24 bits sample count. */
  u8 temp[2*5*3 + 3];

  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_CORR_OFFSET, 2*5*3 + 3, temp, temp);
  nap_track_corr_unpack(temp, sample_count, corrs);
}


/** Write CA code to track channel's code ram.
 * CA code for SV to be tracked must be written into channel's code ram
 * before tracking is started.
 *
 * \param prn PRN number (0-31) of CA code to be written.
 */
void nap_track_code_wr_blocking(u8 channel, gnss_signal_t sid)
{
  nap_xfer_blocking(NAP_REG_TRACK_BASE + channel * NAP_TRACK_N_REGS
                     + NAP_REG_TRACK_CODE_OFFSET, 128, 0, ca_code(sid));
}

/** \} */

/** \} */

