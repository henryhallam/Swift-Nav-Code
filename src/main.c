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

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <libopencm3/stm32/f2/rcc.h>
#include <libopencm3/stm32/f2/dma.h>
#include <libopencm3/stm32/f2/flash.h>
#include <libopencm3/stm32/f2/gpio.h>
#include <libopencm3/stm32/f2/timer.h>

#include "main.h"
#include "debug.h"
#include "swift_nap_io.h"
#include "hw/leds.h"
#include "hw/spi.h"

extern u8 it_wrapped;

const clock_scale_t hse_16_368MHz_in_130_944MHz_out_3v3 =
{ /* 130.944 MHz (Overclocked!!) */
  .pllm = 16,
  .plln = 256,
  .pllp = 2,
  .pllq = 6,
  .hpre = RCC_CFGR_HPRE_DIV_NONE,
  .ppre1 = RCC_CFGR_PPRE_DIV_8,
  .ppre2 = RCC_CFGR_PPRE_DIV_4,
  .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_3WS,
  .apb1_frequency = 16368000,
  .apb2_frequency = 2*16368000,
};



int main(void)
{
  for (u32 i = 0; i < 600000; i++)
    __asm__("nop");

  rcc_clock_setup_hse_3v3(&hse_16_368MHz_in_130_944MHz_out_3v3);
	
  led_setup();

  // Debug pins (CC1111 TX/RX)
  RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11);
  gpio_clear(GPIOC, GPIO10|GPIO11);


  debug_setup();

  printf("\n\n# Firmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");


  swift_nap_setup();
  swift_nap_reset();

  led_toggle(LED_RED);

  unsigned int spoon=0;
  

  static u8 spork[250]={[0 ... 249]='~'};
  static u8 spork2[250];

  u32 t=timing_count();
  debug_send_msg(0x22,250,spork);
  t = timing_count() - t;
  printf("250 chars took %u counts = %.1f us\n",(unsigned int)t, t/16.368);

  t=timing_count();
  memcpy(spork2,spork,250);
  t = timing_count() - t;
  printf("memcpy took %u counts = %.1f us\n",(unsigned int)t, t/16.368);

  printf("DMA2_S5CR = %08X, DMA2_S7CR = %08X\n",(unsigned int)DMA2_S5CR, (unsigned int)DMA2_S7CR);

  printf("DMA2_S5M0AR = %08X\n",(unsigned int)DMA2_S5M0AR);

  u8 already = 0;

  while(1)
  {
    //debug_process_messages();
    debug_send_msg(0xE0,4,(u8 *)&spoon);
    spoon++;    
    for (int i=4800; i; i--) __asm__("nop");

    if (it_wrapped && !already) {
      already=1;
      printf("it wrapped\n");
    }

  }

  while (1);
  
	return 0;
}

