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

#ifndef SWIFTNAV_DEBUG_H
#define SWIFTNAV_DEBUG_H

#include <libopencm3/cm3/common.h>

#define MSG_PRINT 0x01

#define DEBUG_MAGIC_1 0xBE
#define DEBUG_MAGIC_2 0xEF

#define DEBUG_MSG(msg_type, item) send_debug_msg(msg_type, sizeof(item), (u8*)&(item))

/* Define the type of our callback function
 * for convenience.
 */
typedef void(*msg_callback_t)(u8 msg[]);

/* Define a linked list of message callbacks. */
typedef struct msg_callbacks_node {
  u8 msg_type;
  msg_callback_t cb;
  struct msg_callbacks_node* next;
} msg_callbacks_node_t;

void debug_setup();
void debug_send_msg(u8 msg_type, u8 len, u8 buff[]);
void debug_register_callback(u8 msg_type, msg_callback_t cb, msg_callbacks_node_t* node);
msg_callback_t debug_find_callback(u8 msg_id);
void debug_process_messages();

void screaming_death();
void speaking_death(char *msg);

#endif
