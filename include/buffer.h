/*
 * DaisySound - audio coprocessor firmware for the Daisy computer.
 * Copyright (C) 2026 Joe Cassara
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_BUFFER_H_
#define INCLUDE_BUFFER_H_
#include <stdint.h>
#include <stdbool.h>

#define kBufferMaxLen 256

typedef struct {
  uint8_t buffer[kBufferMaxLen];
  uint8_t buffer_len;
  uint8_t read_index;
  uint32_t timestamp;
  bool in_frame;
} Buffer;

void BufferInit(Buffer* buffer);
void BufferAdd(uint8_t b, Buffer* buffer);
bool BufferHasBytes(Buffer* buffer);
bool BufferIsFull(Buffer* buffer);

#endif  // INCLUDE_BUFFER_H_
