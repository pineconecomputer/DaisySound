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

#include "buffer.h"
#include <string.h>

// Clears a frame buffer back to its empty, not-yet-receiving state.
void BufferInit(Buffer* buffer) {
  buffer->in_frame = false;
  buffer->buffer_len = 0;
  buffer->timestamp = 0;
  memset(buffer->buffer, 0x00, kBufferMaxLen);
}

// Appends one received byte. Silently drops bytes once full, so a malformed
// frame that never terminates cannot overrun the buffer.
void BufferAdd(uint8_t b, Buffer* buffer) {
  if (buffer->buffer_len < kBufferMaxLen) {
    buffer->buffer[buffer->buffer_len] = b;
    buffer->buffer_len++;
  }
}

// True if any bytes have been accumulated.
bool BufferHasBytes(Buffer* buffer) { return (buffer->buffer_len != 0); }

// True once the buffer can accept no more bytes; the frame is over-long.
bool BufferIsFull(Buffer* buffer) {
  return (buffer->buffer_len == kBufferMaxLen);
}
