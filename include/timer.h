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

#ifndef INCLUDE_TIMER_H_
#define INCLUDE_TIMER_H_
#include <Arduino.h>
#include <stdbool.h>

typedef struct {
  bool is_enabled;
  bool is_done;
  uint64_t deadline_ms;
  uint64_t birthtime_ms;
} Timer;

bool TimerIsDone(Timer* timer);
void TimerCreate(Timer* timer, uint64_t deadline_ms);
void TimerReset(Timer* timer);
void TimerDisable(Timer* timer);

#endif  // INCLUDE_TIMER_H_
