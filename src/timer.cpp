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

#include "timer.h"
#include <Arduino.h>

// Reports whether an armed timer has passed its deadline. Latches is_done and
// disarms the timer, so a single expiry is reported to exactly one caller.
bool TimerIsDone(Timer* timer) {
  static bool result;
  if (millis() >= (timer->birthtime_ms + timer->deadline_ms)) {
    timer->is_done = true;
    result = (timer->is_done) && (timer->is_enabled);
    timer->is_enabled = false;
  } else {
    result = false;
  }
  return result;
}

// Arms a timer to expire deadline_ms milliseconds from now.
void TimerCreate(Timer* timer, uint64_t deadline_ms) {
  timer->is_enabled = true;
  timer->is_done = false;
  timer->deadline_ms = deadline_ms;
  timer->birthtime_ms = millis();
}

// Re-arms a timer for another full interval, reusing its existing deadline.
void TimerReset(Timer* timer) { TimerCreate(timer, timer->deadline_ms); }

// Disarms a timer so TimerIsDone stops reporting it, without waiting it out.
void TimerDisable(Timer* timer) { timer->is_enabled = false; }
