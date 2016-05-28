/*
 * This file is part of belnanosat project.
 *
 * Copyright (C) 2016 Uladzislau Paulovich <selatnick@gmail.com>
 *
 * belnanosat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * belnanosat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with belnanosat.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RADIATION_SENSOR_H
#define RADIATION_SENSOR_H

#include <inttypes.h>

/*
 * How old a signal pulse should be to be discarded
 */
#define RADIATION_SENSOR_NOISE_TIMEOUT 200000
#define RADIATION_SENSOR_MAX_TIMESTAMPS 20

extern void radiation_sensor_setup(void);
extern void radiation_sensor_clear_data(void);

extern uint64_t radiation_sensor_window_start_time;
extern uint32_t radiation_sensor_cnt;
// Packet is saved to sdcard approximately every 100ms
// therefore we hope to not overflow this buffer
// Each element contains (pulse_timestamp_i - window_start_time)
extern uint32_t radiation_sensor_pulses[RADIATION_SENSOR_MAX_TIMESTAMPS];

#endif
