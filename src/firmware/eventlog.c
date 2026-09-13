/*
 * bbs-fw
 *
 * Copyright (C) Daniel Nilsson, 2022.
 *
 * Released under the GPL License, Version 3
 */

#include "eventlog.h"
#include "uart.h"

static bool is_enabled;

void eventlog_init(bool enabled)
{
	is_enabled = enabled;
}

bool eventlog_is_enabled()
{
	return is_enabled;
}

void eventlog_set_enabled(bool enabled)
{
	is_enabled = enabled;
}

void eventlog_write(uint8_t evt)
{
	if (!is_enabled)
	{
		return;
	}

	uart_write(0xee);
	uart_write(evt);
	uart_write((uint8_t)0xee + evt);
}
void eventlog_write_data(uint8_t evt, int16_t data)
{
	if (!is_enabled)
	{
		return;
	}

	uint8_t checksum = 0;

	uart_write(0xed); checksum += (uint8_t)0xed;
	uart_write(evt); checksum += evt;
	uart_write((uint8_t)(data >> 8)); checksum += (uint8_t)(data >> 8);
	uart_write((uint8_t)data); checksum += (uint8_t)data;
	uart_write(checksum);
}

// 0xEC frame: header, target current %, target speed %, cadence x10 rpm (hi/lo),
// checksum. Note this frame is intentionally distinct from the 0xED data frame
// because it carries three values instead of one.
void eventlog_write_telemetry(uint8_t target_current_percent, uint8_t target_speed_percent, uint16_t cadence_rpm_x10)
{
	if (!is_enabled)
	{
		return;
	}

	uint8_t checksum = 0;

	uart_write(0xec); checksum += (uint8_t)0xec;
	uart_write(target_current_percent); checksum += target_current_percent;
	uart_write(target_speed_percent); checksum += target_speed_percent;
	uart_write((uint8_t)(cadence_rpm_x10 >> 8)); checksum += (uint8_t)(cadence_rpm_x10 >> 8);
	uart_write((uint8_t)cadence_rpm_x10); checksum += (uint8_t)cadence_rpm_x10;
	uart_write(checksum);
}
