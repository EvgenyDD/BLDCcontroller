/*
	Copyright 2012-2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * nrf_driver.c
 *
 *  Created on: 29 mar 2015
 *      Author: benjamin
 */

#include <string.h>
#include "nrf_driver.h"
#include "rf.h"
#include "rfhelp.h"
#include "conf_general.h"
#include "app.h"
#include "buffer.h"
#include "commands.h"
#include "crc.h"
#include "packet.h"
#include "hw.h"
#include "mc_interface.h"
#include <math.h>
#include "timeout.h"
#include "encoder.h"
#include "ch.h"

// Settings
#define MAX_PL_LEN				25
#define RX_BUFFER_SIZE			PACKET_MAX_PL_LEN

#define ALIVE_INTERVAL			5  // Send alive packets at this rate
#define NRF_RESTART_TIMEOUT		500  // Restart the NRF if nothing has been received or acked for this time

// Variables
static THD_WORKING_AREA(rx_thread_wa, 2048);
static THD_WORKING_AREA(tx_thread_wa, 512);
//static mote_state mstate;
//static uint8_t rx_buffer[RX_BUFFER_SIZE];
static int nosend_cnt;
static int nrf_restart_rx_time;
static int nrf_restart_tx_time;

float smartSpeedIncrement;

// Functions
static THD_FUNCTION(rx_thread, arg);
static THD_FUNCTION(tx_thread, arg);
static int rf_tx_wrapper(uint8_t *data, uint32_t len);

void nrf_driver_init(void) {
	rfhelp_init();

	nosend_cnt = 0;
	nrf_restart_rx_time = 0;
	nrf_restart_tx_time = 0;

	chThdCreateStatic(rx_thread_wa, sizeof(rx_thread_wa), NORMALPRIO+1, rx_thread, NULL);
	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa), NORMALPRIO, tx_thread, NULL);
}

#include "hal.h"
static int rf_tx_wrapper(uint8_t *data, uint32_t len)
{
	palSetPad(GPIOA, 3);
	int res = rfhelp_send_data_crc(data, len);
	palClearPad(GPIOA, 3);

	if (res == 0) {
		nrf_restart_tx_time = NRF_RESTART_TIMEOUT;
	}

	return res;
}

uint8_t turnOffNow = 0;

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nrf TX");

	for(;;) {
		nosend_cnt++;

		if (nosend_cnt >= ALIVE_INTERVAL) {
			uint8_t pl[32];
			int32_t index = 0;

			if(turnOffNow)
			{
				pl[index++] = 1;
				rf_tx_wrapper(pl, index);
				rf_tx_wrapper(pl, index);
				RELAY_TURN_OFF();
			}

		    pl[index++] = 0; //packet id
		    buffer_append_float16(pl, mc_interface_get_rpm() / 7.0, 1.0, &index);
		    buffer_append_int16(pl, mc_interface_read_reset_avg_motor_current() * 10, &index);
		    buffer_append_int16(pl, mc_interface_read_reset_avg_input_current() * 10, &index);
		    pl[index++] = (uint8_t)(GET_INPUT_VOLTAGE() * 10.0);
		    pl[index++] = (int8_t)(NTC_TEMP(ADC_IND_TEMP_MOTOR));
		    pl[index++] = (int8_t)(NTC_TEMP(ADC_IND_TEMP_INVRT));
		    pl[index++] = mc_interface_get_fault();
		    buffer_append_uint16(pl, mc_interface_get_amp_hours(false) * 1000, &index);
		    buffer_append_uint16(pl, mc_interface_get_amp_hours_charged(false) * 1000, &index);

			rf_tx_wrapper(pl, index);
			nosend_cnt = 0;
		}

		chThdSleepMilliseconds(10);
	}

}


static THD_FUNCTION(rx_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nrf RX");

	for(;;) {
		uint8_t buf[32];
		uint16_t len;
		uint8_t pipe;

		for(;;) {
			palSetPad(GPIOC, 3);
			int res = rfhelp_read_rx_data_crc(buf, &len, &pipe);
			palClearPad(GPIOC, 3);

			// If something was read
			if(res >= 0)
			{

/* 				static uint8_t delay = 0;
				if(++delay >= 20)
				{
					delay = 0;
					for(uint8_t i=0; i<len; i++)
						commands_printf("%d = %d", i, buf[i]);
					commands_printf("===============");
				} */

//			uint8_t packet = buf[0] & ;

				nrf_restart_rx_time = NRF_RESTART_TIMEOUT;

//				switch (packet) {
//				case 0:
//				{
					if(buf[0] & COM_NRF_OFF)
						RELAY_TURN_OFF();

					float value = (float) (((uint32_t) buf[1]) << 24 | ((uint32_t) buf[2]) << 16 |
									       ((uint32_t) buf[3]) << 8	| ((uint32_t) buf[4])) / 1000.0;

					float mul = (buf[0] & COM_NRF_INV) ? -1.0 : 1.0;

					if(buf[0] & COM_NRF_BRAKE)
					{
						mc_interface_set_brake_current(value);
						smartSpeedIncrement = 0.0;
					}
					else if(buf[0] & COM_NRF_DUTY)
					{
						mc_interface_set_duty(value * mul);
					}
					else if(buf[0] & COM_NRF_CURR)
					{
						mc_interface_set_current(value * mul);
					}
					else if(buf[0] & COM_NRF_SPEED)
					{
						mc_interface_set_pid_speed(value * mul);
					}
					else if(buf[0] & COM_NRF_SMRT_SPEED)
					{
						smartSpeedIncrement = value * mul;
					}
					else
					{
						smartSpeedIncrement = 0.0;
						mc_interface_release_motor();
					}

					timeout_reset();
//				}
//					break;
//
//				case 1:
//
//					break;
//
//				default:
//					break;
//				}
			}

			// Stop when there is no more data to read.
			if (res <= 0) {
				break;
			} else {
				// Sleep a bit to prevent locking the other threads.
				chThdSleepMilliseconds(1);
			}
		}

		chThdSleepMilliseconds(5);

		// Restart the nrf if nothing has been received for a while
		if (nrf_restart_rx_time > 0 && nrf_restart_tx_time > 0) {
			nrf_restart_rx_time -= 5;
			nrf_restart_tx_time -= 5;
		} else {
			rfhelp_power_up();
			rfhelp_restart();
			nrf_restart_rx_time = NRF_RESTART_TIMEOUT;
			nrf_restart_tx_time = NRF_RESTART_TIMEOUT;
		}
	}
}
