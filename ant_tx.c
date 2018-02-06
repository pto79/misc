/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2014
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */
/**@file
 * @defgroup nrf_ant_broadcast_tx_example ANT Broadcast TX Example
 * @{
 * @ingroup nrf_ant_broadcast
 *
 * @brief Example of basic ANT Broadcast TX.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "math.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "bsp.h"
#include "hardfault.h"
#include "ant_stack_config.h"
#include "ant_channel_config.h"
#include "softdevice_handler.h"
#include "ant_key_manager.h"
#include "ant_key_manager_config.h"

#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"



#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// Channel configuration.
#define ANT_BROADCAST_CHANNEL_NUMBER    0x00    /**< ANT Channel 0. */
#define EXT_ASSIGN_NONE                 0x00    /**< ANT Ext Assign. */

// Miscellaneous defines.
#define ANT_NETWORK_NUMBER              0x00    /**< Default public network number. */

//#define ANT_PLUS_NETWORK_KEY    {0xB9, 0xA5, 21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}            /**< The ANT+ network key. */

static uint8_t m_counter = 1u;                  /**< Counter to increment the ANT broadcast data payload. */

		static nrf_saadc_value_t adc_value_0;
		static nrf_saadc_value_t adc_value_1;
		static nrf_saadc_value_t adc_value_2;
		static nrf_saadc_value_t adc_value_3;
		static nrf_saadc_value_t adc_value_4;
		
#define ADC_ref 2.56
#define zero_x 1.569
#define zero_y 1.569
#define zero_z 1.569
#define sensitivity_x 0.3
#define sensitivity_y 0.3
#define sensitivity_z 0.3

unsigned int value_x;
unsigned int value_y;
unsigned int value_z;
float xv;
float yv;
float zv;
float angle_x;
float angle_y;
float angle_z;
		

/**@brief Function for setting payload for ANT message and sending it.
 */
void ant_message_send()
{
    uint32_t err_code;
    uint8_t  message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];

		uint16_t pitch, roll, yaw, roll1, pitch1;

		//pitch = (atan2(adc_value_2 , sqrt(adc_value_0 * adc_value_0 + adc_value_1 * adc_value_1)) * 57);
		roll = atan2(adc_value_1 , adc_value_2) * 157;
	
		roll1 = atan2(adc_value_1, sqrt(adc_value_0 * adc_value_0 + adc_value_2 * adc_value_2)) * 157;
	
		pitch = atan2(adc_value_0, sqrt(adc_value_1 * adc_value_1 + adc_value_2 * adc_value_2)) * 300;
		
		yaw = atan2(sqrt(adc_value_0 * adc_value_0 + adc_value_1 * adc_value_1) , adc_value_2) * 157;
	
	
	value_x = adc_value_0;
	value_y = adc_value_1;
	value_z = adc_value_2;
	
	xv=(value_x/1024.0*ADC_ref-zero_x)/sensitivity_x;
	yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
	zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;
	angle_x =atan2(-yv,-zv)*57.2957795+180;
	angle_y =atan2(-xv,-zv)*57.2957795+180;
	angle_z =atan2(-yv,-xv)*57.2957795+180;
	
	adc_value_0 = (uint16_t)angle_x;
	adc_value_1 = (uint16_t)angle_y;
	adc_value_2 = (uint16_t)angle_z;
	
	
    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
    //message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = (uint8_t)(roll1 >> 8);
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = (uint8_t)roll1;
    //message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = (uint8_t)(adc_value_0 >> 8);
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = (uint8_t)adc_value_0;
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 3] = (uint8_t)(adc_value_1 >> 8);
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 4] = (uint8_t)adc_value_1;
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 5] = (uint8_t)(adc_value_2 >> 8);
		//message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 6] = (uint8_t)adc_value_2;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = (uint8_t)(pitch >> 8);
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = (uint8_t)pitch;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 3] = (uint8_t)(adc_value_3 >> 8);
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 4] = (uint8_t)adc_value_3;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 5] = (uint8_t)(adc_value_4 >> 8);
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 6] = (uint8_t)adc_value_4;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = 0x80;



    // Broadcast the data.
    err_code = sd_ant_broadcast_message_tx(ANT_BROADCAST_CHANNEL_NUMBER,
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           message_payload);
    APP_ERROR_CHECK(err_code);

    // Increment the counter.
    m_counter++;
}

/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
    uint32_t err_code;

    if (p_ant_evt->channel == ANT_BROADCAST_CHANNEL_NUMBER)
    {
        switch (p_ant_evt->event)
        {
            case EVENT_TX:
                ant_message_send();

                err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
                APP_ERROR_CHECK(err_code);
                break;

            default:
                break;
        }
    }
}

/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    uint32_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LED,
                        NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_handler_init(&clock_lf_cfg, NULL, 0, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
	
	  err_code = ant_plus_key_set(ANT_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting up the ANT module to be ready for TX broadcast.
 */
static void ant_channel_tx_broadcast_setup(void)
{
    uint32_t err_code;

    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = ANT_BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_MASTER,
        .ext_assign        = EXT_ASSIGN_NONE,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = CHAN_ID_DEV_NUM,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUMBER,
    };

    err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Fill tx buffer for the first frame.
    ant_message_send();

    // Open channel.
    err_code = sd_ant_channel_open(ANT_BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}



#define SAMPLES_IN_BUFFER 2

static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
/*	
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        
        //NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[0]);
			adc_value_0 = p_event->data.done.p_buffer[0];
			adc_value_1 = p_event->data.done.p_buffer[1];
    }
*/
	
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;


	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X;                                    //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
	
    nrf_saadc_channel_config_t channel0_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);	//p0.04 -> X
    nrf_saadc_channel_config_t channel1_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);	//p0.29 -> Y
		nrf_saadc_channel_config_t channel2_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);	//p0.31 -> Z
		nrf_saadc_channel_config_t channel3_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);	//p0.03 -> brake
		nrf_saadc_channel_config_t channel4_config 
        = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);	//p0.28	-> steering

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config.pin_p = NRF_SAADC_INPUT_AIN4;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; 

    err_code = nrf_drv_saadc_channel_init(0, &channel0_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel1_config);
    APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_saadc_channel_init(2, &channel2_config);
    APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_saadc_channel_init(3, &channel3_config);
    APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_saadc_channel_init(4, &channel4_config);
    APP_ERROR_CHECK(err_code);	

    //err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER);
    //APP_ERROR_CHECK(err_code);

}



/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SAADC HAL simpler example.\r\n");
    saadc_init();	
	
    utils_setup();
    softdevice_setup();
    ant_channel_tx_broadcast_setup();
		

    // Main loop.
    for (;;)
    {
        //nrf_drv_saadc_sample();
        //nrf_delay_ms(100);
			
			nrf_drv_saadc_sample_convert(0, &adc_value_0);
			nrf_drv_saadc_sample_convert(1, &adc_value_1);
			nrf_drv_saadc_sample_convert(2, &adc_value_2);
			nrf_drv_saadc_sample_convert(3, &adc_value_3);
			nrf_drv_saadc_sample_convert(4, &adc_value_4);

			
				//Sample SAADC on two channels
			/*
				err_code = nrf_drv_saadc_sample_convert(0, &adc_value_0);
				APP_ERROR_CHECK(err_code);
				err_code = nrf_drv_saadc_sample_convert(1, &adc_value_1);
				APP_ERROR_CHECK(err_code);			
			*/
			
#if CPU_LOAD_TRACE
        // Disabling interrupts in this way is highly not recommended. It has an impact on the work
        // of the softdecive and it is used only in order to show CPU load.
        __disable_irq();
        bsp_board_led_off(BSP_BOARD_LED_0);
        __WFI();
        bsp_board_led_on(BSP_BOARD_LED_0);
        __enable_irq();
#else
        // Put CPU in sleep if possible.
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
#endif // CPU_LOAD_TRACE
    }
}

/**
 *@}
 **/
