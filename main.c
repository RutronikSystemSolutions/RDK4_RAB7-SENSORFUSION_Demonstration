/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty RDK4  Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
* Created on: 2023-04-04
* Company: Rutronik Elektronische Bauelemente GmbH
* Address: Jonavos g. 30, Kaunas 44262, Lithuania
* Author: GDR
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "TLE926x.h"
#include "sys_timer.h"
#include "rab7.h"

/*Priority for SBC interrupts*/
#define SBC_IRQ_PRIORITY		1

#define PRINT_DATA_MS		1000

static SBC_ErrorCode sbc_setup(void);
static void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static uint32_t print_timestamp = 0;

/*TLE9262 SBC Interrupt Data*/
cyhal_gpio_callback_data_t sbc_int_data =
{
		.callback = sbc_interrupt_handler,
		.callback_arg = NULL,
};

/*SBC Power Supply Variables*/
sbc_vcc3_on_t vcc3_supp;
sbc_vcc2_on_t vcc2_supp;

int main(void)
{
    cy_rslt_t result;
    SBC_ErrorCode sbc_err;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize LEDs*/
    result = cyhal_gpio_init( USER_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( USER_LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( USER_LED_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Enable system time counter*/
	if (!sys_timer_init())
	{
		CY_ASSERT(0);
	}

    __enable_irq();

    /*Clean the terminal*/
    printf("\x1b[2J\x1b[;H");

    /* System Basis Chip configuration */
    sbc_err = sbc_setup();
    if(sbc_err.flippedBitsMask)
    {
    	printf("SBC failure.\r\n");
    	CY_ASSERT(0);
    }

    /*Initialise RAB7-SENSORFUSION adapter*/
    result = sensorfusion_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("RAB7-SENSORFUSION initialisation failure\r\n");
        CY_ASSERT(0);
    }

	/*Initialize current time*/
	uint32_t currentTime = get_system_time_ms();

    for (;;)
    {
     	/*Feed the watchdog*/
    	sbc_wd_trigger();

    	poll_sensors();

    	/*Print the data every PRINT_DATA_MS*/
    	currentTime = get_system_time_ms();
    	if((currentTime - print_timestamp) >  PRINT_DATA_MS)
    	{
    		cyhal_gpio_toggle(USER_LED_GREEN);
    		print_timestamp = get_system_time_ms();

        	printf("\x1b[2J\x1b[;H");
        	printf("   [ Sensor Fusion Adapter Board Data Output ]  \r\n");
        	printf("DPS368 --> T: %.2f deg C, P: %.2f Pa\r\n",
        			sensor_data_storage.dps_temperature,
					sensor_data_storage.dps_pressure*100
					);
        	printf("BMP585 --> T: %.2f deg C, P: %.2f Pa\r\n",
        			sensor_data_storage.bmp_temperature,
					sensor_data_storage.bmp_pressure
					);
        	printf("BME690 --> T: %.2f deg C, H: %.2f %%, P: %.2f Pa, Gas: %d, R: %.2f Ohm\r\n",
        			sensor_data_storage.bme_temperature,
					sensor_data_storage.bme_humidity,
					sensor_data_storage.bme_pressure,
					sensor_data_storage.bme_gas_index,
					sensor_data_storage.bme_gas_resistance);
        	printf("BMI323 --> accx: %d accy: %d accz: %d gyrx: %d gyry: %d gyrz: %d\r\n",
        			(unsigned int)sensor_data_storage.bmi_acc_x,
					(unsigned int)sensor_data_storage.bmi_acc_y,
					(unsigned int)sensor_data_storage.bmi_acc_z,
					(unsigned int)sensor_data_storage.bmi_gyr_x,
					(unsigned int)sensor_data_storage.bmi_gyr_y,
					(unsigned int)sensor_data_storage.bmi_gyr_z
					);
        	printf("SHT41  --> T: %.2f deg C, H: %.2f %%\r\n",
        			(float)sensor_data_storage.sht_temperature/1000,
					(float)sensor_data_storage.sht_humidity/1000
					);
        	printf("SGP41  --> VOC: %d raw, VOC INDEX: %d, NOX: %d raw, NOX INDEX: %d \r\n",
        			(int)sensor_data_storage.sgp_sraw_voc,
					(int)sensor_data_storage.sgp_voc_index,
					(int)sensor_data_storage.sgp_sraw_nox,
					(int)sensor_data_storage.sgp_nox_index
					);
        	printf("BMM350 --> magx: %.2f uT, magy: %.2f uT, magz: %.2f uT, T: %.2f deg C\r\n",
        			sensor_data_storage.bmm_mag_x,
					sensor_data_storage.bmm_mag_y,
					sensor_data_storage.bmm_mag_z,
					sensor_data_storage.bmm_temperature
					);
    	}
    }
}

static SBC_ErrorCode sbc_setup(void)
{
	cy_rslt_t result;
	SBC_ErrorCode sbc_err;

    /*Initialize SBC Interrupt Pin*/
    result = cyhal_gpio_init(INT_SBC, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {
    	/*SCB host interrupt init failure*/
    	sbc_err.flippedBitsMask = 0x03;
    	return sbc_err;
    }
    /*Register callback functions */
    cyhal_gpio_register_callback(INT_SBC, &sbc_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(INT_SBC, CYHAL_GPIO_IRQ_RISE, SBC_IRQ_PRIORITY, true);

    /*SBC Initializations*/
    sbc_err = sbc_init();
    if(sbc_err.flippedBitsMask)
    {
    	printf("SBC initialization failure.\r\n");
    	CY_ASSERT(0);
    }

    /* Turn ON the 5V Power Supply VCC2 */
    vcc2_supp = VCC2_ON_ALWAYS;
    sbc_err = sbc_switch_vcc2(vcc2_supp);
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not enable the VCC2.\r\n");
    	return sbc_err;
    }
    printf("SBC VCC2 Power Enabled.\r\n");

    /*Turn ON the 3.3V Power Supply VCC3 for the Arduino Shield(s) */
    vcc3_supp = VCC3_ENABLED;
    sbc_err = sbc_switch_vcc3(vcc3_supp);
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not enable the VCC3.\r\n");
    	return sbc_err;
    }
    printf("SBC VCC3 Power Enabled.\r\n");

    /*Enter Normal Mode*/
    sbc_err = sbc_mode_normal();
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not enter Normal mode.\r\n");
    	return sbc_err;
    }

    /*SBC Watchdog Configuration*/
    sbc_err = sbc_configure_watchdog(TIME_OUT_WD, NO_WD_AFTER_CAN_LIN_WAKE, WD_1000MS);
    if(sbc_err.flippedBitsMask)
    {
    	printf("Could not configure the watchdog.\r\n");
    	return sbc_err;
    }
    Cy_SysLib_Delay(100);

	return sbc_err;
}

/* SBC Interrupt handler callback function */
static void sbc_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    SBC_ISR();
}


/* [] END OF FILE */
