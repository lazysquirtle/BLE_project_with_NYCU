/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "boards.h"

#include "app_error.h"
#include "app_uart.h" 
#include "app_util_platform.h"

#include "nrf_delay.h"
#include "nrf_assert.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "mcp4725.h"

#define UART_TX_BUF_SIZE 256                                          
#define UART_RX_BUF_SIZE 256 
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define SAMPLES_IN_BUFFER 1
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);

/* MCP4725 Configuratin */
mcp4725_pins_config_t   pins_config = {
.scl_pin = SCL_PIN,
.sda_pin = SDA_PIN
};

/* saadc indicator */
volatile uint8_t state = 1;
static nrf_saadc_value_t     m_buffer_pool[2][1];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

void uart_error_handle(app_uart_evt_t * p_event)

{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR){
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR){
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    //printf("In uart_event_handle\n");
}

static void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params ={
    RX_PIN_NUMBER,
    TX_PIN_NUMBER,
    RTS_PIN_NUMBER,
    CTS_PIN_NUMBER,
    UART_HWFC,
    false,
    NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);
    APP_ERROR_CHECK(err_code);
}


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        /*
        Set data > 100 to prevent logic 0 from being taken. 
        The counter value is not a serial number because   
        the m_adc_evt_counter will increment in either case.
        */

        int i;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++){
            int16_t data = p_event->data.done.p_buffer[i];
            if (data > 100){
                printf("\n\rADC event number %d : ", (int)m_adc_evt_counter);
                printf("%d\t",data);
            }                   
        }
        m_adc_evt_counter++;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    //APP_ERROR_CHECK(err_code);

}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    /*
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    */
    
    uart_init();
    uint8_t   cr[5];
    uint8_t   time[5];
    uint8_t   count[5];
    uint8_t   i;
    uint32_t  ret_error;
    cr[4] = '\0';
    time[4] = '\0';
    count[4] = '\0';
    
    //meassure the elapsed time
    /*
    uint32_t start;
    uint32_t stop;
    uint32_t elapsed;
    // enable DWT
    CoreDebug->DEMCR |= 0x01000000;
    // Reset cycle counter
    DWT->CYCCNT = 0;
    // enable cycle counter
    DWT->CTRL |= 0x1;
    */
    printf("\n\r");
    printf("           Control the Pulse           \n\r");
    printf("=======================================\n\r");
    printf("Please enter Voltage value (unit : mv) \n\r");
    printf("==== Voltage Range: 500mV - 4500mV ====\n\r");
 

    for (i = 0 ;i < 4 ;i++ ){
        while (app_uart_get(&cr[i]) != NRF_SUCCESS);  
        app_uart_put(cr[i]);
    }
        
        
    uint16_t voltage = atoi(cr);
    printf("\n\r");
    printf("voltage = %d\n\r",voltage);
    if (voltage > 4500 || voltage < 500){
        printf("Out of Range! Press Reset!\n\r");
        ASSERT(false);
    }       
    //voltage input conversion for mcp4725_set_voltage() 
    voltage = (voltage << 12)/5000; 
    
    printf("=======================================\n\r");
    printf("Please enter the Pulsewidth (unit : us)\n\r");
    printf("== Pulsewidth Range : 800us - 1200us ==\n\r");

    for (i = 0 ;i < 4 ;i++ ){
        while (app_uart_get(&time[i]) != NRF_SUCCESS); 
        app_uart_put(time[i]);
    }
    uint16_t pulsewidth = atoi(time);
    printf("\n\r");
    printf("Pulsewidth = %d\n\r",pulsewidth);
    if (pulsewidth > 1200 || pulsewidth < 800){
        printf("Out of Range! Press Reset\n\r");
        ASSERT(false);
    }
    //time interval of mcp4725_set_volatage() equals 450us
    pulsewidth = pulsewidth - 450;        
    
    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);

    ret_code = mcp4725_setup(&pins_config);
    APP_ERROR_CHECK(ret_code);
    
    
    printf("=======================================\n\r");
    printf("Please enter the number of pulses.     \n\r");
    printf("=======================================\n\r");
    for (i = 0 ;i < 4 ;i++ ){
        while (app_uart_get(&count[i]) != NRF_SUCCESS); 
        app_uart_put(count[i]);
    }
    uint16_t total_num_pulse = atoi(count);
    printf("\n\r");
    if (total_num_pulse < 0){                         //overflow
        printf("Out of Range! Press Reset\n\r");
        ASSERT(false);
    }
    
    //meassure the elapsed time
    /*        
    start = DWT->CYCCNT;
    ret_code = mcp4725_set_voltage(voltage,false);
    APP_ERROR_CHECK(ret_code);
    stop = DWT->CYCCNT;
    elapsed = stop-start;
    printf("cycles for mcp4725_set_volatge = %u", elapsed);
    */

    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    
    for(int i = 0; i < total_num_pulse; i++){
        ret_code = mcp4725_set_voltage(voltage,false);
        APP_ERROR_CHECK(ret_code); 
        nrf_delay_us(pulsewidth);
        ret_code = mcp4725_set_voltage(0,false);
        APP_ERROR_CHECK(ret_code);
        nrf_delay_us(pulsewidth);
    }
    

}


/** @} */
