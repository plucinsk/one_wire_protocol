/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @file
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "bsp.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ONE_WIRE_PIN BSP_BUTTON_0

static int timing[42];
static int number_of_events = 0;

typedef enum
{
  /*** HOST PART ***/
  RSTL_us           = 1000,
  RSTH_us           = 30,
  /*** SENSOR PART ***/
  RESPL_us          = 80,
  RESPH_us          = 80,
  BITL_us           = 50,
  BIT0H_us          = 26,
  BIT1H_us          = 70,
  END_us            = 50,
  SAMPLING_RATE_us  = 10
} one_wire_t;

volatile bool cc0_timeout;
static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(2);
static nrf_ppi_channel_t m_ppi_channel1;
static nrf_ppi_channel_t m_ppi_channel2;

void TIMER0_IRQHandler(void)
{
  if (NRF_TIMER0 -> EVENTS_COMPARE[0])
  {
    cc0_timeout = true;
    nrf_gpio_pin_toggle(13);
  }
}

static void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
}

static void timer2_event_handler(nrf_timer_event_t event_type, void * p_context)
{
  NRF_LOG_INFO("Limit Reached");
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity)
{
  number_of_events++;
  NRF_LOG_INFO("number_of_events = %0d", number_of_events);
  nrf_drv_gpiote_out_toggle(13);
  /*timing[number_of_events] = nrf_drv_timer_capture_get(&m_timer1, NRF_TIMER_CC_CHANNEL0);
  NRF_LOG_INFO("timing[%0d] = %0d", number_of_events, timing[number_of_events]);
  number_of_events++;

  if(number_of_events == 42)
  {
    nrf_drv_gpiote_in_event_disable(ONE_WIRE_PIN);
    nrf_drv_timer_disable(&m_timer1);
    number_of_events = 0;
  }*/
}

void set_irq_us (uint32_t irq_time_us)
{
    cc0_timeout = false;
    NRF_TIMER0 -> TASKS_CAPTURE[0] = 1;
    NRF_TIMER0 -> EVENTS_COMPARE[0] = 0;
    NRF_TIMER0 -> INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0 -> CC[0] += irq_time_us;
    while(! cc0_timeout)
    {
      __WFE();
    }
    NRF_TIMER0 -> INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
}

void irq_init(void)
{
  ret_code_t err_code;

  NRF_CLOCK -> EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK -> TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK -> EVENTS_HFCLKSTARTED)
  {

  }

  NRF_TIMER0 -> MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
  NRF_TIMER0 -> PRESCALER = 4;
  NRF_TIMER0 -> BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);

  NRF_TIMER0 -> INTENCLR = ~0;
  NRF_TIMER0 -> INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
  NVIC_ClearPendingIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER0_IRQn);

  NRF_TIMER0 -> TASKS_CLEAR = 1;
  NRF_TIMER0 -> TASKS_START = 1;
}

static void timer1_init(void)
{
  ret_code_t err_code;

  nrf_drv_timer_config_t timer1_cfg;
  timer1_cfg.mode = NRF_TIMER_MODE_TIMER;
  timer1_cfg.frequency = NRF_TIMER_FREQ_1MHz;
  timer1_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

  err_code = nrf_drv_timer_init(&m_timer1, &timer1_cfg, timer1_event_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_timer_enable(&m_timer1);
}

static void timer2_init(void)
{
  ret_code_t err_code;

  nrf_drv_timer_config_t timer2_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer2_cfg.mode = NRF_TIMER_MODE_COUNTER;
  timer2_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;

  err_code = nrf_drv_timer_init(&m_timer2, &timer2_cfg, timer2_event_handler);

  nrf_drv_timer_extended_compare(&m_timer2,
                                   NRF_TIMER_CC_CHANNEL0,
                                   5,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

  nrf_drv_timer_enable(&m_timer2);
}

static void gpiote_init(void)
{
  ret_code_t err_code;

  nrf_drv_gpiote_out_config_t config_out = GPIOTE_CONFIG_OUT_SIMPLE(false);
  nrf_drv_gpiote_in_config_t configH2L = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

  //configH2L.hi_accuracy = true;
  //configH2L.is_watcher = false;
  configH2L.pull = NRF_GPIO_PIN_PULLUP;
  //configH2L.sense = NRF_GPIOTE_POLARITY_TOGGLE;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_gpiote_in_init(ONE_WIRE_PIN, &configH2L, gpiote_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_gpiote_out_init(13, &config_out);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(ONE_WIRE_PIN, true);
  //nrf_drv_gpiote_out_task_enable(13);
}

static void ppi_init(void)
{
  ret_code_t err_code;
  
  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
  APP_ERROR_CHECK(err_code);

  //err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel2);
  //APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1, 
                                        nrf_drv_gpiote_in_event_addr_get(ONE_WIRE_PIN), 
                                        nrf_drv_gpiote_out_task_addr_get(13));
  APP_ERROR_CHECK(err_code);

  //err_code = nrf_drv_ppi_channel_fork_assign(m_ppi_channel1, nrf_drv_timer_task_address_get(&m_timer1, NRF_TIMER_TASK_CLEAR));
  //APP_ERROR_CHECK(err_code);

  //err_code = nrf_drv_ppi_channel_assign(m_ppi_channel2,
                                        //nrf_drv_gpiote_in_event_addr_get(ONE_WIRE_PIN),
                                        //nrf_drv_timer_task_address_get(&m_timer2, NRF_TIMER_TASK_COUNT));
  //APP_ERROR_CHECK(err_code);

  
}

int main(void)
{
  ret_code_t err_code;
  static int count = 0;
  err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
  gpiote_init();
  //ppi_init();
  //timer1_init();
  //timer2_init();
  


  while (1)
  {
    if (count != number_of_events)
    {
      count = number_of_events;
      NRF_LOG_INFO("count = %0d", count);
    }
  }
}