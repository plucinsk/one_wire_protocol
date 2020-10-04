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
//#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "bsp.h"
#include "boards.h"
#include "app_error.h"
//#include "app_timer.h"
#include "nrf_gpio.h"
//#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ONE_WIRE_PIN 31
#define NUMBER_OF_SAMPLES 40

static int number_of_events = 0;
static int timing_array[NUMBER_OF_SAMPLES];
volatile bool cc0_timeout;

//APP_TIMER_DEF(m_timer_id);
//#define TIMER_INTERVAL APP_TIMER_TICKS(3000) //3000 ms intervals

static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(2);
static nrf_ppi_channel_t m_ppi_channel1;
static nrf_ppi_channel_t m_ppi_channel2;

/*static void timer_timeout_handler(void * p_context)
{
  if (number_of_events == 0 || number_of_events == NUMBER_OF_SAMPLES)
  {
    NRF_LOG_INFO("got here! number_of_events = %0d", number_of_events);
    set_irq_us(1000);
    set_irq_us(1000);
    set_irq_us(80);
    //gpiote_init();
    //ppi_init();
  }
  else
  {
    NRF_LOG_INFO("number_of_events = %0d", number_of_events);
    nrf_drv_timer_clear(&m_timer2);
    NRF_LOG_INFO("COUNTER CLEARED!");
  }
}*/

/*static void timer_init(void)
{
  ret_code_t err_code;
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  app_timer_create(&m_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
}*/

static int * calculate_humidity_and_temperature( int *input_arr)
{
  int arr[NUMBER_OF_SAMPLES];
  static int data_o[2];
  static int crc;

  for(int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    arr[i] = *input_arr;
    if (arr[i] > 100 && arr[i] < 150)
    {
      arr[i] = 1;
    } 
    else 
    {
      arr[i] = 0;
    }
    input_arr++;
  }

  for(int j = 0; j < 16; j++)
  {
    data_o[0] += arr[j]       * (0x8000 >> j);
    data_o[1] += arr[j+16]    * (0x8000 >> j);
  }

  for (int k = 0; k < 8; k++)
  {
    crc += arr[k + 32] * (0x80 >> k);
  }

  if (!((( ((data_o[0] & 0xFF00) >> 8) + (data_o[0] & 0x00FF) + ((data_o[1] & 0xFF00) >> 8) + (data_o[1] & 0x00FF)) & crc) == crc))
  {
    data_o[0] = 0xDEAD;
    data_o[1] = 0xBEEF;
  }

  return data_o;
}

void TIMER0_IRQHandler(void)
{
  if (NRF_TIMER0 -> EVENTS_COMPARE[0])
  {
    cc0_timeout = true;
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    nrf_gpio_pin_toggle(ONE_WIRE_PIN);
  }
}

static void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
}

static void timer2_event_handler(nrf_timer_event_t event_type, void * p_context)
{
  int * p;
  NRF_LOG_INFO("Limit Reached");
  nrf_drv_gpiote_uninit();
  bsp_board_led_invert(2);
  p = calculate_humidity_and_temperature(timing_array);
  for (int i = 0; i < 2; i++)
  {
    NRF_LOG_INFO("data_o[%0d] = %0d", i, *(p + i));
  }
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity)
{
  int captured_time;

  number_of_events++;
  //NRF_LOG_INFO("number_of_events = %0d", number_of_events);
  captured_time = nrf_drv_timer_capture_get(&m_timer1, NRF_TIMER_CC_CHANNEL0);
  timing_array[number_of_events - 1] = captured_time;
  //NRF_LOG_INFO("timing_array[%0d] = %0d", number_of_events - 1, timing_array[number_of_events -1]);
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
                                   NUMBER_OF_SAMPLES,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

  nrf_drv_timer_enable(&m_timer2);
}

static void gpiote_init(void)
{
  ret_code_t err_code;

  nrf_drv_gpiote_in_config_t configH2L = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  configH2L.pull = NRF_GPIO_PIN_PULLUP;

  nrf_drv_gpiote_uninit();

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  
  err_code = nrf_drv_gpiote_in_init(ONE_WIRE_PIN, &configH2L, gpiote_event_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(ONE_WIRE_PIN, true);
}

static void ppi_init(void)
{
  ret_code_t err_code;
  
  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel2);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1, 
                                        nrf_drv_gpiote_in_event_addr_get(ONE_WIRE_PIN), 
                                        nrf_drv_timer_task_address_get(&m_timer2, NRF_TIMER_TASK_COUNT));
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_assign(m_ppi_channel2,
                                        nrf_drv_gpiote_in_event_addr_get(ONE_WIRE_PIN),
                                        nrf_drv_timer_task_address_get(&m_timer1, NRF_TIMER_TASK_CAPTURE0));
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_fork_assign(m_ppi_channel2, 
                                             nrf_drv_timer_task_address_get(&m_timer1, NRF_TIMER_TASK_CLEAR));
  APP_ERROR_CHECK(err_code);

  nrf_drv_ppi_channel_enable(m_ppi_channel1);
  nrf_drv_ppi_channel_enable(m_ppi_channel2);
}

int main(void)
{
  bool pressed = false;
  ret_code_t err_code;

  err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  bsp_board_init(BSP_INIT_LEDS);
  bsp_board_init(BSP_INIT_BUTTONS);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  timer1_init();
  timer2_init();
  //timer_init();
  
  nrf_gpio_cfg_output(ONE_WIRE_PIN);
  nrf_gpio_pin_set(ONE_WIRE_PIN);
  irq_init();
  ppi_init();

  while (1)
  {
    if(!pressed)
    {
      if(bsp_board_button_state_get(0))
      {
        bsp_board_led_invert(0);
        set_irq_us(1000);
        set_irq_us(1000);
        set_irq_us(80);
        gpiote_init();
        pressed = true;
      }
    }
  }
}
