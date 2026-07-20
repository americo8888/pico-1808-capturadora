/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Reinhard Panhuber
 * Copyright (c) 2026 Jolan (Modified for PCM1808 Stereo 24-bit Slave)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "tusb_config.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "i2s_slave.pio.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
#define AUDIO_SAMPLE_RATE CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE
#define N_CHANNELS        CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX
#define BYTES_PER_SAMPLE  CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX

// I2S Pins
#define PIN_DATA 10
#define PIN_BCLK 11
#define PIN_LRCK 12

// DMA buffer size: 1ms of audio at 48kHz Stereo
#define SAMPLES_PER_MS    (AUDIO_SAMPLE_RATE / 1000)
#define DMA_BUF_SIZE      (SAMPLES_PER_MS * N_CHANNELS)
#define DMA_BUFFER_COUNT  8
#define USB_PACKET_SIZE   (DMA_BUF_SIZE * BYTES_PER_SAMPLE)

// Ring of DMA buffers. The DMA IRQ queues filled buffers and audio_task()
// drains them into TinyUSB without overwriting unread audio.
static uint32_t i2s_rx_buffer[DMA_BUFFER_COUNT][DMA_BUF_SIZE];
static uint8_t usb_tx_buffer[USB_PACKET_SIZE];
static volatile uint8_t dma_write_index = 0;
static volatile uint8_t ready_queue[DMA_BUFFER_COUNT];
static volatile uint8_t ready_head = 0;
static volatile uint8_t ready_tail = 0;
static volatile uint8_t ready_count = 0;
static volatile uint32_t dma_overrun_count = 0;
static volatile uint32_t usb_backpressure_count = 0;
static volatile uint32_t usb_short_write_count = 0;

static uint dma_chan;

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

uint32_t sampFreq;
uint8_t clkValid;

audio20_control_range_4_n_t(1) sampleFreqRng;

void led_blinking_task(void);
void audio_task(void);
static void dma_handler(void);
static bool buffer_is_queued(uint8_t buffer_index);
static bool pop_ready_buffer(uint8_t *buffer_index);

static bool buffer_is_queued(uint8_t buffer_index) {
  uint8_t pos = ready_tail;

  for (uint8_t i = 0; i < ready_count; i++) {
    if (ready_queue[pos] == buffer_index) {
      return true;
    }

    pos = (uint8_t)((pos + 1) % DMA_BUFFER_COUNT);
  }

  return false;
}

// DMA Interrupt Handler
static void dma_handler(void) {
  dma_hw->ints0 = 1u << dma_chan;

  uint8_t filled_index = dma_write_index;

  if (ready_count >= DMA_BUFFER_COUNT - 1) {
    // Keep one free buffer for the next DMA transfer. If the USB side falls
    // behind, drop the oldest complete buffer and record the event.
    ready_tail = (uint8_t)((ready_tail + 1) % DMA_BUFFER_COUNT);
    ready_count--;
    dma_overrun_count++;
  }

  ready_queue[ready_head] = filled_index;
  ready_head = (uint8_t)((ready_head + 1) % DMA_BUFFER_COUNT);
  ready_count++;

  uint8_t next_index = 0;
  bool found_next = false;
  for (uint8_t i = 0; i < DMA_BUFFER_COUNT; i++) {
    if (i != filled_index && !buffer_is_queued(i)) {
      next_index = i;
      found_next = true;
      break;
    }
  }

  if (!found_next) {
    next_index = filled_index;
    dma_overrun_count++;
  }

  dma_write_index = next_index;
  dma_channel_set_write_addr(dma_chan, i2s_rx_buffer[dma_write_index], true);
}

static bool pop_ready_buffer(uint8_t *buffer_index) {
  uint32_t save = save_and_disable_interrupts();

  if (ready_count == 0) {
    restore_interrupts(save);
    return false;
  }

  *buffer_index = ready_queue[ready_tail];
  ready_tail = (uint8_t)((ready_tail + 1) % DMA_BUFFER_COUNT);
  ready_count--;

  restore_interrupts(save);
  return true;
}

// PIO I2S Slave Initializer
static void pio_i2s_slave_init(PIO pio, uint sm, uint offset) {
    pio_sm_config c = i2s_slave_input_program_get_default_config(offset);
    
    // Set IN pins starting at PIN_DATA
    sm_config_set_in_pins(&c, PIN_DATA);
    
    // Configure JMP pin for LRCK if needed, but we use 'wait pin'
    // 'wait pin X' in PIO uses relative GPIO index to the 'in' base if configured,
    // but we use absolute indices or just ensure the pins are consecutive.
    // In our .pio, we used 'pin 1' for BCLK and 'pin 2' for LRCK.
    // So if PIN_DATA=10, pin 0=10, pin 1=11, pin 2=12.
    
    // Shift Left (MSB first), Autopush enabled, threshold 24 bits
    sm_config_set_in_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    
    // We don't need a specific clock divider as we wait for BCLK edges
    sm_config_set_clkdiv(&c, 1.0f);
    
    pio_sm_init(pio, sm, offset, &c);
    
    // Set GPIOs to PIO function
    pio_gpio_init(pio, PIN_DATA);
    pio_gpio_init(pio, PIN_BCLK);
    pio_gpio_init(pio, PIN_LRCK);
    
    // Pins are inputs
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_DATA, 3, false);
}

/*------------- MAIN -------------*/
int main(void) {
  board_init();

  // init device stack
  tusb_rhport_init_t dev_init = {
      .role = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_AUTO};
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  board_init_after_tusb();

  // PIO Init
  PIO pio = pio0;
  uint sm = 0;
  uint offset = (uint) pio_add_program(pio, &i2s_slave_input_program);
  pio_i2s_slave_init(pio, sm, offset);
  pio_sm_set_enabled(pio, sm, true);

  // DMA Init
  dma_chan = (uint) dma_claim_unused_channel(true);
  dma_channel_config dma_c = dma_channel_get_default_config(dma_chan);
  channel_config_set_read_increment(&dma_c, false);
  channel_config_set_write_increment(&dma_c, true);
  channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_32);
  channel_config_set_dreq(&dma_c, pio_get_dreq(pio, sm, false));
  
  dma_channel_configure(
      dma_chan,
      &dma_c,
      i2s_rx_buffer[0],
      &pio->rxf[sm],
      DMA_BUF_SIZE,
      false
  );
  
  // Enable DMA Interrupt
  dma_channel_set_irq0_enabled(dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);
  
  dma_channel_start(dma_chan);

  // Audio Init values
  sampFreq = AUDIO_SAMPLE_RATE;
  clkValid = 1;

  sampleFreqRng.wNumSubRanges = 1;
  sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bRes = 0;

  while (1) {
    tud_task();
    led_blinking_task();
    audio_task();
  }
}

void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void) {
  tu_fifo_t *ep_in_fifo = tud_audio_get_ep_in_ff();
  if (ep_in_fifo == NULL || tu_fifo_remaining(ep_in_fifo) < USB_PACKET_SIZE) {
    usb_backpressure_count++;
    return;
  }

  uint8_t filled_buf_index = 0;
  if (!pop_ready_buffer(&filled_buf_index)) return;

  uint32_t *src = i2s_rx_buffer[filled_buf_index];
  
  // Pack 32-bit (containing 24-bit data in [23:0]) into 3-byte USB format
  // UAC2 24-bit is Little Endian by default for subslots.
  for (uint16_t i = 0; i < DMA_BUF_SIZE; i++) {
    uint32_t sample = src[i];
    usb_tx_buffer[i * 3 + 0] = (uint8_t)(sample & 0xFF);
    usb_tx_buffer[i * 3 + 1] = (uint8_t)((sample >> 8) & 0xFF);
    usb_tx_buffer[i * 3 + 2] = (uint8_t)((sample >> 16) & 0xFF);
  }

  uint16_t written = tud_audio_write(usb_tx_buffer, USB_PACKET_SIZE);
  if (written != USB_PACKET_SIZE) {
    usb_short_write_count++;
  }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport; (void) pBuff;
  (void) p_request;
  return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport; (void) pBuff;
  (void) p_request;
  return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) p_request;
  (void) pBuff;
  return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport; (void) p_request;
  return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport; (void) p_request;
  return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  if (entityID == 1) {
    if (ctrlSel == AUDIO20_TE_CTRL_CONNECTOR) {
      if (p_request->bRequest != AUDIO20_CS_REQ_CUR) return false;

      audio20_desc_channel_cluster_t ret = { .bNrChannels = 2, .bmChannelConfig = 0x03, .iChannelNames = 0 };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));
    }
  }

  if (entityID == 4) {
    if (ctrlSel == AUDIO20_CS_CTRL_SAM_FREQ) {
      if (p_request->bRequest == AUDIO20_CS_REQ_CUR) {
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));
      } else if (p_request->bRequest == AUDIO20_CS_REQ_RANGE) {
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));
      }
    } else if (ctrlSel == AUDIO20_CS_CTRL_CLK_VALID) {
      if (p_request->bRequest != AUDIO20_CS_REQ_CUR) return false;

      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));
    }
  }
  return false;
}

void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;
  if (tusb_time_millis_api() - start_ms < blink_interval_ms) return;
  start_ms += blink_interval_ms;
  board_led_write(led_state);
  led_state = 1 - led_state;
}
