// RH_RF24.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF24.h,v 1.11 2015/01/02 21:38:24 mikem Exp $
//
// Supports RF24/RF26 and RFM24/RFM26 modules in FIFO mode
// also Si4464/63/62/61/60-A1
// Si4063 is the same but Tx only
//
// Per http://www.hoperf.cn/upload/rf/RFM24.pdf
// and http://www.hoperf.cn/upload/rf/RFM26.pdf
// Sigh: the HopeRF documentation is utter rubbish: full of errors and incomplete. The Si446x docs are better:
// http://www.silabs.com/Support%20Documents/TechnicalDocs/Si4464-63-61-60.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN626.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN627.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN647.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN633.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN736.pdf
// http://nicerf.com/manage/upfile/indexbanner/635231050196868750.pdf    (API description)
// http://www.silabs.com/Support%20Documents/Software/Si446x%20RX_HOP%20PLL%20Calculator.xlsx
#ifndef RH_RF24_h
#define RH_RF24_h

#include "main.h"
#include <inttypes.h>

#define Si446x_CONF_MAX_CTS_TRIES    2500
#define Si446x_CTS_RESPONSE          0xFF
#define Si446x_CMD_READ_CMD_BUFF     0x44
#define Si446x_CMD_POWER_UP          0x02
#define Si446x_CMD_SET_PROPERTY      0x11
#define Si446x_PROP_GLOBAL_XO_TUNE   0x0000
#define Si446x_PROP_PA_MODE          0x2200
#define Si446x_CMD_PART_INFO         0x01
#define Si446x_CONF_ID               0x4463
#define Si446x_CMD_NOP               0x00
#define Si446x_PROP_MODEM_MOD_TYPE   0x2000
#define Si446x_PROP_FREQ_CONTROL_INTE 0x4000
#define Si446x_PROP_MODEM_CLKGEN_BAND 0x2051
#define Si446x_CMD_CHANGE_STATE 0x34


typedef enum
{
  NO_CHANGE = 0x00,
  SLEEP = 0x01,
  SPI_ACTIVE = 0x02,
  READY = 0x03,
  TUNE_TX = 0x05,
  TUNE_RX = 0x06,
  START_TX = 0x07,
  START_RX = 0x08
}radio_state_t;


void radio_reset(void);
uint8_t set_properties(const uint16_t id, const uint8_t *buff, const uint8_t len);
void radio_set_power(uint8_t power);
uint16_t radio_get_id(void);
void radio_set_state(radio_state_t s);
bool setFrequency(float centre, float afcPullInRange);
bool radio_init(void);
uint8_t si446x_ctrl_wait_cts(void);
void si446x_ctrl_send_cmd(const uint8_t cmd);
void si446x_ctrl_send_cmd_stream(const uint8_t cmd, const uint8_t* buffer, const uint8_t len);
uint8_t si446x_ctrl_get_response(uint8_t *buffer, const uint8_t len);

void si446x_hal_sdn_high(void);
void si446x_hal_sdn_low(void);
void si446x_hal_spi_nsel_low(void);
void si446x_hal_spi_nsel_high(void);
void si446x_hal_spi_write_byte(const uint8_t cmd);
uint8_t si446x_hal_spi_read_byte(const uint8_t cmd);
void si446x_hal_spi_write(const uint8_t* buffer, const uint8_t len);
void si446x_hal_spi_read(uint8_t* buffer, const uint8_t len);





















#endif