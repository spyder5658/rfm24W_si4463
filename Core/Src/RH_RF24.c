#include "RH_RF24.h"

extern SPI_HandleTypeDef hspi1;




/*-----------radio library------------*/


void radio_reset(void)
{
  si446x_hal_sdn_high();
  HAL_Delay(150);
  si446x_hal_sdn_low();
  HAL_Delay(150);
}


uint8_t set_properties(const uint16_t id, const uint8_t *buff, const uint8_t len)
{
  uint8_t cmd[15] = {0};
  cmd[0] = id >> 8;
  cmd[1] = len;
  cmd[2] = id & 0xff;

  for (uint8_t i = 0; i < len; i++)
  {
    cmd[i + 3] = buff[i];
  }

  si446x_ctrl_send_cmd_stream(Si446x_CMD_SET_PROPERTY, cmd, len + 3);
  return si446x_ctrl_wait_cts();
}


void radio_set_power(uint8_t power)
{
  if (power > 0x7F)       //0x7F == 20dBm
  {
    power = 0x7F;
  }

  uint8_t power_ctrl[] = {0x18, power, 0x00 };
  set_properties(Si446x_PROP_PA_MODE, power_ctrl, sizeof(power_ctrl));
}


uint16_t radio_get_id(void)
{
  si446x_ctrl_send_cmd(Si446x_CMD_PART_INFO);

  uint8_t part_info[4] = {0};
  if (si446x_ctrl_get_response(part_info, sizeof(part_info)))
  {
    return (part_info[2] << 8) + part_info[3];
  }

  return 0;
}

void radio_set_state(radio_state_t s)
{
  const uint8_t cmd = (uint8_t)s;
  si446x_ctrl_send_cmd_stream(Si446x_CMD_CHANGE_STATE, &cmd, 1);
  si446x_ctrl_wait_cts();
}


bool setFrequency(float centre, float afcPullInRange)
{
    // See Si446x Data Sheet section 5.3.1
    // Also the Si446x PLL Synthesizer / VCO_CNT Calculator Rev 0.4
    uint8_t outdiv;
    uint8_t band;
    
    // Non-continuous frequency bands
    if (centre <= 1050.0 && centre >= 850.0)
        outdiv = 4, band = 0;
    else if (centre <= 525.0 && centre >= 425.0)
        outdiv = 8, band = 2;
    else if (centre <= 350.0 && centre >= 284.0)
        outdiv = 12, band = 3;
    else if (centre <= 175.0 && centre >= 142.0)
        outdiv = 24, band = 5;
    else 
        return false;

    
    // Set the MODEM_CLKGEN_BAND (not documented)
    uint8_t modem_clkgen[] = { band+8 };
    if (!set_properties(Si446x_PROP_MODEM_CLKGEN_BAND, modem_clkgen, sizeof(modem_clkgen)))
	return false;

    centre *= 1000000.0; // Convert to Hz

    // Need the Xtal/XO freq from the radio_config file:
    uint32_t xtal_frequency[1] = {30000000};
    unsigned long f_pfd = 2 * xtal_frequency[0] / outdiv;
    unsigned int n = ((unsigned int)(centre / f_pfd)) - 1;
    float ratio = centre / (float)f_pfd;
    float rest  = ratio - (float)n;
    unsigned long m = (unsigned long)(rest * 524288UL); 
    unsigned int m2 = m / 0x10000;
    unsigned int m1 = (m - m2 * 0x10000) / 0x100;
    unsigned int m0 = (m - m2 * 0x10000 - m1 * 0x100); 

    // PROP_FREQ_CONTROL_GROUP
    uint8_t freq_control[] = { n, m2, m1, m0 };
    return set_properties(Si446x_PROP_FREQ_CONTROL_INTE, freq_control, sizeof(freq_control));
}


bool radio_init(void)
{
  radio_reset();
  const uint8_t cmd[] = {0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};
  si446x_ctrl_send_cmd_stream(Si446x_CMD_POWER_UP, cmd, sizeof(cmd));
  si446x_ctrl_wait_cts(); // May take longer to set the CTS bit

  set_properties(Si446x_PROP_GLOBAL_XO_TUNE, (const uint8_t[]){98}, 1);
  radio_set_power(127);

   if (radio_get_id() != Si446x_CONF_ID)
  {
    // printf("%02x\n",radio_get_id());
    return false;
  }

  return true;

}



/*-----------radio control------------*/


uint8_t si446x_ctrl_wait_cts(void)
{
  for(uint16_t i=0; i < Si446x_CONF_MAX_CTS_TRIES; i++)
  {
    si446x_hal_spi_nsel_low();
    uint8_t cts = si446x_hal_spi_read_byte(Si446x_CMD_READ_CMD_BUFF);
    si446x_hal_spi_nsel_high();

    if(cts == Si446x_CTS_RESPONSE)
    {
      return 1;
    }
  }
  return 0;
}


void si446x_ctrl_send_cmd(const uint8_t cmd)
{
  si446x_hal_spi_nsel_low();
  si446x_hal_spi_write_byte(cmd);
  si446x_hal_spi_nsel_high();
  si446x_ctrl_wait_cts();
}

void si446x_ctrl_send_cmd_stream(const uint8_t cmd, const uint8_t* buffer, const uint8_t len)
{
  si446x_hal_spi_nsel_low();
  si446x_hal_spi_write_byte(cmd);
  si446x_hal_spi_write(buffer, len);
  si446x_hal_spi_nsel_high();
  si446x_ctrl_wait_cts();
}



uint8_t si446x_ctrl_get_response(uint8_t *buffer, const uint8_t len)
{
  if(si446x_ctrl_wait_cts())
  {
    si446x_hal_spi_nsel_low();
    si446x_hal_spi_read_byte(Si446x_CMD_READ_CMD_BUFF);
    si446x_hal_spi_read(buffer, len);
    si446x_hal_spi_nsel_high();
    return 1;
  }
  return 0;
}

/*-----------radio control------------*/




/*-----------low level-----------------------*/
void si446x_hal_sdn_high(void)
{
  HAL_GPIO_WritePin(RFM_SDN_GPIO_Port,RFM_SDN_Pin,SET);
}

void si446x_hal_sdn_low(void)
{
  HAL_GPIO_WritePin(RFM_SDN_GPIO_Port,RFM_SDN_Pin,RESET);
}

void si446x_hal_spi_nsel_low(void)
{
  HAL_GPIO_WritePin(RFM_CS_GPIO_Port,RFM_CS_Pin,RESET);

}

void si446x_hal_spi_nsel_high(void)
{
  HAL_GPIO_WritePin(RFM_CS_GPIO_Port,RFM_CS_Pin,SET);

}


void si446x_hal_spi_write_byte(const uint8_t cmd)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 1, HAL_MAX_DELAY);
}


uint8_t si446x_hal_spi_read_byte(const uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}


void si446x_hal_spi_write(const uint8_t* buffer, const uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    si446x_hal_spi_write_byte(buffer[i]);
  }
}


void si446x_hal_spi_read(uint8_t* buffer, const uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    buffer[i] = si446x_hal_spi_read_byte(Si446x_CMD_NOP);
  }
}

/*-----------low level-----------------------*/
