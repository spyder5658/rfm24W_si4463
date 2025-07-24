/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include "RH_RF24.h"
#include <stdbool.h>
int _write(int file, char *data, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
  return len;
}



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



#define MINI_MORSE_DIT_TIME    1200/20
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/*-----------low level-----------------------*/


/*-----------radio control------------*/

void si446x_hal_spi_read(uint8_t* buffer, const uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    buffer[i] = si446x_hal_spi_read_byte(Si446x_CMD_NOP);
  }
}

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
  if (power > 0x7F)
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


void radio_init_morse(void)
{
  const uint8_t modem_config[] = {0x0,0x1,0x1,0x00};
  set_properties(Si446x_PROP_MODEM_MOD_TYPE, modem_config, sizeof(modem_config));

  setFrequency(312.3, 0);

}




enum MorseLetter
{
  DIT,
  DAH
};

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

void radio_set_state(radio_state_t s)
{
  const uint8_t cmd = (uint8_t)s;
  si446x_ctrl_send_cmd_stream(Si446x_CMD_CHANGE_STATE, &cmd, 1);
  si446x_ctrl_wait_cts();
}
 void mini_morse_start_tx(const enum MorseLetter m)
{
  radio_set_state(START_TX);
  // si446x_morse_high();
  HAL_GPIO_WritePin(GPIO_MORSE_GPIO_Port,GPIO_MORSE_Pin,SET);

}

void mini_morse_stop_tx(const enum MorseLetter m)
{
  // si446x_morse_low();
  HAL_GPIO_WritePin(GPIO_MORSE_GPIO_Port,GPIO_MORSE_Pin,RESET);
  radio_set_state(READY);
}

void mini_morse_delay(const uint16_t delay)
{
  HAL_Delay(delay);
}

const char alphameric[] =
{
  // Alphabets
  (char)0b01000010, // A, .-
  (char)0b10000001, // B, -...
  (char)0b10100001, // C, -.-.
  (char)0b10000110, // D, -..
  (char)0b00000100, // E, .
  (char)0b00100001, // F, ..-.
  (char)0b11000110, // G, --.
  (char)0b00000001, // H, ....
  (char)0b00000010, // I, ..
  (char)0b01111001, // J, .---
  (char)0b10100110, // K, -.-
  (char)0b01000001, // L, .-..
  (char)0b11000010, // M, --
  (char)0b10000010, // N, -.
  (char)0b11100110, // O, ---
  (char)0b01100001, // P, .--.
  (char)0b11010001, // Q, --.-
  (char)0b01000110, // R, .-.
  (char)0b00000110, // S, ...
  (char)0b10000100, // T, -
  (char)0b00100110, // U, ..-
  (char)0b00010001, // V, ...-
  (char)0b01100110, // W, .--
  (char)0b10010001, // X, -..-
  (char)0b10110001, // Y, -.--
  (char)0b11000001, // Z, --..

  // Numbers
  (char)0b11111101, // 0, -----
  (char)0b01111101, // 1, .----
  (char)0b00111101, // 2, ..---
  (char)0b00011101, // 3, ...--
  (char)0b00001101, // 4, ....-
  (char)0b00000101, // 5, .....
  (char)0b10000101, // 6, -....
  (char)0b11000101, // 7, --...
  (char)0b11100101, // 8, ---..
  (char)0b11110101, // 9, ----.
};

const struct PunctuationPair
{
  const char letter;
  const char code;
} punctuation[] =
{
  // Punctuations
  {',', (char)0b11001111}, // --..--
  {'.', (char)0b01010111}, // .-.-.-
  {'!', (char)0b10101111}, // -.-.--
  {':', (char)0b11100011}, // ---...
  {';', (char)0b10101011}, // -.-.-.
  {'(', (char)0b10110101}, // -.--.
  {')', (char)0b10110111}, // -.--.-
  {'"', (char)0b01001011}, // .-..-.
  {'@', (char)0b01101011}, // .--.-.
  {'&', (char)0b01000101}, // .-...
  {'?', (char)0b00110011}, // ..--..
  {'-', (char)0b10000111}, // -....-
  {'+', (char)0b01010101}, // .-.-.
  {'/', (char)0b10010101}, // -..-.
  {'=', (char)0b10001101}, // -...-
  {'*', (char)0b10010001}, // -..-
  {'\\', (char)0b01111011}, // .----.
};

void mini_morse_tx(const char* msg, const uint16_t len)
{
  for(uint16_t i = 0; i < len; i++)
  {
    mini_morse_encode_char(msg[i]);
  }
}

char character_to_code(const char x)
{
  char output = 0;
  if (x >= 'A' && x <= 'Z')
  {
    output = alphameric[x - 'A'];
  }
  else if (x >= 'a' && x <= 'z')
  {
    output = alphameric[x - 'a'];
  }
  else if (x >= '0' && x <= '9')
  {
    output = alphameric[x - '0' + 26];
  }
  else
  {
    for (uint8_t i = 0; i < sizeof(punctuation) / 2; i++)
    {
      if (x == punctuation[i].letter)
      {
        output = punctuation[i].code;
      }
    }
  }
  return output;
}

void mini_morse_encode_char(const char x)
{
  if (x == ' ')
  {
    mini_morse_delay(7 * MINI_MORSE_DIT_TIME);
  }
  else
  {
    const char code = character_to_code(x);
    const char length = ((code >> 2) & 1) | (code & 2) | ((code << 2) & 4);
    const char stop = 7 - length + (length > 6);
    for (int i = 7; i > stop; i--)
    {
      if ((code >> i) & 1)
      {
        mini_morse_start_tx(DAH);
        mini_morse_delay(3 * MINI_MORSE_DIT_TIME);
        mini_morse_stop_tx(DAH);
      }
      else
      {
        mini_morse_start_tx(DIT);
        mini_morse_delay(MINI_MORSE_DIT_TIME);
        mini_morse_stop_tx(DIT);
      }
      mini_morse_delay(MINI_MORSE_DIT_TIME);
    }
    mini_morse_delay(2 * MINI_MORSE_DIT_TIME );
  }
}



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  const uint8_t morse_msg[] = "123456\0";

    // Send large code buffer using fragmentation
    if(!radio_init())
    {
      printf("radio failed\r\n");
      while (1);
    }
      printf("radio success\r\n");
       radio_init_morse();
    printf("after setup\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
 

    mini_morse_tx("Hello\0", strlen("Hello\0"));
    HAL_Delay(2000);

    // Example_Send_Large_Code();
    // HAL_Delay(2000);

    // HAL_Delay(3000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
