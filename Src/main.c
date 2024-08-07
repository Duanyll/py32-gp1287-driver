/**
  * Demo: Write Option Bytes
  * 
  * Board: PY32F003W1XS (SOP16)
  * 
  * This demo shows how to config reset pin as gpio output
  */

#include "main.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_ll_spi.h"
#include "py32f0xx_ll_tim.h"
#include "ws2812_spi.h"


static void APP_GPIOConfig(void);
static void APP_TIM1Config(void);
static void APP_SPIConfig(void);

int main(void)
{
  BSP_RCC_HSI_24MConfig();

  LL_mDelay(1000);

  APP_GPIOConfig();
  APP_TIM1Config();
  APP_SPIConfig();

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

  uint8_t i = 0, r = 0, g = 0x60, b = 0xC0;
  ws2812_pixel_all(r, g, b);
  ws2812_send_spi();

  while (1)
  {
    i = (i + 1) % WS2812_NUM_LEDS;
    ws2812_pixel(i, r++, g++, b++);
    ws2812_send_spi();
    LL_mDelay(20);
  }
}

uint8_t SPI_TxRxByte(uint8_t data) {
  uint8_t SPITimeout = 0xFF;
  /* Check the status of Transmit buffer Empty flag */
  while (READ_BIT(SPI1->SR, SPI_SR_TXE) == RESET) {
    if (SPITimeout-- == 0)
      return 0;
  }
  LL_SPI_TransmitData8(SPI1, data);
  SPITimeout = 0xFF;
  while (READ_BIT(SPI1->SR, SPI_SR_RXNE) == RESET) {
    if (SPITimeout-- == 0)
      return 0;
  }
  // Read from RX buffer
  return LL_SPI_ReceiveData8(SPI1);
}

static void APP_GPIOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitTypeDef;

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

  // PA0 - PH_VA - TIM1 CH3 PWM
  // PA1 - PF_VA - TIM1 CH4 PWM
  // PA2 - LIGHT - ADC CH2
  // PA3 - LED_DIN - SPI1 MOSI
  // PA4 - BTN2 - ADC CH4
  // PA6 - I2C_INT - GPIO Output PushPull
  // PA7 - PH_EN# - GPIO Output PushPull
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  GPIO_InitTypeDef.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitTypeDef.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitTypeDef.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &GPIO_InitTypeDef);

  // PB0 - PF_EN# - GPIO Output PushPull
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitTypeDef);

  // PB1 - PWR_SDA - GPIO Output OpenDrain
  // PB2 - PWR_SCL - GPIO Output OpenDrain
  // PB3 - PWR_KEY - GPIO Output OpenDrain
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitTypeDef.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitTypeDef.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitTypeDef.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOB, &GPIO_InitTypeDef);

  // PB6 - TX - USART1 TX
  // PF0 - I2C_SDA - I2C1 SDA
  // PF1 - I2C_SCL - I2C1 SCL
}

static void APP_TIM1Config(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler = 0;
  TIM1CountInit.Autoreload = 30 - 1; // 24MHz / 30 = 800KHz
  TIM1CountInit.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM1CountInit);

  LL_GPIO_InitTypeDef GPIO_InitTypeDef;
  // PA0 - CH3
  // PA1 - CH4
  GPIO_InitTypeDef.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitTypeDef.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitTypeDef.Alternate = LL_GPIO_AF_2;
  GPIO_InitTypeDef.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitTypeDef.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOA, &GPIO_InitTypeDef);

  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct;
  TIM_OC_Initstruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_Initstruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_Initstruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_Initstruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;

  TIM_OC_Initstruct.CompareValue = 10;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_Initstruct);
  TIM_OC_Initstruct.CompareValue = 6;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_Initstruct);
}

static void APP_SPIConfig(void) {
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

  // PA3 MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* The frequency after prescaler should be below 8.25MHz */
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_Enable(SPI1);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
