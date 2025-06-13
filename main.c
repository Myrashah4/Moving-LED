/* USER CODE BEGIN Header */

/**

******************************************************************************

* @file : main.c

* @brief : Main program body

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

#include "usb_host.h" // If USB is used

#include "stm32f407xx.h"



/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */



/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */



/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

#define LIS302DL_WHO_AM_I_ADDR (0x0F)

#define LIS302DL_CTRL_REG1_ADDR (0x20)

#define LIS302DL_OUT_X_ADDR (0x29)

#define LIS302DL_OUT_Y_ADDR (0x2B)

#define LIS302DL_OUT_Z_ADDR (0x2D)



#define LIS302DL_CTRL_REG1_CONFIG (0x47)



// Calibration constants

#define X_OFFSET -9

#define ACCEL_MAX_SCALE_VAL 2300 // for PWM scaling



// PWM constants

#define PWM_PERIOD_TICKS 1000 // Corresponds to TIM4->ARR + 1 for 1kHz PWM with 1MHz timer clock

/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */



/* USER CODE END PM */



/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1; // If I2C is used by HAL

I2S_HandleTypeDef hi2s3; // If I2S is used by HAL

SPI_HandleTypeDef hspi1; // If SPI1 is used by HAL (we are using register level for LIS302DL)



/* USER CODE BEGIN PV */

uint8_t x_raw, y_raw, z_raw; // Changed to uint8_t as LIS302DL output is 8-bit

int16_t x_final, y_final, z_final;

// uint16_t rxd,rxdf; // These were global, SPI_Transmit now returns its read value directly

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_I2S3_Init(void);

static void MX_SPI1_Init(void);

void MX_USB_HOST_Process(void); // If USB is used



// User-defined function

void My_GPIO_Init(void);

void My_SPI_Init(void);

uint8_t SPI_Transmit_Receive(uint8_t data);

void LIS_Write(uint8_t addr, uint8_t data);

uint8_t LIS_Read_Reg(uint8_t addr);

void LIS_Init(void);

void LIS_Read_Accel(void);

int16_t Convert_To_Val(uint8_t raw_val);

void PWM_Tim4_Init(void);

void TIM3_ms_Delay(uint16_t delay);

/* USER CODE BEGIN PFP */



/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */



void My_GPIO_Init(){

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;





GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);

GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);



//AFR[0] CLEAR

GPIOA->AFR[0] &= ~((0xFUL << GPIO_AFRL_AFSEL5_Pos) |

(0xFUL << GPIO_AFRL_AFSEL6_Pos) |

(0xFUL << GPIO_AFRL_AFSEL7_Pos));

//Alternate function setting 0101

GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0 |

GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0 |

GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0);

//Setting max speed 11

GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);



/*

// No pull-up, no pull-down for PA5, PA7

//GPIOA->PUPDR &= (GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD7_Msk);

// Example: No pull-up, no pull-down for MISO

// GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk);



//GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);

*/



// PE(3) - Chip Select

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

// PE3 - Output mode

GPIOE->MODER &= ~GPIO_MODER_MODER3_Msk;

GPIOE->MODER |= GPIO_MODER_MODER3_0; // Set as output mode

GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;

GPIOE->BSRR = GPIO_BSRR_BS3;



//Configuring LEDs

RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;



GPIOD->MODER &= ~(GPIO_MODER_MODER12_Msk | GPIO_MODER_MODER13_Msk |

GPIO_MODER_MODER14_Msk | GPIO_MODER_MODER15_Msk);

GPIOD->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 |

GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);



GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL12_Msk | GPIO_AFRH_AFSEL13_Msk |

GPIO_AFRH_AFSEL14_Msk |GPIO_AFRH_AFSEL15_Msk);

GPIOD->AFR[1] |= (GPIO_AFRH_AFSEL12_1 |GPIO_AFRH_AFSEL13_1 |

GPIO_AFRH_AFSEL14_1 |GPIO_AFRH_AFSEL15_1);



GPIOD->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 |

GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15);

}



void My_SPI_Init(){

RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI1 clock



SPI1->CR1 &= ~SPI_CR1_SPE;



//Baud rate - PCLK2/16 5.25Mhz, CPOL=0, CPHA = 0

SPI1->CR1 = SPI_CR1_MSTR | (SPI_CR1_BR_1 | SPI_CR1_BR_0) | SPI_CR1_SSM | SPI_CR1_SSI;



//SPI1->CR2 = 0x0000; // Motorola Mode - Should be set initially



SPI1->CR1 |= SPI_CR1_SPE;

}



uint8_t SPI_Transmit_Receive(uint8_t data){

while(!(SPI1->SR & SPI_SR_TXE)){} // Wait for TX buffer to be empty

SPI1->DR = data;



while(!(SPI1->SR & SPI_SR_RXNE)){}// Wait for RX buffer to have data

return (uint8_t)SPI1->DR;

}



void LIS_Write(uint8_t addr, uint8_t data){

GPIOE->BSRR = GPIO_BSRR_BR3; // Select LIS accelerometer



SPI_Transmit_Receive(addr); // Send address (MSB=0 for write)

SPI_Transmit_Receive(data); // Send data



GPIOE->BSRR = GPIO_BSRR_BS3; // De-select LIS accelerometer

}



// Read a byte from an LIS302DL register

uint8_t LIS_Read_Reg(uint8_t addr){

uint8_t received_data;

GPIOE->BSRR = GPIO_BSRR_BR3; // Select LIS accelerometer



SPI_Transmit_Receive(addr | 0x80U); // Send address - (0x80U - we are reading just the MSB)

received_data = SPI_Transmit_Receive(0x00); // Send dummy byte to receive data



GPIOE->BSRR = GPIO_BSRR_BS3; //De-select LIS accelerometer

return received_data;

}

//WHO_AM_I

void LIS_Init(){

LIS_Write(LIS302DL_CTRL_REG1_ADDR, LIS302DL_CTRL_REG1_CONFIG);

}



void LIS_Read_Accel(){

x_raw = LIS_Read_Reg(LIS302DL_OUT_X_ADDR);

y_raw = LIS_Read_Reg(LIS302DL_OUT_Y_ADDR);

z_raw = LIS_Read_Reg(LIS302DL_OUT_Z_ADDR);

}



void PWM_Tim4_Init(void){

RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable TIM4 clock



TIM4->PSC = 84 - 1;

TIM4->ARR = PWM_PERIOD_TICKS - 1;



TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);

TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);

TIM4->CCMR1 |= TIM_CCMR1_OC1PE;

TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);

TIM4->CCMR1 |= TIM_CCMR1_OC2PE;



TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC4M_Msk);

TIM4->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);

TIM4->CCMR2 |= TIM_CCMR2_OC3PE;

TIM4->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);

TIM4->CCMR2 |= TIM_CCMR2_OC4PE;



TIM4->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);



TIM4->CR1 |= TIM_CR1_ARPE;

TIM4->EGR |= TIM_EGR_UG;

TIM4->CR1 |= TIM_CR1_CEN;

}



void delay(volatile uint32_t time){

while(time--);

}





int16_t Convert_To_Val(uint8_t raw_val){

int8_t signed_val = (int8_t)raw_val;

return ((int16_t)signed_val * ACCEL_MAX_SCALE_VAL) / 127;

}

/* USER CODE END 0 */



/**

* @brief The application entry point.

* @retval int

*/

int main(void)

{

/* MCU Configuration--------------------------------------------------------*/

SystemClock_Config();



/* USER CODE BEGIN 2 */

My_GPIO_Init();

My_SPI_Init();

LIS_Init();

PWM_Tim4_Init();



// Initialize PWM channels to 0

TIM4->CCR1 = 0;

TIM4->CCR2 = 0;

TIM4->CCR3 = 0;

TIM4->CCR4 = 0;



/* Constants */

const uint16_t MIN_BRIGHTNESS = 50; // Minimum PWM value out of 1000



/* Infinite loop */

while (1)

{

LIS_Read_Accel();

x_final = Convert_To_Val(x_raw) + X_OFFSET;

y_final = Convert_To_Val(y_raw);



uint16_t pwm_duty;



// X-axis movement: Left (PD12 / TIM4_CH1) and Right (PD14 / TIM4_CH3)

if (x_final > 0) {

pwm_duty = ((uint32_t)x_final * (PWM_PERIOD_TICKS - 1)) / ACCEL_MAX_SCALE_VAL;

if (pwm_duty < MIN_BRIGHTNESS) pwm_duty = MIN_BRIGHTNESS;

if (pwm_duty >= PWM_PERIOD_TICKS) pwm_duty = PWM_PERIOD_TICKS - 1;

TIM4->CCR3 = pwm_duty; // Right

TIM4->CCR1 = 0;

} else if (x_final < 0) {

pwm_duty = ((uint32_t)(-x_final) * (PWM_PERIOD_TICKS - 1)) / ACCEL_MAX_SCALE_VAL;

if (pwm_duty < MIN_BRIGHTNESS) pwm_duty = MIN_BRIGHTNESS;

if (pwm_duty >= PWM_PERIOD_TICKS) pwm_duty = PWM_PERIOD_TICKS - 1;

TIM4->CCR1 = pwm_duty; // Left

TIM4->CCR3 = 0;

} else {

// Center X: Both off or dim

TIM4->CCR1 = MIN_BRIGHTNESS;

TIM4->CCR3 = MIN_BRIGHTNESS;

}



// Y-axis movement: Forward (PD13 / TIM4_CH2) and Backward (PD15 / TIM4_CH4)

if (y_final > 0) {

pwm_duty = ((uint32_t)y_final * (PWM_PERIOD_TICKS - 1)) / ACCEL_MAX_SCALE_VAL;

if (pwm_duty < MIN_BRIGHTNESS) pwm_duty = MIN_BRIGHTNESS;

if (pwm_duty >= PWM_PERIOD_TICKS) pwm_duty = PWM_PERIOD_TICKS - 1;

TIM4->CCR2 = pwm_duty; // Forward

TIM4->CCR4 = 0;

} else if (y_final < 0) {

pwm_duty = ((uint32_t)(-y_final) * (PWM_PERIOD_TICKS - 1)) / ACCEL_MAX_SCALE_VAL;

if (pwm_duty < MIN_BRIGHTNESS) pwm_duty = MIN_BRIGHTNESS;

if (pwm_duty >= PWM_PERIOD_TICKS) pwm_duty = PWM_PERIOD_TICKS - 1;

TIM4->CCR4 = pwm_duty; // Backward

TIM4->CCR2 = 0;

} else {

// Center Y: Both off or dim

TIM4->CCR2 = MIN_BRIGHTNESS;

TIM4->CCR4 = MIN_BRIGHTNESS;

}



delay(500000);

}

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

RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

RCC_OscInitStruct.HSEState = RCC_HSE_ON;

RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

RCC_OscInitStruct.PLL.PLLM = 8;

RCC_OscInitStruct.PLL.PLLN = 336;

RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

RCC_OscInitStruct.PLL.PLLQ = 7; // For USB, SDIO if used. 336/7 = 48 MHz.

if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

{

Error_Handler();

}



/** Initializes the CPU, AHB and APB buses clocks

*/

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // HCLK = 168MHz

RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // PCLK1 = HCLK/4 = 42MHz. TIMxCLK = 2*PCLK1 = 84MHz for TIM2-7,12-14

RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // PCLK2 = HCLK/2 = 84MHz. TIMxCLK = PCLK2 or 2*PCLK2 for TIM1,8-11. SPI1 CLK = PCLK2.



if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)

{

Error_Handler();

}

}



/**

* @brief I2C1 Initialization Function

* @param None

* @retval None

*/

static void MX_I2C1_Init(void)

{

/* USER CODE BEGIN I2C1_Init 0 */

/* USER CODE END I2C1_Init 0 */

/* USER CODE BEGIN I2C1_Init 1 */

/* USER CODE END I2C1_Init 1 */

hi2c1.Instance = I2C1;

hi2c1.Init.ClockSpeed = 100000;

hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

hi2c1.Init.OwnAddress1 = 0;

hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

hi2c1.Init.OwnAddress2 = 0;

hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

if (HAL_I2C_Init(&hi2c1) != HAL_OK)

{

Error_Handler();

}

/* USER CODE BEGIN I2C1_Init 2 */

/* USER CODE END I2C1_Init 2 */

}



/**

* @brief I2S3 Initialization Function

* @param None

* @retval None

*/

static void MX_I2S3_Init(void)

{

/* USER CODE BEGIN I2S3_Init 0 */

/* USER CODE END I2S3_Init 0 */

/* USER CODE BEGIN I2S3_Init 1 */

/* USER CODE END I2S3_Init 1 */

hi2s3.Instance = SPI3;

hi2s3.Init.Mode = I2S_MODE_MASTER_TX;

hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;

hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;

hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;

hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;

hi2s3.Init.CPOL = I2S_CPOL_LOW;

hi2s3.Init.ClockSource = I2S_CLOCK_PLL;

hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

if (HAL_I2S_Init(&hi2s3) != HAL_OK)

{

Error_Handler();

}

/* USER CODE BEGIN I2S3_Init 2 */

/* USER CODE END I2S3_Init 2 */

}



/**

* @brief SPI1 Initialization Function (HAL based, not used for LIS302DL in this code)

* @param None

* @retval None

*/

static void MX_SPI1_Init(void)

{

/* USER CODE BEGIN SPI1_Init 0 */

/* USER CODE END SPI1_Init 0 */

/* USER CODE BEGIN SPI1_Init 1 */

/* USER CODE END SPI1_Init 1 */

hspi1.Instance = SPI1;

hspi1.Init.Mode = SPI_MODE_MASTER;

hspi1.Init.Direction = SPI_DIRECTION_2LINES;

hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;

hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;

hspi1.Init.NSS = SPI_NSS_SOFT;

hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Matches My_SPI_Init if PCLK2 is 84MHz

hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;

hspi1.Init.TIMode = SPI_TIMODE_DISABLE;

hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

hspi1.Init.CRCPolynomial = 10;

if (HAL_SPI_Init(&hspi1) != HAL_OK)

{

Error_Handler();

}

/* USER CODE BEGIN SPI1_Init 2 */

/* USER CODE END SPI1_Init 2 */

}



/**

* @brief GPIO Initialization Function (HAL based)

* @param None

* @retval None

*/

static void MX_GPIO_Init(void)

{

GPIO_InitTypeDef GPIO_InitStruct = {0};



/* GPIO Ports Clock Enable */

__HAL_RCC_GPIOE_CLK_ENABLE(); // For PE3 if not handled by My_GPIO_Init

__HAL_RCC_GPIOC_CLK_ENABLE(); // For PC0 (OTG FS Power Switch), PC3 (PDM_OUT)

__HAL_RCC_GPIOH_CLK_ENABLE(); // For HSE on PH0/PH1 if used

__HAL_RCC_GPIOA_CLK_ENABLE(); // For PA0 (User Button) if not handled by My_GPIO_Init

__HAL_RCC_GPIOB_CLK_ENABLE(); // For PB2 (BOOT1), PB10 (CLK_IN)

__HAL_RCC_GPIOD_CLK_ENABLE(); // For PD5 (OTG FS OverCurrent) if not handled by My_GPIO_Init



/*Configure GPIO pin Output Level */

// HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET); // PE3 (CS for LIS302DL) is handled by My_GPIO_Init



/*Configure GPIO pin Output Level */

HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET); // PC0



/*Configure GPIO pin Output Level */

// HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin |Audio_RST_Pin, GPIO_PIN_RESET); // PD12-15 for LEDs handled by PWM



/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */

GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin; // Assumes this is defined, e.g. GPIO_PIN_0 on GPIOC

GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

GPIO_InitStruct.Pull = GPIO_NOPULL;

GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct); // Assumes OTG_FS_PowerSwitchOn_GPIO_Port is GPIOC



/*Configure GPIO pin : PDM_OUT_Pin */

GPIO_InitStruct.Pin = PDM_OUT_Pin; // Assumes defined e.g. GPIO_PIN_3 on GPIOC

GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

GPIO_InitStruct.Pull = GPIO_NOPULL;

GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;

HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct); // Assumes PDM_OUT_GPIO_Port is GPIOC



/*Configure GPIO pin : B1_Pin */

GPIO_InitStruct.Pin = B1_Pin; // User button PA0

GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Or EVT_RISING

GPIO_InitStruct.Pull = GPIO_NOPULL;

HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // B1_GPIO_Port is GPIOA



/*Configure GPIO pin : BOOT1_Pin */

GPIO_InitStruct.Pin = BOOT1_Pin; // PB2

GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

GPIO_InitStruct.Pull = GPIO_NOPULL;

HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct); // BOOT1_GPIO_Port is GPIOB



/*Configure GPIO pin : CLK_IN_Pin */

GPIO_InitStruct.Pin = CLK_IN_Pin; // PB10

GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

GPIO_InitStruct.Pull = GPIO_NOPULL;

GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

GPIO_InitStruct.Alternate = GPIO_AF5_SPI2; // I2S2_CK for microphone

HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct); // CLK_IN_GPIO_Port is GPIOB



/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */

GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin; // PD5

GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

GPIO_InitStruct.Pull = GPIO_NOPULL;

HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct); // OTG_FS_OverCurrent_GPIO_Port is GPIOD



/*Configure GPIO pin : MEMS_INT2_Pin */ // PE1 (for LIS3DSH on some Discovery boards, not LIS302DL)

GPIO_InitStruct.Pin = MEMS_INT2_Pin;

GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;

GPIO_InitStruct.Pull = GPIO_NOPULL;

HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct); // MEMS_INT2_GPIO_Port is GPIOE

}



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



/**

* @brief This function is executed in case of error occurrence.

* @retval None

*/

void Error_Handler(void)

{

/* USER CODE BEGIN Error_Handler_Debug */

__disable_irq();

while (1)

{

// Example: Toggle a spare LED or set a breakpoint

}

/* USER CODE END Error_Handler_Debug */

}



#ifdef USE_FULL_ASSERT

/**

* @brief Reports the name of the source file and the source line number

* where the assert_param error has occurred.

* @param file: pointer to the source file name

* @param line: assert_param error line source number

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




