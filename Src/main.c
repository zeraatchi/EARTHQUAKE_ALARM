/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tm_stm32_hd44780.h"
#include "sd_hal_mpu6050.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
SD_MPU6050 mpu1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void calibration_(void);
void alarm_off(void);
void alarm_on(void);
void update_g(char inp);
int16_t g_x;
int16_t g_y;
int16_t g_z;
float temper;

char key_read(void);
#define m_select  1
#define m_left    2
#define m_right   3
#define m_up      4
#define m_down    5
#define m_nothing 0

void lcd_bk_light(char i);
#define on_bk 1
#define off_bk 0

void vib_menu_show(void);

uint32_t ADCValue=0;


int i;
int16_t x_offset,y_offset,z_offset;

	char alarm=0;
	char menu=0;
	int max_g,min_g;
  char len;
	char buffer[50];
	
	
  SD_MPU6050_Result result ;
	uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
	uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t customChar[] = {
    0x1F,    /*  xxx 11111 */
    0x11,    /*  xxx 10001 */
    0x11,    /*  xxx 10001 */
    0x11,    /*  xxx 10001 */
    0x11,    /*  xxx 10001 */
    0x11,    /*  xxx 10001 */
    0x11,    /*  xxx 10001 */
    0x1F    /*  xxx 11111 */
};

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);


    return ch;
}
   

int main(void)

{
	max_g=100;
	min_g=-1*max_g;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
	
	    TM_HD44780_Init(16, 2);
			TM_HD44780_Clear();
    
    /* Save custom character on location 0 in LCD */
    //TM_HD44780_CreateChar(0, customChar);
    

 /*

//print out static text
len=sprintf(buffer,"Hello\r\n"); //sprintf will return the length of 'buffer'  
HAL_UART_Transmit(&huart1, buffer, len, 1000);

//print out variable
len=sprintf(buffer,"This is i:%i\r\n",i); //sprintf will return the length of 'buffer'  
HAL_UART_Transmit(&huart1, buffer, len, 1000);  


//https://letanphuc.net/2015/09/stm32f0-uart-tutorial-5/
*/

HAL_ADC_Start(&hadc1);
HAL_Delay(1000);
			TM_HD44780_Clear();
			TM_HD44780_Puts(1,0,"X");
			TM_HD44780_Puts(7,0,"Y");
			TM_HD44780_Puts(13,0,"Z");

//printf ("start: %c %c \n", 'a', 65);

 calibration_();
 TM_HD44780_DisplayOff();
 lcd_bk_light(off_bk);

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //Toggle the state of pin PC9
		//HAL_UART_Transmit(&huart1, buffer, sizeof(buffer), HAL_MAX_DELAY);
		 //printf ("Characters: %c %c \n", 'a', 65);
   
	  //HAL_Delay(1);
  /*
	  if(result == SD_MPU6050_Result_Ok)
	  {
		  HAL_UART_Transmit(&huart1,mpu_ok,(uint16_t)15,1000);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart1,mpu_not,(uint16_t)17,1000);
	  }
		*/
	  //HAL_Delay(1);
		if (alarm==0){
      update_g(0);
			if (g_x>max_g||g_x<min_g||g_y>max_g||g_y<min_g||g_z>max_g||g_z<min_g){ 
				        alarm_on();
								alarm=1;
				         TM_HD44780_DisplayOn();
                 lcd_bk_light(on_bk);
	       }
       }
		
			  switch (key_read()) {
					case m_right: if (alarm==1) {alarm_off();HAL_Delay(500);alarm=0;TM_HD44780_DisplayOff();lcd_bk_light(off_bk);}break;
          case m_select: if (menu==0) {alarm_on();HAL_Delay(100);alarm_off();alarm=1;menu=1; TM_HD44780_DisplayOn();lcd_bk_light(on_bk);TM_HD44780_Clear();TM_HD44780_Puts(0,0,"Vibration Factor");vib_menu_show();}break;
					case m_up :   if (menu==1){max_g++;min_g=max_g*-1;vib_menu_show();}break;
          case m_down : if (menu==1) if (max_g>30) {max_g--;min_g=max_g*-1;vib_menu_show();}break;
					case m_left:  if(menu==1) {update_g(1);HAL_Delay(1000);alarm=0;menu=0;TM_HD44780_DisplayOff();lcd_bk_light(off_bk);}break;
					}
		
		   HAL_Delay(50);
		

  

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void update_g(char inp){
			SD_MPU6050_ReadTemperature(&hi2c1,&mpu1);
			temper = mpu1.Temperature;
			SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
			g_x = mpu1.Gyroscope_X-x_offset;
			g_y = mpu1.Gyroscope_Y-y_offset;
			g_z = mpu1.Gyroscope_Z-z_offset;
			/* USER CODE BEGIN 3 */
			//SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
			//int16_t a_x = mpu1.Accelerometer_X;
			//int16_t a_y = mpu1.Accelerometer_Y;
			//int16_t a_z = mpu1.Accelerometer_Z;

			//len=sprintf(buffer,"%i\t  %i\t   %i\t  \n",g_x,g_y,g_z); //sprintf will return the length of 'buffer'  
			//HAL_UART_Transmit(&huart1, buffer, len, 1000);

      if (inp==1){
			  TM_HD44780_Clear();
			  TM_HD44780_Puts(1,0,"X");
			  TM_HD44780_Puts(7,0,"Y");
			  TM_HD44780_Puts(13,0,"Z");
		}
		TM_HD44780_Puts(0,1,"                ");
			len=sprintf(buffer,"%i",g_x);	
			TM_HD44780_Puts(0,1,buffer);
			len=sprintf(buffer,"%i",g_y);	
			TM_HD44780_Puts(6,1,buffer);
			len=sprintf(buffer,"%i",g_z);	
			TM_HD44780_Puts(13,1,buffer);
		}

void vib_menu_show(void){
						  sprintf(buffer,"%    i    ",max_g);
						  TM_HD44780_Puts(6,1,buffer); 
}

char key_read(void){

		   HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK)
        {
            ADCValue = HAL_ADC_GetValue(&hadc1);
					if (ADCValue>2400&&ADCValue<2700) return(m_select);
					else if (ADCValue>1600&&ADCValue<2000) return(m_left);
					else if (ADCValue>400&&ADCValue<800) return(m_up);
					else if (ADCValue>1000&&ADCValue<1300) return(m_down);
					else if (ADCValue>=0&&ADCValue<300) return(m_right);
					else return(m_nothing);

					 
        }
		   HAL_ADC_Stop(&hadc1);
			}

void alarm_on(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);
}

void alarm_off(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
}

void lcd_bk_light(char i){
	if (i==1) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
}

void calibration_(void){
	 result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
TM_HD44780_Clear();
TM_HD44780_Puts(0,0," calibrating...");


x_offset=0;
y_offset=0;
z_offset=0;
for (i=1;i<100;i++){
		SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	  x_offset=x_offset+mpu1.Gyroscope_X;
	  y_offset=y_offset+mpu1.Gyroscope_Y;
	  z_offset=z_offset+mpu1.Gyroscope_Z;
  	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
	  HAL_Delay(200);
}
x_offset=x_offset/100;
y_offset=y_offset/100;
z_offset=z_offset/100;



TM_HD44780_Clear();
TM_HD44780_Puts(1,0,"X");
TM_HD44780_Puts(7,0,"Y");
TM_HD44780_Puts(13,0,"Z");
		len=sprintf(buffer,"%i",x_offset);	
		TM_HD44780_Puts(0,1,buffer);
    len=sprintf(buffer,"%i",y_offset);	
		TM_HD44780_Puts(6,1,buffer);
		len=sprintf(buffer,"%i",z_offset);	
		TM_HD44780_Puts(12,1,buffer);
HAL_Delay(3000);
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
