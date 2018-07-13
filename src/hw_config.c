/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\hw_config.c
  * @author  T.O.M.A.S. Team
  * @version V1.0.0
  * @date    05-October-2011
  * @brief   Hardware initialization
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Library includes. */
#include "hw_config.h"

volatile uint16_t ADCConvertedValues[2];


/*-----------------------------------------------------------*/
void prvSetupHardware( void )
{
	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure LED IOs as output push-pull */
    /* Initialize LEDs on STM32F4_Discovery board */
	prvLED_Config(GPIO);
	/* Configure User button pin (PA0) as external interrupt -> modes switching */
	//STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_EXTI);

	/* Configuration of Timer4 to control LEDs based on MEMS data */
	//prvTIM4_Config();

	/* Configure LIS302 in order to produce data used for TIM4 reconfiguration and LED control */
	//prvMEMS_Config();
}

void prvLED_Config(char state)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  /* Configure PD12, PD13, PD14 and PD15 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3; //GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;*/

  if(state==GPIO)
  {
	  /* standard output pin */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  //GPIO_Init(GPIOD, &GPIO_InitStructure);
  }
  else
  {
	  /*-------------------------- GPIO Configuration ----------------------------*/
	  /* GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull - alternative mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  /* Connect TIM4 pins to AF2 */
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  }
}


void prvMEMS_Config(void)
{
	uint8_t ctrl = 0;
	uint32_t i=0;		//simple index for software delay

	LIS302DL_InitTypeDef  LIS302DL_InitStruct;
	LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;

	/* Set configuration of LIS302DL*/
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
	LIS302DL_Init(&LIS302DL_InitStruct);

	/* Set configuration of Internal High Pass Filter of LIS302DL*/
	LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
	LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

	/* Required delay for the MEMS Accelerometer: Turn-on time = 3/Output data Rate
	                                                            = 3/100 = 30ms */
	for(i=0;i<0x1FFFF;i++);

	/* Configure Interrupt control register: enable Click interrupt1 */
	ctrl = 0x07;
	LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

	/* Enable Interrupt generation on click/double click on Z axis */
	ctrl = 0x70;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

	/* Configure Click Threshold on X/Y axis (10 x 0.5g) */
	ctrl = 0xAA;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

	/* Configure Click Threshold on Z axis (10 x 0.5g) */
	ctrl = 0x0A;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);

	/* Configure Time Limit */
	ctrl = 0x03;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);

	/* Configure Latency */
	ctrl = 0x7F;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);

	/* Configure Click Window */
	ctrl = 0x7F;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
}

/**
  * @brief  MEMS accelerometer management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error has occured */
  while (1)
  {
  }
}

void prvTIM4_Config(void)
{
  uint16_t PrescalerValue = 0;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:

    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1 (APB1 Prescaler = 4, see system_stm32f4xx.c file).
      TIM4CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM4CLK = 2*(HCLK / 4) = HCLK/2 = SystemCoreClock/2

    To get TIM4 counter clock at 2 KHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM1 counter clock) - 1
       Prescaler = (168 MHz/(2 * 2 KHz)) - 1 = 41999

    To get TIM4 output clock at 1 Hz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1
           = 1999

    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 50%
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 50%
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 50%

    ==> TIM4_CCRx = TIM4_ARR/2 = 1000  (where x = 1, 2, 3 and 4).

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;//TIM_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_CCR;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel2 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel3 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Output Compare PWM1 Mode configuration: Channel4 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void USART3Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//enable bus clocks, USART1 is APB2, others are APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* enable the peripheral clock for the pins used by
	 * USART2, PC10 for TX and PC11 for RX*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers


	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStructure.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);

}

void USART1Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//enable bus clocks, USART1 is APB2, others are APB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* enable the peripheral clock for the pins used by
	 * USART2, PC10 for TX and PC11 for RX*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers


	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStructure.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

}

void init_tim()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    /* Time base configuration */
    	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1; // 0..239
    	TIM_TimeBaseStructure.TIM_Period = 65535;//100000 - 1; // 0..999
    	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
        TIM_Cmd(TIM4, ENABLE);

}

void USART2Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//enable bus clocks, USART1 is APB2, others are APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* enable the peripheral clock for the pins used by
	 * USART2, PC10 for TX and PC11 for RX*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART2 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers


	/* The RX and TX pins are now connected to their AF
	 * so that the USART2 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStructure.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting



	USART_Cmd(USART2, ENABLE);

}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
