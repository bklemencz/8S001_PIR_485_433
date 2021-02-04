/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "eeprom.h"

#define EV1527_Enabled	FALSE
#define RS485TX_Enabled TRUE
#define BAUDRATE 57600
#define RS485_Byte_Delay_ms 1

#define CONFIG_UNUSED_PINS_STM8S001 \
{ \
 GPIOA->DDR |= GPIO_PIN_2; \
 GPIOB->DDR |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7; \
 GPIOC->DDR |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7; \
 GPIOD->DDR |= GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_7; \
 GPIOE->DDR |= GPIO_PIN_5; \
 GPIOF->DDR |= GPIO_PIN_4; \
}

const uint8_t EV1527_PulseTime_Nominal_10us = 35;
#define F_CPU 16000000UL
#define T_COUNT(x) (( F_CPU * x / 1000000UL )-3)/3

// PIN1 UART1_Rx
// PIN5 PA3 Output RF_Tx
// PIN6 PB4 BTN Input
// PIN7 PC3 Motion sensor Input
// PIN8 SWIM / 485 Output

uint32_t rfcode;
uint8_t GroupID;
uint8_t Mess_Repeat_Time_s;
volatile uint8_t message_delay_s;
bool BTN_EVENT,MOTION_EVENT;

void _delay_us (uint16_t us)
{
	uint16_t i;
	uint16_t cy;
	cy = T_COUNT(us);
	_asm("nop\n $N:\n decw X\n jrne $L\n nop\n ", cy);
	
}

void _delay_ms( uint16_t ms )
{
	while ( ms-- )
	{
		_delay_us( 1000 );
	}
}

void EV1527_SendBit(uint8_t Bit) // Bit = 2 -> Sync
{
    if (Bit == 0)
		{
			GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*10);
            GPIO_WriteLow(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*30);
		}
    if (Bit == 1)
        {
			GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*30);
            GPIO_WriteLow(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*10);
		}
    if (Bit == 2)
		{
			GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*10);
            GPIO_WriteLow(GPIOA,GPIO_PIN_3);
            _delay_us(EV1527_PulseTime_Nominal_10us*310);
		}
}

void EV1527_Transmit(uint32_t Code, uint8_t Length, uint8_t Repeat)
{
    uint8_t nRepeat;
    int8_t i;
    disableInterrupts();
    for (nRepeat = 0; nRepeat < Repeat; nRepeat++)
    {
        for (i = Length - 1; i >= 0; i--)
        {
            if (Code & (1L << i))
                EV1527_SendBit(1);
            else
                EV1527_SendBit(0);
        }
        EV1527_SendBit(2);
    }
    enableInterrupts();
}

void InitTIM4(void)
{
  TIM4->PSCR=0;				//1 frequency division, timer clock equals system clock = 16m

  TIM4->ARR=0XA0;			//10us reload value 0XA0

  TIM4->CNTR=0;				//It is necessary to clear down the counter
  TIM4->IER |= 1<<0;		//Enable tim4 update interrupt
  

  TIM4->SR1  |= 1<<0;		//Clear tim4 update interrupt flag

  TIM4->CR1 |= 1<<7;		//Allow reassembly to enable timer
  TIM4->CR1 |= 1<<0;		//Enabling tim4 counter
}

void io_init(void)
{
	CLK->ICKR = CLK_ICKR_RESET_VALUE;
	CLK->ECKR = CLK_ECKR_RESET_VALUE;
	CLK->SWR = CLK_SWR_RESET_VALUE;
	CLK->SWCR = CLK_SWCR_RESET_VALUE;
	CLK->CKDIVR = CLK_CKDIVR_RESET_VALUE;
	CLK->PCKENR1 = CLK_PCKENR1_RESET_VALUE;
	CLK->PCKENR2 = CLK_PCKENR2_RESET_VALUE;
	CLK->CSSR = CLK_CSSR_RESET_VALUE;
	CLK->CCOR = CLK_CCOR_RESET_VALUE;
	while ((CLK->CCOR & CLK_CCOR_CCOEN) != 0)
	{
	}
	CLK->CCOR = CLK_CCOR_RESET_VALUE;
	CLK->HSITRIMR = CLK_HSITRIMR_RESET_VALUE;
	CLK->SWIMCCR = CLK_SWIMCCR_RESET_VALUE; 			//CLK_DeInit();

	CLK->ECKR &= (uint8_t)(~CLK_ECKR_HSEEN); 			//CLK_HSECmd(DISABLE);
	CLK->ICKR &= (uint8_t)(~CLK_ICKR_LSIEN); 			//CLK_LSICmd(DISABLE);
	CLK->ICKR |= CLK_ICKR_HSIEN;			 			//CLK_HSICmd(ENABLE);
	CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
	CLK->CKDIVR |= (uint8_t)0x00; 						//CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_CPUDIV);
	CLK->CKDIVR |= (uint8_t)((uint8_t)0x80 & (uint8_t)CLK_CKDIVR_CPUDIV); //CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
	FLASH->CR1 = FLASH_CR1_RESET_VALUE;
	FLASH->CR2 = FLASH_CR2_RESET_VALUE;
	FLASH->NCR2 = FLASH_NCR2_RESET_VALUE;
	FLASH->IAPSR &= (uint8_t)(~FLASH_IAPSR_DUL);
	FLASH->IAPSR &= (uint8_t)(~FLASH_IAPSR_PUL);
	(void)FLASH->IAPSR; 											// FLASH_DeInit();
	rfcode = EEPROM_Read4Byte(0);
	GroupID = EEPROM_ReadByte(4);
	Mess_Repeat_Time_s = EEPROM_ReadByte(5);
	GPIO_DeInit(GPIOA);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);		//RF Output PA3
	GPIO_DeInit(GPIOC);
	GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_IN_PU_IT);				//Motion Sensor
	CONFIG_UNUSED_PINS_STM8S001;
	if (RS485TX_Enabled)
	{
		_delay_ms(10000); //Wait 5s to turn off SWIM
		CFG->GCR |= 0x01; // disable SWIM interface
		disableInterrupts();
		GPIO_DeInit(GPIOB);
		GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_LOW_FAST);	//RS485 Rx Enable
		UART1_DeInit();
		UART1_Init(BAUDRATE, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
		UART1_ITConfig(UART1_IT_RXNE_OR, DISABLE); // Receive IT disabled, if used later enable
		UART1_Cmd(ENABLE);
	}
	disableInterrupts();
	EXTI_DeInit();
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC,EXTI_SENSITIVITY_RISE_FALL);	//IT for motion sensor

	TIM4_DeInit();
	TIM4_TimeBaseInit(TIM4_PRESCALER_128,0x7D);
	TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);
	TIM4_Cmd(ENABLE);
	enableInterrupts();	
}

void UART_Sendbyte(char bytetosend)
{
		UART1->DR = (unsigned char) bytetosend; 									//  Put the next character into the data transmission register.
		while ((UART1->SR & (u8) UART1_FLAG_TXE) == RESET); 						//  Wait for transmission to complete.
}


void RS485_Transmit_Motion(uint8_t Group)
{
	GPIO_WriteHigh(GPIOB,GPIO_PIN_4);							//RS485 Tx Enable
	UART_Sendbyte('P'); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte(Group); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte('C'); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte('M'); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte('0'); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte('0'); //_delay_ms(RS485_Byte_Delay_ms);
	UART_Sendbyte('\r'); //_delay_ms(RS485_Byte_Delay_ms);
	_delay_ms(1);
	GPIO_WriteLow(GPIOB,GPIO_PIN_4);							//RS485 Rx Enable
}


main()
{
	io_init();
	if (RS485TX_Enabled) RS485_Transmit_Motion(GroupID);
	while (1)
	{
		if (MOTION_EVENT && (message_delay_s==0))
		{
			if (EV1527_Enabled) EV1527_Transmit(rfcode, 24, 25);
			if (RS485TX_Enabled) RS485_Transmit_Motion(GroupID);
			message_delay_s = Mess_Repeat_Time_s;
		}
	}
}