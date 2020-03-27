/* ###*B*###
 * ERIKA Enterprise - a tiny RTOS for small microcontrollers
 *
 * Copyright (C) 2002-2013  Evidence Srl
 *
 * This file is part of ERIKA Enterprise.
 *
 * ERIKA Enterprise is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation,
 * (with a special exception described below).
 *
 * Linking this code statically or dynamically with other modules is
 * making a combined work based on this code.  Thus, the terms and
 * conditions of the GNU General Public License cover the whole
 * combination.
 *
 * As a special exception, the copyright holders of this library give you
 * permission to link this code with independent modules to produce an
 * executable, regardless of the license terms of these independent
 * modules, and to copy and distribute the resulting executable under
 * terms of your choice, provided that you also meet, for each linked
 * independent module, the terms and conditions of the license of that
 * module.  An independent module is a module which is not derived from
 * or based on this library.  If you modify this code, you may extend
 * this exception to your version of the code, but you are not
 * obligated to do so.  If you do not wish to do so, delete this
 * exception statement from your version.
 *
 * ERIKA Enterprise is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with ERIKA Enterprise; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA.
 * ###*E*### */

/*
 * Trying to use several tasks, alarm, event, linked with interrupt and event.
 *
 * Use case: users can send data over the usart channel from the pc to the board.
 * When some data are received, the microcontroller store these data in an array,
 * RxMsg[20], so this array store the latest received message.
 * When the user will push the button, an event will be raised causing the unlock
 * of the print operation, served by a task (TaskLcd).
 * The board also works as a mirror, it means that all the messages received
 * on the usart3 will be forwarded back to the pc. This action is managed by the
 * TaskReceiver that once received the entire message will raise the event that
 * will unlock the TaskSend.
 *
 * Author: Andrea Toppi.
 *
 */

#include "ee.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include "defines.h"
#include "tm_stm32f4_hd44780.h"

USART_InitTypeDef usart3;
GPIO_InitTypeDef usart_tx;
GPIO_InitTypeDef usart_rx;

int toggle_counter = 0;		//	Toggle counter stop(0)/play(1)
int lcd_counter = 0;		//	Lcd reset counter stop(0)/play(1)
int n_toggle = 0;			//	ToggleRxLed counter

EE_UINT8 RxMsg[20];			//	rx message
EE_UINT8 rx_msg_length = 0;	//	Length of the received message
EE_UINT8 i = 0;				//	Receiver counter
EE_UINT8 ch;				//	rx buffer
EE_UINT8 n = '\n';			//	End of message flag

DeclareTask(TaskLcd);
DeclareTask(TaskLedBlink);
DeclareTask(TaskSend);
DeclareTask(TaskButton);
DeclareEvent(Print);
DeclareEvent(Receive);
DeclareEvent(Send);

/*
 * This function put the default message on the lcd.
 * */
void PrintHello(){
	//Put string to LCD
	TM_HD44780_Clear();
	TM_HD44780_Puts(0, 0, "STM32F4/29 Discovery");
	TM_HD44780_Puts(2, 1, "20x4 HD44780 LCD");
	TM_HD44780_Puts(0, 2, "Test Event Interrupt");
	lcd_counter = 0;
}

/*
 * USARTx configured as follow:
		- BaudRate = 9600 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		// STM32 USART IRQ TX/RX Loop (USART3 Tx PD.8, Rx PD.9) STM32F4 Discovery
*/
void Usart3_Init()
{
	/* USART3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	usart_tx.GPIO_Pin = GPIO_Pin_8;
	usart_tx.GPIO_Mode = GPIO_Mode_AF;
	usart_tx.GPIO_OType = GPIO_OType_PP;
	usart_tx.GPIO_PuPd = GPIO_PuPd_NOPULL;
	usart_tx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &usart_tx);

	usart_rx.GPIO_Pin = GPIO_Pin_9;
	usart_rx.GPIO_Mode = GPIO_Mode_AF;
	usart_rx.GPIO_OType = GPIO_OType_PP;
	usart_rx.GPIO_PuPd = GPIO_PuPd_NOPULL;
	usart_rx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &usart_rx);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	USART_StructInit(&usart3);
	USART_Init(USART3, &usart3);
	USART_Cmd(USART3, ENABLE);
}

/*
 * Callback function called when the DelayOne alarm is raised, at the expiring
 * of the delay counter.
 * Allows leds to set up/down 2 times.
 * */
void ToggleRxLed(){
	STM_EVAL_LEDToggle(LED6);
	n_toggle++;
	if (n_toggle == 4){
		toggle_counter = 0;
		n_toggle = 0;
	}
}

/*
 * Set up the toggle counter to count a 200ms delays.
 * Called by TaskReceive()
 * */
void RxToggle(Led_TypeDef led){
	toggle_counter = 1;
}

/*
 * Put out a message over usart3 serial port
 * */
void ConsoleOut(char* str)
{
	EE_UINT8 i = 0;

	while (str[i] != '\n') {
		USART_SendData(USART3, (uint8_t) str[i++]);
		STM_EVAL_LEDToggle(LED3);
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
	if (str[i] == '\n'){
		USART_SendData(USART3, n);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
}

/*
 * Activated when the rx buffer is not empty. Store sequences of data in a
 * RxMsg[20] circular array.
 * */
void StoreData(){
	ch = USART_ReceiveData(USART3);
	if (i == 20)
		i = 0;

	if (ch != n){
		RxMsg[i] = ch;
		i++;
	}
	else{
		RxMsg[i] = ch;
		rx_msg_length = i;
		SetEvent(TaskSend,Send);
		i = 0;
	}
}

/*
 * Called by the TaskLcd to print on the display the last received message.
 * */
void PrintData(){
	EE_UINT8 l = 0;
	TM_HD44780_Clear();
	TM_HD44780_Puts(2, 0, "Msg ricevuto:");
	while(l < rx_msg_length){
		TM_HD44780_Puts(l,1,&RxMsg[l]);
		l++;
	}
	lcd_counter = 1;
}

/*
 * SysTick ISR2
 */
ISR2(systick_handler)
{
	/* count the interrupts, waking up expired alarms */
	CounterTick(counterOne);
	if (toggle_counter)
		CounterTick(delay);
	if (lcd_counter)
		CounterTick(resetLcd);
}

/*
 * Button ISR2
 */
ISR1(exti0_irq_handler)
{
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	ActivateTask(TaskButton);
}

/*
 * Usart3_rx ISR2
 */
ISR1(usart3_irq_handler)
{
	SetEvent(TaskReceive,Receive);
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}

/*
 * This task is activated when the button interrupt comes. Set up the event print
 * */
TASK(TaskButton)
{
	SetEvent(TaskLcd,Print);
}

/**
 * This task puts RxMsg on lcd when Print event is set up.
 */
TASK(TaskLcd){

	EE_UINT8 i = 0;
	EventMaskType current;

	while(1){
		WaitEvent(Print);

		PrintData();

		GetEvent(Print, &current); /* save event */
		ClearEvent(Print);
	}
	TerminateTask();
}

/**
 * This task blinks led 4.
 */
TASK(TaskLedBlink)
{
	STM_EVAL_LEDToggle(LED4);
}

/**
 * Task used to send back over the usart the entire received message.
 */
TASK(TaskSend){
	EventMaskType mask;

	while(1){
		WaitEvent(Send);

		ConsoleOut(RxMsg);	// Send back to the usart the message received

		GetEvent(Send, &mask); /* save event */
		ClearEvent(Send);
	}
}

/**
 * Task used to receive data from buffer and store into an array: RxMsg.
 */
TASK(TaskReceive){
	EventMaskType mask;

	while(1){
		WaitEvent(Receive);

		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
		RxToggle(LED6);
		StoreData();

		GetEvent(Receive, &mask); /* save event */
		ClearEvent(Receive);
	}
}

int main(void)
{
	/*
	 * Setup the microcontroller system.
	 * Initialize the Embedded Flash Interface, the PLL and update the
	 * SystemFrequency variable.
	 * For default settings look at:
	 * pkg/mcu/st_stm32_stm32f4xx/src/system_stm32f4xx.c
	 */
	SystemInit();

	/*Initialize Erika related stuffs*/
	EE_system_init();

	/*Initialize systick */
	EE_systick_set_period(MILLISECONDS_TO_TICKS(1, SystemCoreClock));
	EE_systick_enable_int();
	EE_systick_start();

	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	Usart3_Init();

	//usart interrupt enabling on reception
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	//Initialize LCD 20 cols x 4 rows
	TM_HD44780_Init(20, 4);

	PrintHello();

	/* Output message */
	//ConsoleOut("Hello From Erika RTOS\r\n");

	ActivateTask(TaskLcd);
	ActivateTask(TaskReceive);
	ActivateTask(TaskSend);

	/* Program cyclic alarms which will fire after an initial offset,
	 * and after that periodically
	 * */
	SetRelAlarm(AlarmLedBlink, 10, 500);
	SetRelAlarm(DelayOne,10,100);
	SetRelAlarm(ResetLcd,10,2000);

	/* Forever loop: background activities (if any) should go here */
	for (;;);

}

