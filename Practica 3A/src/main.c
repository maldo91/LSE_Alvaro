/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )
#define timer_PERIOD_MS						( 100 / portTICK_RATE_MS )
#define mainQUEUE_LENGTH					( 20 )

static void SetupHardware( void );
static void ledsTask( void *pvParameters );
static void debounce( void *pvParameters );

static xQueueHandle xQueue;
static xSemaphoreHandle event_signal_1;
static xSemaphoreHandle event_signal_2;
static xSemaphoreHandle event_signal_3;
static xSemaphoreHandle event_signal_4;

static void SetupHardware( void );
static void leds_Config(void);
void EXTILine1_Config(void);
void EXTILine2_Config(void);
void EXTILine3_Config(void);
void EXTILine4_Config(void);
void init_usart(void);

GPIO_InitTypeDef  GPIO_Leds;
GPIO_InitTypeDef  GPIO_Buttons;
EXTI_InitTypeDef  EXTI_InitStructure;
/*-----------------------------------------------------------*/

int main(void)
{
	SetupHardware();
	vSemaphoreCreateBinary( event_signal_1 ); // Create the semaphore
	vSemaphoreCreateBinary( event_signal_2 );
	vSemaphoreCreateBinary( event_signal_3 );
	vSemaphoreCreateBinary( event_signal_4 );
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( uint32_t ) );
	xTaskCreate(ledsTask,( signed char * ) "leds",configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate(debounce,( signed char * ) "debounce",configMINIMAL_STACK_SIZE+1, NULL, 1, NULL );
	vTaskStartScheduler();

	for( ;; );
}
/*-----------------------------------------------------------*/



static void ledsTask( void *pvParameters )
{
uint32_t ReceivedValue;

	for( ;; )
	{
		if (xQueueReceive( xQueue, &ReceivedValue, portMAX_DELAY ))
		{
			switch(ReceivedValue)
			{
			case 1:
				GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
				break;
			case 2:
				GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
				break;
			case 3:
				GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
				break;
			case 4:
				GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
				break;
			default:
				break;
			}
		}
		else
		{
			//Error, no received data
		}
		vTaskDelay(200);
	}
	vTaskDelete(NULL);
}
/*-----------------------------------------------------------*/
static void debounce( void *pvParameters )
{
	for( ;; )
		{
		EXTI_ClearFlag(EXTI_Line1);
		EXTI_ClearFlag(EXTI_Line2);
		EXTI_ClearFlag(EXTI_Line3);
		EXTI_ClearFlag(EXTI_Line4);
		xSemaphoreGive(event_signal_1);
		xSemaphoreGive(event_signal_2);
		xSemaphoreGive(event_signal_3);
		xSemaphoreGive(event_signal_4);
		vTaskDelay(200); //debounce delay

		}

}
/*-----------------------------------------------------------*/

static void SetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	init_usart(); //USART initialization

    leds_Config();
	EXTILine1_Config();  // Buttons interruptions configuration
	EXTILine2_Config();
	EXTILine3_Config();
	EXTILine4_Config();

	GPIO_SetBits(GPIOD, GPIO_Pin_12); //Leds are lighting at the beginning
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
}

static void leds_Config(void)
{
	  /* GPIOD Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	  GPIO_Leds.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	  GPIO_Leds.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_Leds.GPIO_OType = GPIO_OType_PP;
	  GPIO_Leds.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_Leds.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_Leds);
}

void EXTILine1_Config(void)
{

  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PD1 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PD1 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);

  /* Configure EXTI Line1 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line1 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTILine2_Config(void)
{

  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PD2 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PD2 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

  /* Configure EXTI Line2 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTILine3_Config(void)
{

  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PD3 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PD3 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3);

  /* Configure EXTI Line3 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line3 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTILine4_Config(void)
{

  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOD clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PD4 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PD4 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* Handle PD1 interrupt */
void EXTI1_IRQHandler(void) {
	uint32_t value = 1;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    	if(xSemaphoreTakeFromISR(event_signal_1,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
    	}
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/* Handle PD2 interrupt */
void EXTI2_IRQHandler(void) {
	uint32_t value = 2;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
    	if(xSemaphoreTakeFromISR(event_signal_2,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
		}
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/* Handle PD3 interrupt */
void EXTI3_IRQHandler(void) {
	uint32_t value = 3;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
    	if(xSemaphoreTakeFromISR(event_signal_3,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
		}
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

/* Handle PD4 interrupt */
void EXTI4_IRQHandler(void) {
	uint32_t value = 4;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
    	if(xSemaphoreTakeFromISR(event_signal_4,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
		}
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void init_usart(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration:  USART2 TX on PA2 and RX on PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	// TX = PA2, RX = PA3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE); // enable USART2

}

// this is the interrupt request handler (IRQ) for ALL USART2 interrupts
void USART2_IRQHandler(void){
	if( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ){
		char character;
		uint32_t value;
		character = (USART_ReceiveData(USART2));
		if (character == '1' ||character == '2'|| character == '3' || character == '4')
		{
			value = (uint32_t) character - 48; //Convert ascii to integer
			xQueueSendFromISR( xQueue, &value, 0 );
		}
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
void vApplicationTickHook( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;

	/* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
	1 in FreeRTOSConfig.h.


	/* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
	normally be performed before leaving the interrupt (because during the
	execution of the interrupt a task of equal or higher priority than the
	running task was unblocked).  The syntax required to context switch from
	an interrupt is port dependent, so check the documentation of the port you
	are using.  http://www.freertos.org/a00090.html

	In this case, the function is running in the context of the tick interrupt,
	which will automatically check for the higher priority task to run anyway,
	so no further action is required. */
}
/*-----------------------------------------------------------*/
